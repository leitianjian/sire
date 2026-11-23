#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <aris/ext/json.hpp>

#include "contact_trajectory.hpp"
#include "cuda_test_buffer.hpp"
#include "physics/contact/contact_time_spectrum.hpp"
#include "sire/core/force_screw.hpp"

namespace {

using sire::physics::contact::cuda::SeparationSearch;
using sire::physics::contact::cuda::TrajectoryStatus;
using sire::physics::contact::cuda::launchContactSeparations;
using sire::physics::contact::cuda::launchContactTrajectories;
using sire::physics::contact::cuda::separationWorkspaceSize;
using sire::physics::contact::cuda::trajectoryWorkspaceSize;
using sire::physics::contact::detail::contactTimeGrid;
using sire::physics::contact::detail::contactTimeSpectrum;

using sire::test::DeviceBuffer;
using sire::test::checkCuda;

struct SeparationReference {
  double horizon;
  double expected;
};

struct Inputs {
  std::string label;
  std::vector<double> matrix;
  std::vector<double> bias;
  std::vector<double> initial;
  double time;
  std::vector<double> expected;
  TrajectoryStatus status{TrajectoryStatus::Success};
  std::optional<SeparationReference> separation{};
};

auto cpuReference(const Inputs& input) -> std::vector<double> {
  const auto n = input.initial.size();
  const auto augmented = n + 1;
  std::vector<double> matrix(augmented * augmented, 0.0);
  std::vector<double> transition(matrix.size());
  for (std::size_t row = 0; row < n; ++row) {
    std::copy_n(input.matrix.data() + row * n, n, matrix.data() + row * augmented);
    matrix[row * augmented + n] = input.bias[row];
  }
  sire::core::screw::MatrixExpPadeWorkspace(augmented).apply(
      matrix.data(), transition.data(), input.time);
  std::vector<double> result(n);
  for (std::size_t row = 0; row < n; ++row) {
    const auto* coefficients = transition.data() + row * augmented;
    for (std::size_t col = 0; col < n; ++col) {
      result[row] += coefficients[col] * input.initial[col];
    }
    result[row] += coefficients[n];
  }
  return result;
}

auto readFrame(const char* path) -> Inputs {
  std::ifstream file(path);
  if (!file) throw std::runtime_error(std::string("Cannot read ") + path);
  nlohmann::json frame;
  file >> frame;
  const auto& trajectory = frame.at("normal_trajectory");
  Inputs input{path, trajectory.at("A").get<std::vector<double>>(),
          trajectory.at("b").get<std::vector<double>>(),
          trajectory.at("x0").get<std::vector<double>>(), frame.at("h").get<double>(),
          trajectory.at("state_at_h").get<std::vector<double>>()};
  if (trajectory.value("separation_method", std::string{}) == "exponential") {
    input.separation = SeparationReference{trajectory.at("suggest_dt").get<double>(),
                                          trajectory.at("separation_time").get<double>()};
  }
  return input;
}

void compare(const std::string& label, double value, double expected) {
  const double tolerance = 1e-11 + 2e-10 * std::abs(expected);
  if (!std::isfinite(value) || !std::isfinite(expected) ||
      std::abs(value - expected) > tolerance) {
    std::cerr.precision(17);
    std::cerr << label << ": GPU " << value << ", CPU " << expected << '\n';
    throw std::runtime_error("Contact trajectory values disagree");
  }
}

auto runBatch(const std::vector<Inputs>& batch, std::size_t n) -> std::size_t {
  std::vector<double> matrix, bias, initial, time;
  for (const auto& input : batch) {
    if (input.matrix.size() != n * n || input.bias.size() != n ||
        input.initial.size() != n ||
        (!input.expected.empty() && input.expected.size() != n)) {
      throw std::runtime_error("Wrong trajectory shape for " + input.label);
    }
    matrix.insert(matrix.end(), input.matrix.begin(), input.matrix.end());
    bias.insert(bias.end(), input.bias.begin(), input.bias.end());
    initial.insert(initial.end(), input.initial.begin(), input.initial.end());
    time.push_back(input.time);
  }
  const auto size = trajectoryWorkspaceSize(batch.size(), n / 2);
  DeviceBuffer<double> gpu_matrix(matrix.size()), gpu_bias(bias.size());
  DeviceBuffer<double> gpu_initial(initial.size()), gpu_time(time.size());
  DeviceBuffer<double> gpu_state(initial.size()), gpu_workspace(size);
  DeviceBuffer<TrajectoryStatus> gpu_status(batch.size());
  gpu_matrix.upload(matrix);
  gpu_bias.upload(bias);
  gpu_initial.upload(initial);
  gpu_time.upload(time);
  constexpr double untouched = -184.25;
  gpu_state.upload(std::vector<double>(initial.size(), untouched));

  if (launchContactTrajectories(batch.size(), n / 2, gpu_matrix.data(),
          gpu_bias.data(), gpu_initial.data(), gpu_time.data(), gpu_state.data(),
          gpu_status.data(), gpu_workspace.data(), size - 1, nullptr) != cudaErrorInvalidValue) {
    throw std::runtime_error("Short device workspace was accepted");
  }
  checkCuda(launchContactTrajectories(batch.size(), n / 2, gpu_matrix.data(),
      gpu_bias.data(), gpu_initial.data(), gpu_time.data(), gpu_state.data(),
      gpu_status.data(), gpu_workspace.data(), size, nullptr));
  checkCuda(cudaDeviceSynchronize());

  const auto state = gpu_state.download();
  const auto status = gpu_status.download();
  for (std::size_t index = 0; index < batch.size(); ++index) {
    const auto& input = batch[index];
    if (status[index] != input.status) {
      throw std::runtime_error("Unexpected GPU status for " + input.label);
    }
    if (input.status == TrajectoryStatus::Success) {
      const auto reference = input.expected.empty() ? cpuReference(input) : input.expected;
      for (std::size_t col = 0; col < n; ++col) {
        compare(input.label, state[index * n + col], reference[col]);
      }
    } else {
      for (std::size_t col = 0; col < n; ++col) {
        if (state[index * n + col] != untouched) {
          throw std::runtime_error("GPU overwrote an invalid trajectory: " + input.label);
        }
      }
    }
  }

  std::vector<SeparationSearch> searches(batch.size(), {0, 0, 0});
  std::size_t search_count = 0;
  for (std::size_t index = 0; index < batch.size(); ++index) {
    const auto& input = batch[index];
    if (!input.separation) continue;
    const auto spectrum = contactTimeSpectrum(n, input.matrix.data());
    searches[index] = {input.separation->horizon, spectrum.rate, spectrum.frequency};
    ++search_count;
  }
  if (search_count == 0) return 0;
  const auto separation_size = separationWorkspaceSize(batch.size(), n / 2);
  DeviceBuffer<SeparationSearch> gpu_searches(batch.size());
  DeviceBuffer<double> gpu_separation(batch.size()), gpu_separation_work(separation_size);
  gpu_searches.upload(searches);
  gpu_separation.upload(std::vector<double>(batch.size(), untouched));
  checkCuda(launchContactSeparations(batch.size(), n / 2, gpu_matrix.data(),
      gpu_bias.data(), gpu_initial.data(), gpu_searches.data(), gpu_separation.data(),
      gpu_status.data(), gpu_separation_work.data(), separation_size, nullptr));
  checkCuda(cudaDeviceSynchronize());
  const auto roots = gpu_separation.download();
  const auto root_status = gpu_status.download();
  for (std::size_t index = 0; index < batch.size(); ++index) {
    const auto& input = batch[index];
    const double root = roots[index];
    if (!input.separation) {
      if (root != untouched || root_status[index] != TrajectoryStatus::InvalidInput) {
        throw std::runtime_error("Invalid root input was accepted: " + input.label);
      }
      continue;
    }
    const auto& search = searches[index];
    const double expected = input.separation->expected;
    const auto grid = contactTimeGrid(search.horizon, {search.spectral_rate,
                                                       search.spectral_frequency});
    const bool matches = expected == -1.0 ? root == -1.0 :
        (root > 0.0 && root <= search.horizon && std::abs(root - expected) <= grid.tolerance);
    if (root_status[index] != TrajectoryStatus::Success || !matches) {
      std::cerr.precision(17);
      std::cerr << input.label << ": GPU separation " << root << ", reference " << expected << '\n';
      throw std::runtime_error("Contact separation times disagree");
    }
  }
  return search_count;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    int devices = 0;
    const auto status = cudaGetDeviceCount(&devices);
    if (status == cudaErrorNoDevice || status == cudaErrorInsufficientDriver ||
        (status == cudaSuccess && devices == 0)) {
      std::cout << "CUDA device unavailable\n";
      return 77;
    }
    checkCuda(status);
    if (trajectoryWorkspaceSize(std::numeric_limits<std::size_t>::max(), 1) != 0 ||
        trajectoryWorkspaceSize(1, std::numeric_limits<std::size_t>::max()) != 0) {
      throw std::runtime_error("Workspace size overflow was accepted");
    }
    checkCuda(launchContactTrajectories(0, 0, nullptr, nullptr, nullptr, nullptr,
                                       nullptr, nullptr, nullptr, 0, nullptr));
    const double nan = std::numeric_limits<double>::quiet_NaN();
    std::map<std::size_t, std::vector<Inputs>> batches;
    batches[2] = {
        {"affine separation", {0, 1, 0, 0}, {0, -2}, {0.1, 0.4}, 0.5, {0.05, -0.6}},
        {"zero time", {0, 1, -100, -2}, {0, 1}, {0.1, 0.4}, 0.0, {0.1, 0.4}},
        {"stiff contact", {0, 1, -2e8, -50000}, {0, -9.81}, {1e-5, 0.2}, 0.001, {}},
        {"invalid time", {0, 1, 0, 0}, {0, 0}, {0.1, 0.4}, nan, {},
         TrajectoryStatus::InvalidInput},
        {"invalid matrix", {0, 1, nan, 0}, {0, 0}, {0.1, 0.4}, 0.001, {},
         TrajectoryStatus::InvalidInput},
        {"overflowing state", {0, 1, 0, 1000}, {0, 0}, {0.1, 0.4}, 1.0, {},
         TrajectoryStatus::NumericalFailure},
    };
    // Exercise independent scratch and event outcomes at the training target
    // batch size, with a partially occupied final CUDA block.
    const double pi = sire::PI;
    for (int i = 0; i < 1024; ++i) {
      const double time = i * 0.0005;
      const double horizon = 0.05 + i * 0.0004;
      batches[2].push_back({"oscillator", {0, 1, -100, 0}, {0, 0}, {0, 1},
                            time, {std::sin(10 * time) / 10, std::cos(10 * time)},
                            TrajectoryStatus::Success,
                            SeparationReference{horizon, horizon > pi / 10 ? pi / 10 : -1.0}});
    }
    const double late_coefficient = std::exp(0.99);
    auto& single = batches[2];
    single.push_back({"initial zero remains in contact", {0, 1, -100, 0},
        {0, 0}, {0, 1}, 0.1, {}, TrajectoryStatus::Success, SeparationReference{0.1, -1.0}});
    single.push_back({"underdamped separation", {0, 1, -100, 0},
        {0, 0}, {0, 1}, 0.5, {}, TrajectoryStatus::Success, SeparationReference{0.5, pi / 10}});
    single.push_back({"overdamped separation", {0, 1, -2, -3},
        {0, 0}, {1, -3}, 1.0, {}, TrajectoryStatus::Success, SeparationReference{1.0, std::log(2.0)}});
    single.push_back({"shortened final scan interval", {0, 1, -2, -3},
        {0, 0}, {late_coefficient - 1, 1 - 2 * late_coefficient}, 1.0, {},
        TrajectoryStatus::Success, SeparationReference{1.0, 0.99}});
    single.push_back({"very short positive separation", {0, 1, 0, 0},
        {0, 0}, {1e-9, -1}, 1e-8, {}, TrajectoryStatus::Success, SeparationReference{1e-8, 1e-9}});
    single.push_back({"stiff separation", {0, 1e4, -2e4, -5e3}, {0, 0}, {0, 1}, 1e-3, {},
        TrajectoryStatus::Success, SeparationReference{1e-3, pi / std::sqrt(2e8 - 2.5e3 * 2.5e3)}});
    batches[4].push_back(
        {"coupled contacts", {0, 0, 1, 0, 0, 0, 0, 1, -200, 35, -6, 1, 21, -350, 2, -8},
         {0, 0, -9.81, -5}, {0.002, 0.005, 0.1, -0.2}, 0.006, {}});
    const std::vector<double> free_depth{0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0};
    batches[4].push_back({"flat zero and another crossing", free_depth, {0, 0, 0, 0},
        {0, 0.503, 0, -1}, 1.0, {}, TrajectoryStatus::Success, SeparationReference{1.0, 0.503}});
    batches[4].push_back({"earlier of two crossings", free_depth, {0, 0, 0, 0},
        {0.503, 5.1, -1, -10}, 1.0, {}, TrajectoryStatus::Success, SeparationReference{1.0, 0.503}});
    for (int i = 1; i < argc; ++i) {
      auto frame = readFrame(argv[i]);
      batches[frame.initial.size()].push_back(std::move(frame));
    }
    std::size_t count = 0, search_count = 0;
    for (const auto& [dimension, batch] : batches) {
      if (dimension == 0 || dimension % 2 != 0) {
        throw std::runtime_error("Expected depth and depth rate for each contact");
      }
      search_count += runBatch(batch, dimension);
      count += batch.size();
    }
    std::cout << "Validated " << count << " CUDA contact trajectories and "
              << search_count << " separation searches\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
