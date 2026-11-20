#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <aris/ext/json.hpp>

#include "cuda_test_buffer.hpp"
#include "physics/contact/contact_time_spectrum.hpp"
#include "shifted_contact_step.hpp"
#include "sire/core/force_screw.hpp"
#include "sire/physics/contact/simple_admm_contact_solver.hpp"
#include "sire/physics/contact/sor_contact_solver.hpp"
#include "spectral_admm.hpp"

namespace {

using sire::physics::contact::cuda::AdmmInputs;
using sire::physics::contact::cuda::AdmmOutputs;
using sire::physics::contact::cuda::AdmmProblem;
using sire::physics::contact::cuda::AdmmResult;
using sire::physics::contact::cuda::AdmmStatus;
using sire::physics::contact::cuda::ContactMaterialState;
using sire::physics::contact::cuda::NormalContactResult;
using sire::physics::contact::cuda::TrajectoryStatus;
using sire::physics::contact::cuda::ShiftedContactOutputs;
using sire::physics::contact::cuda::ShiftedContactScratch;
using sire::physics::contact::cuda::launchBodyContactStep;
using sire::physics::contact::cuda::launchShiftedContactStep;
using sire::physics::contact::cuda::launchShiftedSpectralAdmm;
using sire::physics::contact::cuda::separationWorkspaceSize;
using sire::physics::contact::cuda::spectralAdmmWorkspaceSize;
using sire::physics::contact::detail::contactTimeGrid;
using sire::physics::contact::detail::contactTimeSpectrum;
using sire::physics::contact::sor_contact::findSinglePointContactEndTime;
using sire::physics::contact::simple_admm::cptContactForceShiftedSpectralAdmm;
using sire::physics::contact::simple_admm::cptContactForceShiftedSpectralAdmmWithWarmStart;
using sire::test::DeviceBuffer;
using sire::test::checkCuda;

struct NormalCase {
  std::vector<double> matrix, bias, initial;
  double horizon;
  double separation;
};

struct BodyCase {
  std::vector<double> inverse_inertia, free_acceleration;
  std::vector<ContactMaterialState> material;
};

struct Case {
  std::string label;
  AdmmProblem problem{0, 1000, 0.001, 1e-8, false, false};
  std::vector<double> friction, inverse_mass, velocity, target, acceleration;
  std::vector<double> initial_force, initial_dual, force, dual;
  double residual{0};
  AdmmStatus status{AdmmStatus::Converged};
  std::optional<NormalCase> normal;
  std::optional<BodyCase> body;
};

enum class StepInput { Admm, Normal, Body };

auto readFrame(const char* path) -> Case {
  std::ifstream file(path);
  if (!file) throw std::runtime_error(std::string("Cannot read ") + path);
  nlohmann::json frame;
  file >> frame;
  const auto& admm = frame.at("shifted_spectral_admm");
  Case input{};
  input.label = path;
  input.problem = {frame.at("n").get<std::size_t>(), frame.at("solver_max_iters").get<std::size_t>(),
                   frame.at("h").get<double>(), frame.at("solver_max_error").get<double>(),
                   !admm.at("initial_force").is_null(), !admm.at("initial_dual").is_null()};
  input.friction = frame.at("fri_coef").get<std::vector<double>>();
  input.inverse_mass = frame.at("invM").get<std::vector<double>>();
  input.velocity = frame.at("v0").get<std::vector<double>>();
  input.target = frame.at("v_target").get<std::vector<double>>();
  input.acceleration = frame.at("b").get<std::vector<double>>();
  input.force = frame.at("cResult").get<std::vector<double>>();
  input.dual = admm.at("final_dual").get<std::vector<double>>();
  if (input.problem.has_force_guess) {
    input.initial_force = admm.at("initial_force").get<std::vector<double>>();
  }
  if (input.problem.has_dual_guess) {
    input.initial_dual = admm.at("initial_dual").get<std::vector<double>>();
  }
  input.residual = frame.at("solver_error").get<double>();
  if (input.residual > input.problem.tolerance) input.status = AdmmStatus::IterationLimit;
  const auto& normal = frame.at("normal_trajectory");
  if (normal.value("separation_method", std::string{}) == "exponential") {
    input.normal = NormalCase{normal.at("A").get<std::vector<double>>(),
                             normal.at("b").get<std::vector<double>>(),
                             normal.at("x0").get<std::vector<double>>(),
                             normal.at("suggest_dt").get<double>(),
                             normal.at("separation_time").get<double>()};
  }
  if (frame.contains("body_contact_inputs") && input.normal) {
    const auto& body = frame.at("body_contact_inputs");
    auto depths = body.at("modified_depth").get<std::vector<double>>();
    auto stiffness = body.at("stiffness").get<std::vector<double>>();
    auto damping = body.at("damping").get<std::vector<double>>();
    const auto n = input.problem.contacts;
    if (depths.size() != n || stiffness.size() != n || damping.size() != n || input.friction.size() != n) {
      throw std::runtime_error("Wrong body-contact material shape for " + input.label);
    }
    std::vector<ContactMaterialState> material(n);
    for (std::size_t i = 0; i < n; ++i) {
      material[i] = {depths[i], stiffness[i], damping[i], input.friction[i]};
    }
    input.body = BodyCase{body.at("inverse_inertia").get<std::vector<double>>(),
                          body.at("free_acceleration").get<std::vector<double>>(),
                          std::move(material)};
  }
  return input;
}

auto coldCase(int index) -> Case {
  Case input{};
  input.label = "analytical matrix " + std::to_string(index);
  input.problem.contacts = 1 + index % 4;
  input.problem.step = 0.0005 + 0.00001 * (index % 60);
  const std::size_t n = 3 * input.problem.contacts;
  input.friction.resize(input.problem.contacts, index % 5 == 0 ? 0 : 0.45);
  input.target.resize(input.problem.contacts, -0.2);
  input.inverse_mass.resize(n * n);
  input.velocity.resize(n);
  input.acceleration.resize(n);
  input.force.resize(n);
  for (std::size_t row = 0; row < n; ++row) {
    for (std::size_t col = 0; col < n; ++col) {
      input.inverse_mass[row * n + col] = row == col ? -1.0 - 0.2 * row : -0.1;
    }
    input.velocity[row] = row % 3 == 2 ? -1.1 : 0.1 * (index % 8) - 0.3;
    input.acceleration[row] = row % 3 == 2 ? 3 : -1.4;
  }
  if (index == 0) input.problem.max_iterations = 1;
  input.residual = cptContactForceShiftedSpectralAdmm(
      input.problem.contacts, input.friction, input.inverse_mass, input.velocity,
      input.target, input.acceleration, input.problem.step, input.force,
      input.problem.max_iterations, input.problem.tolerance);
  if (input.residual > input.problem.tolerance) input.status = AdmmStatus::IterationLimit;

  const auto contacts = input.problem.contacts;
  const auto dimension = 2 * contacts;
  NormalCase normal{std::vector<double>(dimension * dimension),
                    std::vector<double>(dimension), std::vector<double>(dimension, 0.001),
                    input.problem.step, -1.0};
  std::copy(input.target.begin(), input.target.end(), normal.initial.begin() + contacts);
  // The forced equilibrium keeps the expected target velocity unchanged while
  // exercising nonsymmetric spectra with mixed real and complex eigenvalues.
  for (std::size_t row = 0; row < dimension; ++row) {
    for (std::size_t col = 0; col < dimension; ++col) {
      const double value = 100.0 * std::sin((row + 1) * (col + 2) * (index + 1.0));
      normal.matrix[row * dimension + col] = value;
      normal.bias[row] -= value * normal.initial[col];
    }
  }
  if (index % 17 == 1) {
    std::fill(normal.matrix.begin(), normal.matrix.end(), 0.0);
    std::fill(normal.bias.begin(), normal.bias.end(), 0.0);
    normal.matrix[contacts] = 1.0;
    normal.initial[0] = -input.target.front() * input.problem.step;
    normal.horizon *= 1.5;
    normal.separation = input.problem.step;
  }
  input.normal = std::move(normal);
  return input;
}

void compare(const std::string& label, const char* component, double actual,
             double expected, double absolute, double relative) {
  if (!std::isfinite(actual) || !std::isfinite(expected) ||
      std::abs(actual - expected) > absolute + relative * std::abs(expected)) {
    std::cerr.precision(17);
    std::cerr << label << ' ' << component << ": GPU " << actual << ", CPU " << expected << '\n';
    throw std::runtime_error("Shifted ADMM numerical mismatch");
  }
}

void checkShapes(const Case& input) {
  const auto count = input.problem.contacts;
  const auto n = 3 * count;
  if (input.friction.size() != count || input.target.size() != count ||
      input.inverse_mass.size() != n * n || input.velocity.size() != n ||
      input.acceleration.size() != n || input.force.size() != n ||
      (input.problem.has_force_guess && input.initial_force.size() != n) ||
      (input.problem.has_dual_guess && input.initial_dual.size() != n) ||
      (!input.dual.empty() && input.dual.size() != n)) {
    throw std::runtime_error("Wrong ADMM shape for " + input.label);
  }
}

void evaluateCpuReference(Case& input, double accepted_step) {
  const auto& normal = *input.normal;
  const auto n = 2 * input.problem.contacts;
  const auto dimension = n + 1;
  std::vector<double> matrix(dimension * dimension, 0.0);
  for (std::size_t row = 0; row < n; ++row) {
    std::copy_n(normal.matrix.data() + row * n, n, matrix.data() + row * dimension);
    matrix[row * dimension + n] = normal.bias[row];
  }
  auto initial = normal.initial;
  initial.push_back(1.0);
  std::vector<double> state(dimension);
  sire::core::screw::matrix_exp_multiply(dimension, matrix.data(), initial.data(),
                                        state.data(), accepted_step);
  std::copy_n(state.data() + input.problem.contacts, input.problem.contacts, input.target.data());
  input.residual = cptContactForceShiftedSpectralAdmmWithWarmStart(
      input.problem.contacts, input.friction, input.inverse_mass, input.velocity,
      input.target, input.acceleration, accepted_step, input.force,
      input.problem.has_force_guess ? &input.initial_force : nullptr,
      input.problem.has_dual_guess ? &input.initial_dual : nullptr,
      &input.dual, input.problem.max_iterations, input.problem.tolerance);
  input.status = input.residual <= input.problem.tolerance
                     ? AdmmStatus::Converged : AdmmStatus::IterationLimit;
}

auto bodyCase(int index) -> Case {
  auto input = coldCase(index);
  const auto n = input.problem.contacts;
  auto& normal = *input.normal;
  normal.horizon = input.problem.step;
  const auto w = 6 * n, d = 3 * n, z = 2 * n;
  BodyCase body{std::vector<double>(w * w, 0.0), std::vector<double>(w, 0.0),
                std::vector<ContactMaterialState>(n)};
  std::fill(normal.matrix.begin(), normal.matrix.end(), 0.0);
  std::fill(normal.bias.begin(), normal.bias.end(), 0.0);
  constexpr double scale = 1e-4;  // matches minimum stiffness 2e8
  for (std::size_t i = 0; i < n; ++i) {
    body.material[i] = {index % 11 == 0 ? 0.0 : 1e-4,
                        2e8 + 1e6 * i, 5e4 - 1e3 * i, input.friction[i]};
    normal.initial[i] = body.material[i].modified_depth / scale;
    normal.initial[n + i] = -input.velocity[3 * i + 2];
    normal.matrix[i * z + n + i] = 1 / scale;
    normal.bias[n + i] = input.acceleration[3 * i + 2];
    for (std::size_t k = 0; k < 3; ++k) {
      body.free_acceleration[6 * i + k + 3] = -input.acceleration[3 * i + k];
    }
  }
  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = 0; j < n; ++j) {
      for (std::size_t k = 0; k < 3; ++k) {
        for (std::size_t l = 0; l < 3; ++l) {
          body.inverse_inertia[(6 * i + k + 3) * w + 6 * j + l + 3] =
              -input.inverse_mass[(3 * i + k) * d + 3 * j + l];
        }
      }
      const double inverse = input.inverse_mass[(3 * i + 2) * d + 3 * j + 2];
      normal.matrix[(n + i) * z + j] = inverse * (body.material[j].stiffness * scale);
      normal.matrix[(n + i) * z + n + j] = inverse * body.material[j].damping;
    }
  }
  normal.separation = findSinglePointContactEndTime(n, normal.horizon, normal.matrix.data(),
                                                    normal.bias.data(), normal.initial.data());
  input.problem.step = normal.separation > 0 ? normal.separation : normal.horizon;
  evaluateCpuReference(input, input.problem.step);
  input.body = std::move(body);
  return input;
}

template <class Launch>
void onContactStream(Launch launch) {
  cudaStream_t stream{};
  checkCuda(cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking));
  const auto enqueue = launch(stream);
  const auto execution = enqueue == cudaSuccess ? cudaStreamSynchronize(stream) : enqueue;
  const auto destroy = cudaStreamDestroy(stream);
  checkCuda(execution);
  checkCuda(destroy);
}

void runBodyStep(const std::vector<Case>& cases, std::size_t capacity,
                 AdmmInputs admm, ShiftedContactOutputs outputs,
                 ShiftedContactScratch scratch, const DeviceBuffer<double>& normal_matrix,
                 const DeviceBuffer<double>& normal_bias, const DeviceBuffer<double>& normal_initial) {
  const auto batch = cases.size();
  const auto w = 6 * capacity, d = 3 * capacity, z = 2 * capacity;
  const double pad = std::numeric_limits<double>::quiet_NaN();
  std::vector<double> inverse_inertia(batch * w * w, pad), free_acceleration(batch * w, pad);
  std::vector<ContactMaterialState> material(batch * capacity, {pad, pad, pad, pad});
  for (std::size_t env = 0; env < batch; ++env) {
    const auto& body = *cases[env].body;
    const auto counts = cases[env].problem.contacts;
    const auto active = 6 * counts;
    if (body.material.size() != counts || body.free_acceleration.size() != active ||
        body.inverse_inertia.size() != active * active) {
      throw std::runtime_error("Wrong body inertia shape for " + cases[env].label);
    }
    std::copy(body.material.begin(), body.material.end(), material.begin() + env * capacity);
    std::copy(body.free_acceleration.begin(), body.free_acceleration.end(), free_acceleration.begin() + env * w);
    for (std::size_t row = 0; row < active; ++row) {
      std::copy_n(body.inverse_inertia.data() + row * active, active,
                  inverse_inertia.data() + (env * w + row) * w);
    }
  }

  DeviceBuffer<ContactMaterialState> gpu_material(material.size());
  DeviceBuffer<double> gpu_inertia(inverse_inertia.size()), gpu_acceleration(free_acceleration.size());
  DeviceBuffer<AdmmProblem> assembled_problems(batch);
  DeviceBuffer<double> friction(batch * capacity), relative_inertia(batch * d * d), relative_acceleration(batch * d);
  gpu_material.upload(material);
  gpu_inertia.upload(inverse_inertia);
  gpu_acceleration.upload(free_acceleration);
  onContactStream([&](cudaStream_t stream) {
    return launchBodyContactStep(batch, capacity,
        {admm.problems, gpu_material.data(), gpu_inertia.data(), gpu_acceleration.data(),
         admm.velocity, admm.force_guess, admm.dual_guess}, outputs,
        {assembled_problems.data(), friction.data(), relative_inertia.data(), relative_acceleration.data(),
         normal_matrix.data(), normal_bias.data(), normal_initial.data()}, scratch, stream);
  });

  const auto friction_values = friction.download();
  const auto inertia_values = relative_inertia.download();
  const auto acceleration_values = relative_acceleration.download();
  const auto a_values = normal_matrix.download();
  const auto b_values = normal_bias.download();
  const auto initial_values = normal_initial.download();
  for (std::size_t env = 0; env < batch; ++env) {
    const auto& input = cases[env];
    if (input.status == AdmmStatus::InvalidInput) continue;
    const auto n = input.problem.contacts;
    const auto& normal = *input.normal;
    for (std::size_t i = 0; i < n; ++i) {
      compare(input.label, "assembled friction", friction_values[env * capacity + i], input.friction[i], 0, 0);
    }
    for (std::size_t i = 0; i < 3 * n; ++i) {
      compare(input.label, "assembled acceleration", acceleration_values[env * d + i], input.acceleration[i], 1e-12, 1e-13);
      for (std::size_t j = 0; j < 3 * n; ++j) {
        compare(input.label, "assembled inverse inertia", inertia_values[(env * d + i) * d + j],
                input.inverse_mass[i * 3 * n + j], 1e-12, 1e-13);
      }
    }
    for (std::size_t i = 0; i < 2 * n; ++i) {
      compare(input.label, "assembled normal bias", b_values[env * z + i], normal.bias[i], 1e-12, 1e-13);
      compare(input.label, "assembled initial depth/rate", initial_values[env * z + i], normal.initial[i], 1e-12, 1e-13);
      for (std::size_t j = 0; j < 2 * n; ++j) {
        compare(input.label, "assembled normal coefficient", a_values[(env * z + i) * z + j],
                normal.matrix[i * 2 * n + j], 1e-9, 1e-12);
      }
    }
  }
}

void runNormalStep(std::vector<Case>& cases, std::size_t capacity,
                   AdmmInputs admm, AdmmOutputs outputs,
                   void* admm_workspace, std::size_t admm_size, StepInput step) {
  std::vector<AdmmProblem> problems;
  const auto stride = 2 * capacity;
  const auto values = cases.size() * stride;
  const double pad = std::numeric_limits<double>::quiet_NaN();
  std::vector<double> matrix(values * stride, pad), bias(values, pad), initial(values, pad);
  for (std::size_t i = 0; i < cases.size(); ++i) {
    const auto& input = cases[i];
    const auto& normal = *input.normal;
    auto problem = input.problem;
    problem.step = normal.horizon;
    problems.push_back(problem);
    const auto n = 2 * problem.contacts;
    const auto offset = i * stride;
    if (normal.matrix.size() != n * n || normal.bias.size() != n || normal.initial.size() != n) {
      throw std::runtime_error("Wrong normal contact shape for " + input.label);
    }
    std::copy(normal.bias.begin(), normal.bias.end(), bias.begin() + offset);
    std::copy(normal.initial.begin(), normal.initial.end(), initial.begin() + offset);
    for (std::size_t row = 0; row < n; ++row) {
      std::copy_n(normal.matrix.data() + row * n, n, matrix.data() + (offset + row) * stride);
    }
  }

  DeviceBuffer<AdmmProblem> gpu_problems(cases.size()), accepted(cases.size());
  DeviceBuffer<NormalContactResult> gpu_normal(cases.size());
  DeviceBuffer<double> gpu_matrix(matrix.size()), gpu_bias(values), gpu_initial(values);
  DeviceBuffer<double> targets(cases.size() * capacity);
  const auto normal_size = separationWorkspaceSize(cases.size(), capacity);
  DeviceBuffer<double> workspace(normal_size);
  gpu_problems.upload(problems);
  targets.upload(std::vector<double>(cases.size() * capacity, pad));
  admm.problems = gpu_problems.data();
  admm.normal_target = nullptr;
  const ShiftedContactOutputs contact_outputs{gpu_normal.data(), targets.data(), outputs};
  const ShiftedContactScratch scratch{accepted.data(), workspace.data(), normal_size,
                                       admm_workspace, admm_size};
  if (step == StepInput::Body) {
    runBodyStep(cases, capacity, admm, contact_outputs, scratch, gpu_matrix, gpu_bias, gpu_initial);
  } else {
    gpu_matrix.upload(matrix);
    gpu_bias.upload(bias);
    gpu_initial.upload(initial);
    onContactStream([&](cudaStream_t stream) {
      return launchShiftedContactStep(cases.size(), capacity,
          {admm, gpu_matrix.data(), gpu_bias.data(), gpu_initial.data()}, contact_outputs, scratch, stream);
    });
  }
  const auto computed_targets = targets.download();
  const auto results = gpu_normal.download();
  for (std::size_t i = 0; i < cases.size(); ++i) {
    auto& input = cases[i];
    const auto& normal = *input.normal;
    const auto& result = results[i];
    if (input.status == AdmmStatus::InvalidInput) {
      if (result.status != TrajectoryStatus::InvalidInput || result.accepted_step != 0.0) {
        throw std::runtime_error("Invalid normal step accepted for " + input.label);
      }
      continue;
    }
    if (result.status != TrajectoryStatus::Success) {
      throw std::runtime_error("Normal step failed for " + input.label);
    }
    if (normal.separation == -1.0 && result.separation_time != -1.0) {
      throw std::runtime_error("Unexpected separation for " + input.label);
    }
    const auto spectrum = input.problem.contacts == 0
                              ? sire::physics::contact::detail::ContactTimeSpectrum{0, 0}
                              : contactTimeSpectrum(2 * input.problem.contacts, normal.matrix.data());
    compare(input.label, "spectral rate", result.spectral_rate, spectrum.rate, 1e-8, 1e-8);
    compare(input.label, "spectral frequency", result.spectral_frequency, spectrum.frequency, 1e-8, 1e-8);
    const auto tolerance = contactTimeGrid(normal.horizon, spectrum).tolerance;
    if (normal.separation != -1.0) {
      compare(input.label, "separation", result.separation_time, normal.separation, tolerance, 0);
    }
    compare(input.label, "accepted timestep", result.accepted_step, input.problem.step, tolerance, 0);
    // Both searches return the separating endpoint. Check its time first,
    // then compare forces with the CPU evaluated at that same timestep.
    if (result.accepted_step != input.problem.step && input.problem.contacts != 0) {
      evaluateCpuReference(input, result.accepted_step);
    }
    for (std::size_t j = 0; j < input.problem.contacts; ++j) {
      compare(input.label, "target velocity", computed_targets[i * capacity + j], input.target[j], 1e-8, 1e-7);
    }
    for (std::size_t j = input.problem.contacts; j < capacity; ++j) {
      if (!std::isnan(computed_targets[i * capacity + j])) {
        throw std::runtime_error("Normal target overwritten past active contacts");
      }
    }
  }
}

void runBatch(std::vector<Case>& cases, StepInput step) {
  std::size_t capacity = 1;
  for (const auto& input : cases) capacity = std::max(capacity, input.problem.contacts);
  ++capacity;
  const auto stride = 3 * capacity;
  const auto values = cases.size() * stride;
  const double pad = std::numeric_limits<double>::quiet_NaN();
  std::vector<AdmmProblem> problems;
  std::vector<double> friction(cases.size() * capacity, pad), target(friction);
  std::vector<double> inverse_mass(values * stride, pad), velocity(values, pad);
  std::vector<double> acceleration(velocity), initial_force(velocity), initial_dual(velocity);
  for (std::size_t i = 0; i < cases.size(); ++i) {
    const auto& input = cases[i];
    checkShapes(input);
    problems.push_back(input.problem);
    const auto contact_offset = i * capacity;
    const auto offset = i * stride;
    const auto n = 3 * input.problem.contacts;
    std::copy(input.friction.begin(), input.friction.end(), friction.begin() + contact_offset);
    std::copy(input.target.begin(), input.target.end(), target.begin() + contact_offset);
    std::copy(input.velocity.begin(), input.velocity.end(), velocity.begin() + offset);
    std::copy(input.acceleration.begin(), input.acceleration.end(), acceleration.begin() + offset);
    std::copy(input.initial_force.begin(), input.initial_force.end(), initial_force.begin() + offset);
    std::copy(input.initial_dual.begin(), input.initial_dual.end(), initial_dual.begin() + offset);
    for (std::size_t row = 0; row < n; ++row) {
      std::copy_n(input.inverse_mass.data() + row * n, n,
                  inverse_mass.data() + (offset + row) * stride);
    }
  }

  DeviceBuffer<AdmmProblem> gpu_problems(problems.size());
  DeviceBuffer<AdmmResult> gpu_results(problems.size());
  DeviceBuffer<double> gpu_friction(friction.size()), gpu_target(target.size());
  DeviceBuffer<double> gpu_inverse_mass(inverse_mass.size()), gpu_velocity(values);
  DeviceBuffer<double> gpu_acceleration(values), gpu_force_guess(values), gpu_dual_guess(values);
  DeviceBuffer<double> gpu_force(values), gpu_dual(values);
  const auto workspace_size = spectralAdmmWorkspaceSize(cases.size(), capacity);
  DeviceBuffer<unsigned char> workspace(workspace_size);
  gpu_problems.upload(problems);
  gpu_friction.upload(friction);
  gpu_inverse_mass.upload(inverse_mass);
  gpu_velocity.upload(velocity);
  gpu_target.upload(target);
  gpu_acceleration.upload(acceleration);
  gpu_force_guess.upload(initial_force);
  gpu_dual_guess.upload(initial_dual);
  constexpr double untouched = -184.25;
  const std::vector<double> sentinel(values, untouched);
  gpu_force.upload(sentinel);
  gpu_dual.upload(sentinel);
  const AdmmInputs inputs{gpu_problems.data(), gpu_friction.data(), gpu_inverse_mass.data(),
                          gpu_velocity.data(), gpu_target.data(), gpu_acceleration.data(),
                          gpu_force_guess.data(), gpu_dual_guess.data()};
  const AdmmOutputs outputs{gpu_force.data(), gpu_dual.data(), gpu_results.data()};
  if (launchShiftedSpectralAdmm(cases.size(), capacity, inputs, outputs, workspace.data(),
                                workspace_size - 1, nullptr) != cudaErrorInvalidValue) {
    throw std::runtime_error("Short ADMM workspace was accepted");
  }
  if (step != StepInput::Admm) {
    runNormalStep(cases, capacity, inputs, outputs, workspace.data(), workspace_size, step);
  }
  else {
    checkCuda(launchShiftedSpectralAdmm(cases.size(), capacity, inputs, outputs,
                                       workspace.data(), workspace_size, nullptr));
    checkCuda(cudaDeviceSynchronize());
  }
  const auto force = gpu_force.download();
  const auto dual = gpu_dual.download();
  const auto results = gpu_results.download();
  for (std::size_t i = 0; i < cases.size(); ++i) {
    const auto& input = cases[i];
    const auto& result = results[i];
    if (result.status != input.status) throw std::runtime_error("Wrong ADMM status for " + input.label);
    const auto n = result.status == AdmmStatus::InvalidInput ? 0 : 3 * input.problem.contacts;
    if (n != 0) {
      const double residual = std::max({result.primal_residual, result.dual_residual,
                                       result.complementarity_residual});
      compare(input.label, "residual", residual, input.residual, 1e-9, 0.05);
    }
    for (std::size_t j = 0; j < n; ++j) {
      compare(input.label, "force", force[i * stride + j], input.force[j], 1e-6, 2e-8);
      if (!input.dual.empty()) {
        compare(input.label, "dual", dual[i * stride + j], input.dual[j], 1e-8, 1e-7);
      }
    }
    for (std::size_t j = n; j < stride; ++j) {
      if (force[i * stride + j] != untouched || dual[i * stride + j] != untouched) {
        throw std::runtime_error("PAD or invalid outputs modified for " + input.label);
      }
    }
  }
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
    checkCuda(launchShiftedSpectralAdmm(0, 0, {}, {}, nullptr, 0, nullptr));
    if (spectralAdmmWorkspaceSize(std::numeric_limits<std::size_t>::max(), 1) != 0 ||
        spectralAdmmWorkspaceSize(1, std::numeric_limits<std::size_t>::max()) != 0) {
      throw std::runtime_error("ADMM workspace overflow was accepted");
    }
    std::vector<Case> cases;
    for (int i = 0; i < 1024; ++i) cases.push_back(coldCase(i));
    Case no_contacts{};
    no_contacts.label = "no contacts";
    no_contacts.normal = NormalCase{{}, {}, {}, no_contacts.problem.step, -1.0};
    cases.push_back(no_contacts);
    auto invalid = coldCase(3);
    invalid.label = "invalid timestep";
    invalid.problem.step = 0;
    invalid.normal->horizon = 0;
    invalid.status = AdmmStatus::InvalidInput;
    cases.push_back(invalid);
    for (int i = 1; i < argc; ++i) cases.push_back(readFrame(argv[i]));
    runBatch(cases, StepInput::Admm);
    std::cout << "Validated " << cases.size() << " CUDA shifted ADMM cases\n";
    cases.erase(std::remove_if(cases.begin(), cases.end(), [](const auto& input) { return !input.normal; }),
                cases.end());
    auto normal_cases = cases;
    runBatch(normal_cases, StepInput::Normal);
    std::cout << "Validated " << cases.size() << " CUDA normal-to-ADMM contact steps\n";
    std::vector<Case> body_cases;
    for (int i = 0; i < 1024; ++i) body_cases.push_back(bodyCase(i));
    no_contacts.body = BodyCase{};
    body_cases.push_back(no_contacts);
    auto bad_material = bodyCase(1);
    bad_material.label = "invalid material stiffness";
    bad_material.body->material.front().stiffness = 0.0;
    bad_material.status = AdmmStatus::InvalidInput;
    body_cases.push_back(std::move(bad_material));
    for (auto& input : cases) {
      if (input.body) body_cases.push_back(std::move(input));
    }
    runBatch(body_cases, StepInput::Body);
    std::cout << "Validated " << body_cases.size() << " CUDA body-contact assembly and solve cases\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
