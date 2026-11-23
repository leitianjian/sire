#include "contact_trajectory.hpp"

#include "contact_time_spectrum.cuh"
#include "shifted_contact_step.hpp"
#include "uniform_normal_modes.cuh"

#include <limits>

#include "core/matrix_exp_pade.hpp"
#include "physics/contact/contact_assembly_math.hpp"
#include "physics/contact/contact_time_search.hpp"

namespace sire::physics::contact::cuda {
namespace {

using sire::core::screw::detail::PadeBuffers;
using sire::core::screw::detail::PadeStatus;
using sire::core::screw::detail::finite;

constexpr std::size_t trajectory_matrices = 6;
constexpr std::size_t separation_matrices = 7;
constexpr unsigned threads_per_block = 128;
// Each environment solves independently. One warp per block distributes a
// typical RL batch across more SMs.
constexpr unsigned normal_threads_per_block = 32;

template <std::size_t FixedDimension>
struct Multiply {
  __host__ __device__ auto operator()(std::size_t dimension, const double* lhs,
                                     const double* rhs, double* output) const
      -> void {
    const auto n = FixedDimension == 0 ? dimension : FixedDimension;
    for (std::size_t row = 0; row < n; ++row) {
      for (std::size_t col = 0; col < n; ++col) {
        double value = 0.0;
        for (std::size_t k = 0; k < n; ++k) {
          value += lhs[row * n + k] * rhs[k * n + col];
        }
        output[row * n + col] = value;
      }
    }
  }
};

__device__ auto writeAffine(std::size_t n, std::size_t matrix_stride,
                            const double* matrix, const double* bias,
                            const double* initial,
                            double* affine) -> bool {
  const auto augmented = n + 1;
  for (std::size_t row = 0; row < n; ++row) {
    if (!finite(initial[row]) || !finite(bias[row])) return false;
    for (std::size_t col = 0; col < n; ++col) {
      const double value = matrix[row * matrix_stride + col];
      if (!finite(value)) return false;
      affine[row * augmented + col] = value;
    }
    affine[row * augmented + n] = bias[row];
  }
  for (std::size_t col = 0; col < augmented; ++col) {
    affine[n * augmented + col] = 0.0;
  }
  return true;
}

template <std::size_t FixedAugmented = 0>
__device__ __noinline__ auto exponential(std::size_t dimension, double* affine, double time)
    -> const double* {
  const auto augmented = FixedAugmented == 0 ? dimension : FixedAugmented;
  const auto area = augmented * augmented;
  PadeBuffers buffers{affine + area, affine + 2 * area, affine + 3 * area,
                      affine + 4 * area, affine + 5 * area};
  int squarings = 0;
  if (preparePade(augmented, affine, time, buffers, squarings, Multiply<FixedAugmented>{}) !=
          PadeStatus::Success ||
      solvePade(augmented, buffers) != PadeStatus::Success ||
      squarePade(augmented, buffers, squarings, Multiply<FixedAugmented>{}) != PadeStatus::Success) {
    return nullptr;
  }
  return buffers.numerator;
}

// Apply exp(t*[A b; 0 0]) to [x0; 1], with an implicit last component.
__device__ auto propagate(std::size_t n, const double* transition,
                           const double* initial, double* result) -> bool {
  if (transition == nullptr) return false;
  for (std::size_t row = 0; row < n; ++row) {
    const auto* coefficients = transition + row * (n + 1);
    double value = 0.0;
    for (std::size_t col = 0; col < n; ++col) {
      value += coefficients[col] * initial[col];
    }
    value += coefficients[n];
    if (!finite(value)) return false;
    result[row] = value;
  }
  return true;
}

__device__ auto evaluateTrajectory(std::size_t n, const double* matrix,
                                    const double* bias, const double* initial,
                                    double time, double* result,
                                    double* workspace) -> TrajectoryStatus {
  if (!finite(time) || time < 0.0 || !writeAffine(n, n, matrix, bias, initial, workspace)) {
    return TrajectoryStatus::InvalidInput;
  }
  const auto augmented = n + 1;
  const auto* transition = exponential(augmented, workspace, time);
  // Use the scaled matrix buffer until every component is finite.
  auto* state = workspace + augmented * augmented;
  if (!propagate(n, transition, initial, state)) return TrajectoryStatus::NumericalFailure;
  for (std::size_t row = 0; row < n; ++row) result[row] = state[row];
  return TrajectoryStatus::Success;
}

__device__ auto minDepth(std::size_t count, const double* state) -> double {
  double depth = state[0];
  for (std::size_t i = 1; i < count; ++i) depth = ::fmin(depth, state[i]);
  return depth;
}

__device__ auto validSearch(SeparationSearch search) -> bool {
  return finite(search.horizon) && search.horizon > 0.0 &&
         finite(search.spectral_rate) && search.spectral_rate >= 0.0 &&
         finite(search.spectral_frequency) && search.spectral_frequency >= 0.0;
}

template <class Flow>
__device__ auto scanSeparation(std::size_t n, const double* initial,
                               SeparationSearch search, double& result,
                               double* scan_state, double* trial_state, Flow& flow)
    -> TrajectoryStatus {
  for (std::size_t row = 0; row < n; ++row) scan_state[row] = initial[row];

  const auto grid = detail::contactTimeGrid(
      search.horizon, {search.spectral_rate, search.spectral_frequency});
  double step = grid.first_step;
  double lower = 0;
  while (lower < search.horizon) {
    const double upper = ::fmin(search.horizon, lower + step);
    if (upper <= lower) return TrajectoryStatus::NumericalFailure;
    const bool fixed_interval = step == grid.max_step && lower + step <= search.horizon;
    if (!flow.advance(fixed_interval ? grid.max_step : upper - lower,
                      scan_state, trial_state, fixed_interval)) {
      return TrajectoryStatus::NumericalFailure;
    }

    // Confirm a negative depth directly from x(0), as in the CPU scan.
    double depth = flow.depth(trial_state);
    if (!finite(depth)) return TrajectoryStatus::NumericalFailure;
    if (depth < 0.0) {
      if (!flow.advance(upper, initial, trial_state, false)) {
        return TrajectoryStatus::NumericalFailure;
      }
      depth = flow.depth(trial_state);
      if (!finite(depth)) return TrajectoryStatus::NumericalFailure;
    }
    if (depth < 0.0) {
      const double left_depth = flow.depth(scan_state);
      if (!finite(left_depth)) return TrajectoryStatus::NumericalFailure;
      detail::ContactTimeBracket bracket(lower, upper, left_depth, depth, grid.tolerance);
      for (int iteration = 0; bracket.needsRefinement(iteration); ++iteration) {
        const auto trial = bracket.nextTrial(iteration);
        if (!flow.advance(trial.time - bracket.left(), scan_state, trial_state, false)) {
          return TrajectoryStatus::NumericalFailure;
        }
        depth = flow.depth(trial_state);
        if (!finite(depth)) return TrajectoryStatus::NumericalFailure;
        bracket.update(trial, depth);
        if (depth >= 0.0) sire::core::screw::detail::swap(scan_state, trial_state);
      }
      result = bracket.right();
      return TrajectoryStatus::Success;
    }
    sire::core::screw::detail::swap(scan_state, trial_state);
    lower = upper;
    step = ::fmin(grid.max_step, step * 1.5);
  }
  result = -1.0;
  return TrajectoryStatus::Success;
}

template <std::size_t FixedDimension>
struct PadeFlow {
  std::size_t n, area;
  double* workspace;
  double* fixed_transition;
  bool fixed_ready{false};
  static constexpr auto augmented = FixedDimension == 0 ? 0 : FixedDimension + 1;

  __device__ auto advance(double time, const double* from, double* to, bool fixed) -> bool {
    if (!fixed) return propagate(n, exponential<augmented>(n + 1, workspace, time), from, to);
    if (!fixed_ready) {
      const auto* transition = exponential<augmented>(n + 1, workspace, time);
      if (!transition) return false;
      for (std::size_t i = 0; i < area; ++i) fixed_transition[i] = transition[i];
      fixed_ready = true;
    }
    return propagate(n, fixed_transition, from, to);
  }

  __device__ auto depth(const double* state) const -> double {
    return minDepth(n / 2, state);
  }
};

template <std::size_t FixedDimension = 0>
__device__ auto findSeparation(std::size_t dimension, const double* initial,
                              SeparationSearch search, double& result,
                              double* workspace) -> TrajectoryStatus {
  const auto n = FixedDimension == 0 ? dimension : FixedDimension;
  const auto area = (n + 1) * (n + 1);
  auto* fixed_transition = workspace + trajectory_matrices * area;
  auto* scan_state = fixed_transition + area;
  PadeFlow<FixedDimension> flow{n, area, workspace, fixed_transition};
  return scanSeparation(n, initial, search, result, scan_state, scan_state + n, flow);
}

__global__ void evaluateBatch(std::size_t batch_size, std::size_t dimension,
                              std::size_t workspace_stride, const double* matrix,
                              const double* bias, const double* initial_state,
                              const double* time, double* state,
                              TrajectoryStatus* status, double* workspace) {
  const auto index = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index >= batch_size) return;
  const auto offset = index * dimension;
  status[index] = evaluateTrajectory(
      dimension, matrix + offset * dimension, bias + offset,
      initial_state + offset, time[index], state + offset,
      workspace + index * workspace_stride);
}

__global__ void searchBatch(std::size_t batch_size, std::size_t dimension,
                            std::size_t workspace_stride, const double* matrix,
                            const double* bias, const double* initial_state,
                            const SeparationSearch* search, double* separation_time,
                            TrajectoryStatus* status, double* workspace) {
  const auto index = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index >= batch_size) return;
  const auto offset = index * dimension;
  auto* work = workspace + index * workspace_stride;
  if (!validSearch(search[index]) ||
      !writeAffine(dimension, dimension, matrix + offset * dimension, bias + offset,
                   initial_state + offset, work)) {
    status[index] = TrajectoryStatus::InvalidInput;
    return;
  }
  status[index] = findSeparation(dimension, initial_state + offset, search[index],
                                 separation_time[index], work);
}

template <std::size_t FixedDimension = 0>
__device__ auto evaluateNormalStep(std::size_t dimension, std::size_t matrix_stride,
                                   const double* matrix, const double* bias,
                                   const double* initial, double horizon,
                                   double* target, double* workspace) -> NormalContactResult {
  NormalContactResult result{TrajectoryStatus::InvalidInput, -1, 0, 0, 0};
  const auto n = FixedDimension == 0 ? dimension : FixedDimension;
  constexpr auto fixed_augmented = FixedDimension == 0 ? 0 : FixedDimension + 1;
  if (!finite(horizon) || horizon <= 0.0 ||
      !writeAffine(n, matrix_stride, matrix, bias, initial, workspace)) return result;
  if (n == 0) return {TrajectoryStatus::Success, -1, horizon, 0, 0};

  const auto augmented = n + 1;
  const auto area = augmented * augmented;
  // The Hessenberg matrix and reflector scratch are released before Padé.
  detail::ContactTimeSpectrum spectrum{};
  if (!time_spectrum::normalSpectrum(workspace, static_cast<int>(augmented),
                                     static_cast<int>(n), workspace + area, spectrum)) {
    result.status = TrajectoryStatus::NumericalFailure;
    return result;
  }
  result.spectral_rate = spectrum.rate;
  result.spectral_frequency = spectrum.frequency;
  result.status = findSeparation<FixedDimension>(
      n, initial, {horizon, spectrum.rate, spectrum.frequency}, result.separation_time, workspace);
  if (result.status != TrajectoryStatus::Success) return result;
  const double accepted = result.separation_time > 0.0
                              ? ::fmin(result.separation_time, horizon)
                              : horizon;
  const auto* transition = exponential<fixed_augmented>(augmented, workspace, accepted);
  auto* state = workspace + area;
  if (!propagate(n, transition, initial, state)) {
    result.status = TrajectoryStatus::NumericalFailure;
    return result;
  }
  for (std::size_t i = 0; i < n / 2; ++i) target[i] = state[n / 2 + i];
  result.accepted_step = accepted;
  return result;
}

template <std::size_t Contacts>
__device__ __noinline__ auto evaluateSmallNormalStep(
    std::size_t stride, const double* matrix, const double* bias, const double* initial,
    double horizon, double* target) -> NormalContactResult {
  constexpr auto n = 2 * Contacts;
  constexpr auto augmented = n + 1;
  // CUDA lays out the same thread-local offset contiguously across a warp.
  // Small solves also touch less scratch than the padded global path.
  double workspace[separation_matrices * augmented * augmented + 2 * n];
  return evaluateNormalStep<n>(n, stride, matrix, bias, initial, horizon, target, workspace);
}

__device__ __noinline__ auto evaluateUniformNormalStep(
    std::size_t contacts, std::size_t stride, const double* matrix, const double* bias,
    const double* initial, const ContactMaterialState* material,
    double horizon, double* target, NormalContactResult& result) -> bool {
  if (!material || contacts > uniform_normal::capacity || !finite(horizon) || horizon <= 0.0) return false;
  for (std::size_t i = 1; i < contacts; ++i) {
    if (material[i].stiffness != material[0].stiffness ||
        material[i].damping != material[0].damping) return false;
  }
  const double velocity_scale = matrix[contacts];
  const double damping_ratio = material[0].damping / material[0].stiffness;
  if (!finite(velocity_scale) || velocity_scale <= 0.0 ||
      !finite(damping_ratio) || damping_ratio < 0.0) return false;
  uniform_normal::Flow flow{};
  flow.n = contacts;
  double transformed[2 * uniform_normal::capacity];
  double scan[2 * uniform_normal::capacity], trial[2 * uniform_normal::capacity];
  detail::ContactTimeSpectrum spectrum{};
  if (!uniform_normal::initialize(flow, stride, matrix, bias, initial, velocity_scale,
                                  damping_ratio, transformed, spectrum)) return false;

  double separation = -1.0;
  if (scanSeparation(2 * contacts, transformed, {horizon, spectrum.rate, spectrum.frequency},
                     separation, scan, trial, flow) != TrajectoryStatus::Success) return false;
  const double accepted = separation > 0.0 ? ::fmin(separation, horizon) : horizon;
  if (!flow.advance(accepted, transformed, trial, false)) return false;
  for (std::size_t i = 0; i < contacts; ++i) {
    double derivative = 0.0;
    for (std::size_t j = 0; j < contacts; ++j) {
      derivative += flow.eigenvectors[i * contacts + j] * trial[contacts + j];
    }
    target[i] = derivative / velocity_scale;
    if (!finite(target[i])) return false;
  }
  result = {TrajectoryStatus::Success, separation, accepted, spectrum.rate, spectrum.frequency};
  return true;
}

__global__ void normalStepBatch(std::size_t batch_size, std::size_t capacity,
                                ShiftedContactInputs inputs,
                                NormalContactResult* results, double* targets,
                                AdmmProblem* accepted_problems, double* workspace,
                                std::size_t workspace_stride) {
  const auto index = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index >= batch_size) return;
  const auto problem = inputs.admm.problems[index];
  NormalContactResult result{TrajectoryStatus::InvalidInput, -1, 0, 0, 0};
  if (problem.contacts <= capacity) {
    const auto stride = 2 * capacity;
    const auto offset = index * stride;
    const auto* matrix = inputs.normal_matrix + offset * stride;
    const auto* bias = inputs.normal_bias + offset;
    const auto* initial = inputs.initial_normal_state + offset;
    auto* target = targets + index * capacity;
    const auto* material = inputs.material ? inputs.material + index * capacity : nullptr;
    const bool modal = problem.contacts != 0 &&
        evaluateUniformNormalStep(problem.contacts, stride, matrix, bias, initial, material,
                                  problem.step, target, result);
    if (!modal) {
      switch (problem.contacts) {
      case 0:
        if (finite(problem.step) && problem.step > 0.0) {
          result = {TrajectoryStatus::Success, -1, problem.step, 0, 0};
        }
        break;
      case 1:
        result = evaluateSmallNormalStep<1>(stride, matrix, bias, initial, problem.step, target);
        break;
      case 2:
        result = evaluateSmallNormalStep<2>(stride, matrix, bias, initial, problem.step, target);
        break;
      case 3:
        result = evaluateSmallNormalStep<3>(stride, matrix, bias, initial, problem.step, target);
        break;
      case 4:
        result = evaluateSmallNormalStep<4>(stride, matrix, bias, initial, problem.step, target);
        break;
      default:
        result = evaluateNormalStep(2 * problem.contacts, stride, matrix, bias, initial,
                                    problem.step, target, workspace + index * workspace_stride);
        break;
      }
    }
  }
  results[index] = result;
  // A zero accepted step prevents the ADMM kernel from reading unavailable
  // targets or publishing an iterate for a failed normal solve.
  accepted_problems[index] = problem;
  accepted_problems[index].step = result.accepted_step;
}

__device__ auto inertiaBlock(const double* matrix, std::size_t stride,
                            std::size_t row_a, std::size_t col_a)
    -> detail::ContactInverseInertiaBlock {
  const auto row_b = row_a + 3;
  return {matrix[row_a * stride + col_a], matrix[row_a * stride + col_a + 3],
          matrix[row_b * stride + col_a], matrix[row_b * stride + col_a + 3]};
}

__device__ auto assembleBodyContact(
    std::size_t index, std::size_t capacity, std::size_t count,
    BodyContactInputs inputs, BodyContactAssembly output) -> bool {
  const auto contact_offset = index * capacity;
  const auto normal_stride = 2 * capacity;
  const auto relative_stride = 3 * capacity;
  const auto body_stride = 6 * capacity;
  const auto* material = inputs.material + contact_offset;
  const auto* raw_matrix = inputs.inverse_inertia + index * body_stride * body_stride;
  const auto* raw_acceleration = inputs.free_acceleration + index * body_stride;
  const auto* velocity = inputs.relative_velocity + index * relative_stride;
  const auto* body_indices = inputs.body_indices ? inputs.body_indices + contact_offset : nullptr;
  auto* friction = output.friction + contact_offset;
  auto* inverse = output.relative_inverse_inertia + index * relative_stride * relative_stride;
  auto* acceleration = output.relative_acceleration + index * relative_stride;
  auto* normal_matrix = output.normal_matrix + index * normal_stride * normal_stride;
  auto* normal_bias = output.normal_bias + index * normal_stride;
  auto* initial = output.initial_normal_state + index * normal_stride;

  double minimum_stiffness = 1e20;  // Matches the CPU SOR and ADMM solvers.
  for (std::size_t i = 0; i < count; ++i) {
    const auto& properties = material[i];
    if ((body_indices && body_indices[i] >= capacity) ||
        !finite(properties.stiffness) || properties.stiffness <= 0.0 ||
        !finite(properties.damping) || properties.damping < 0.0 ||
        !finite(properties.friction) || properties.friction < 0.0 ||
        !finite(properties.modified_depth) || properties.modified_depth < 0.0) {
      return false;
    }
    minimum_stiffness = ::fmin(minimum_stiffness, properties.stiffness);
  }
  const double scale = detail::penetrationScale(minimum_stiffness);
  if (!finite(scale) || scale <= 0.0) return false;

  for (std::size_t i = 0; i < count; ++i) {
    const auto body_i = body_indices ? body_indices[i] : i;
    friction[i] = material[i].friction;
    initial[i] = material[i].modified_depth / scale;
    initial[count + i] = -velocity[3 * i + 2];
    normal_bias[i] = 0.0;
    for (std::size_t j = 0; j < 2 * count; ++j) {
      normal_matrix[i * normal_stride + j] = j == count + i ? 1.0 / scale : 0.0;
    }

    for (std::size_t k = 0; k < 3; ++k) {
      const double relative = raw_acceleration[6 * body_i + k] - raw_acceleration[6 * body_i + k + 3];
      if (!finite(relative) || !finite(velocity[3 * i + k])) return false;
      acceleration[3 * i + k] = relative;
    }
    normal_bias[count + i] = acceleration[3 * i + 2];

    for (std::size_t j = 0; j < count; ++j) {
      const auto body_j = body_indices ? body_indices[j] : j;
      for (std::size_t k = 0; k < 3; ++k) {
        for (std::size_t l = 0; l < 3; ++l) {
          const auto block = inertiaBlock(raw_matrix, body_stride, 6 * body_i + k, 6 * body_j + l);
          const double value = detail::signedRelativeInertia(block);
          if (!finite(value)) return false;
          inverse[(3 * i + k) * relative_stride + 3 * j + l] = value;
        }
      }
      const auto normal = inertiaBlock(raw_matrix, body_stride, 6 * body_i + 2, 6 * body_j + 2);
      auto* row = normal_matrix + (count + i) * normal_stride;
      row[j] = detail::normalContactCoefficient(normal, material[j].stiffness * scale);
      row[count + j] = detail::normalContactCoefficient(normal, material[j].damping);
    }
  }
  return true;
}

__global__ void assembleBodyBatch(std::size_t batch_size, std::size_t capacity,
                                 BodyContactInputs inputs, BodyContactAssembly output) {
  const auto index = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index >= batch_size) return;
  auto problem = inputs.problems[index];
  if (!finite(problem.step) || problem.step <= 0.0 || problem.contacts > capacity ||
      (problem.contacts != 0 && !assembleBodyContact(index, capacity, problem.contacts, inputs, output))) {
    problem.step = 0.0;
  }
  output.problems[index] = problem;
}

auto workspaceSize(std::size_t batch_size, std::size_t contact_count,
                   std::size_t matrices, std::size_t vectors) -> std::size_t {
  constexpr auto max_elements = std::numeric_limits<std::size_t>::max() / sizeof(double);
  if (batch_size == 0 || contact_count == 0 || contact_count > (max_elements - 1) / 2) {
    return 0;
  }
  const auto dimension = 2 * contact_count + 1;
  const auto max_stride = max_elements / batch_size;
  if (dimension > max_stride / dimension / matrices) return 0;
  const auto matrix_count = matrices * dimension * dimension;
  if (vectors && dimension - 1 > (max_stride - matrix_count) / vectors) return 0;
  return batch_size * (matrix_count + vectors * (dimension - 1));
}

}  // namespace

auto trajectoryWorkspaceSize(std::size_t batch_size, std::size_t contact_count)
    -> std::size_t {
  return workspaceSize(batch_size, contact_count, trajectory_matrices, 0);
}

auto launchContactTrajectories(
    std::size_t batch_size, std::size_t contact_count, const double* matrix,
    const double* bias, const double* initial_state, const double* time,
    double* state, TrajectoryStatus* status, double* workspace,
    std::size_t workspace_size, cudaStream_t stream) -> cudaError_t {
  if (batch_size == 0) return cudaSuccess;
  const auto required = trajectoryWorkspaceSize(batch_size, contact_count);
  const auto blocks = (batch_size - 1) / threads_per_block + 1;
  if (required == 0 || workspace_size < required || !matrix || !bias ||
      !initial_state || !time || !state || !status || !workspace ||
      blocks > std::numeric_limits<int>::max()) {
    return cudaErrorInvalidValue;
  }
  evaluateBatch<<<static_cast<unsigned>(blocks), threads_per_block, 0, stream>>>(
      batch_size, 2 * contact_count, required / batch_size, matrix, bias,
      initial_state, time, state, status, workspace);
  return cudaGetLastError();
}

auto separationWorkspaceSize(std::size_t batch_size, std::size_t contact_count)
    -> std::size_t {
  return workspaceSize(batch_size, contact_count, separation_matrices, 2);
}

auto launchContactSeparations(
    std::size_t batch_size, std::size_t contact_count, const double* matrix,
    const double* bias, const double* initial_state, const SeparationSearch* search,
    double* separation_time, TrajectoryStatus* status, double* workspace,
    std::size_t workspace_size, cudaStream_t stream) -> cudaError_t {
  if (batch_size == 0) return cudaSuccess;
  const auto required = separationWorkspaceSize(batch_size, contact_count);
  const auto blocks = (batch_size - 1) / threads_per_block + 1;
  if (required == 0 || workspace_size < required || !matrix || !bias ||
      !initial_state || !search || !separation_time || !status || !workspace ||
      blocks > std::numeric_limits<int>::max()) {
    return cudaErrorInvalidValue;
  }
  searchBatch<<<static_cast<unsigned>(blocks), threads_per_block, 0, stream>>>(
      batch_size, 2 * contact_count, required / batch_size, matrix, bias,
      initial_state, search, separation_time, status, workspace);
  return cudaGetLastError();
}

namespace {

auto validShiftedContactStep(std::size_t batch_size, std::size_t capacity,
                             ShiftedContactInputs inputs, ShiftedContactOutputs outputs,
                             ShiftedContactScratch scratch, std::size_t normal_required)
    -> bool {
  const auto admm_required = spectralAdmmWorkspaceSize(batch_size, capacity);
  return !(normal_required == 0 || admm_required == 0 ||
      (batch_size - 1) / spectral_admm_problems_per_block + 1 > std::numeric_limits<int>::max() ||
      (batch_size - 1) / normal_threads_per_block + 1 > std::numeric_limits<int>::max() ||
      capacity > (std::numeric_limits<int>::max() - 1) / 2 ||
      !inputs.admm.problems || !inputs.admm.friction || !inputs.admm.inverse_mass ||
      !inputs.admm.velocity || !inputs.admm.acceleration || !inputs.normal_matrix ||
      !inputs.normal_bias || !inputs.initial_normal_state || !outputs.normal ||
      !outputs.normal_target || !outputs.admm.forces || !outputs.admm.results ||
      !scratch.accepted_problems || !scratch.normal || scratch.normal_size < normal_required ||
      !scratch.admm || scratch.admm_size < admm_required);
}

}  // namespace

auto launchShiftedContactStep(std::size_t batch_size, std::size_t capacity,
                              ShiftedContactInputs inputs,
                              ShiftedContactOutputs outputs,
                              ShiftedContactScratch scratch,
                              cudaStream_t stream) -> cudaError_t {
  if (batch_size == 0) return cudaSuccess;
  const auto normal_required = separationWorkspaceSize(batch_size, capacity);
  if (!validShiftedContactStep(batch_size, capacity, inputs, outputs, scratch, normal_required)) {
    return cudaErrorInvalidValue;
  }
  const auto blocks = (batch_size - 1) / normal_threads_per_block + 1;
  normalStepBatch<<<static_cast<unsigned>(blocks), normal_threads_per_block, 0, stream>>>(
      batch_size, capacity, inputs, outputs.normal, outputs.normal_target,
      scratch.accepted_problems, scratch.normal, normal_required / batch_size);
  if (const auto error = cudaGetLastError(); error != cudaSuccess) return error;
  auto admm_inputs = inputs.admm;
  admm_inputs.problems = scratch.accepted_problems;
  admm_inputs.normal_target = outputs.normal_target;
  return launchShiftedSpectralAdmm(batch_size, capacity, admm_inputs, outputs.admm,
                                   scratch.admm, scratch.admm_size, stream);
}

auto launchBodyContactStep(std::size_t batch_size, std::size_t capacity,
                           BodyContactInputs inputs, ShiftedContactOutputs outputs,
                           BodyContactAssembly assembly, ShiftedContactScratch scratch,
                           cudaStream_t stream) -> cudaError_t {
  if (batch_size == 0) return cudaSuccess;
  constexpr auto max_elements = std::numeric_limits<std::size_t>::max() / sizeof(double);
  if (capacity == 0 || capacity > max_elements / 6) return cudaErrorInvalidValue;
  const auto body_stride = 6 * capacity;
  const auto normal_required = separationWorkspaceSize(batch_size, capacity);
  const ShiftedContactInputs prepared{
      {assembly.problems, assembly.friction, assembly.relative_inverse_inertia,
       inputs.relative_velocity, nullptr, assembly.relative_acceleration,
       inputs.force_guess, inputs.dual_guess},
      assembly.normal_matrix, assembly.normal_bias, assembly.initial_normal_state, inputs.material};
  if (body_stride > max_elements / batch_size / body_stride ||
      !inputs.problems || !inputs.material || !inputs.inverse_inertia ||
      !inputs.free_acceleration ||
      !validShiftedContactStep(batch_size, capacity, prepared, outputs, scratch, normal_required)) {
    return cudaErrorInvalidValue;
  }
  const auto blocks = (batch_size - 1) / threads_per_block + 1;
  assembleBodyBatch<<<static_cast<unsigned>(blocks), threads_per_block, 0, stream>>>(
      batch_size, capacity, inputs, assembly);
  if (const auto error = cudaGetLastError(); error != cudaSuccess) return error;
  return launchShiftedContactStep(batch_size, capacity, prepared, outputs, scratch, stream);
}

}  // namespace sire::physics::contact::cuda
