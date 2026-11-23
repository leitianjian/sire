#include <limits>

#include <cuda_runtime.h>

#include "contact_history.hpp"
#include "rigid_body_dynamics.hpp"
#include "rigid_body_math.hpp"

namespace sire::physics::contact::cuda {
namespace {

constexpr unsigned threads_per_block = 128;

__device__ auto projectContactRow(
    const DetectedContact* detections, const std::size_t* selected_indices,
    std::size_t capacity, std::size_t body_count, std::size_t dof, std::size_t row,
    const double* velocities, const double* accelerations, const double* jacobians,
    const double* mass_factor, double* contact_factor, double* free_acceleration) -> bool {
  const auto source = selected_indices[row / 6];
  if (source >= capacity) return false;
  const auto& contact = detections[source];
  if (contact.body_a >= body_count || contact.body_b >= body_count) return false;

  const auto body = row % 6 < 3 ? contact.body_a : contact.body_b;
  const auto axis = row % 3;
  const double direction[3]{contact.frame[axis], contact.frame[4 + axis],
                            contact.frame[8 + axis]};
  const double point[3]{contact.frame[3], contact.frame[7], contact.frame[11]};
  double moment[3];
  detail::rigidCross3(point, direction, moment);

  double acceleration[3];
  detail::pointAcceleration(velocities + 6 * body, accelerations + 6 * body, point,
                            acceleration);
  const double projected_acceleration = direction[0] * acceleration[0] +
      direction[1] * acceleration[1] + direction[2] * acceleration[2];
  if (!::isfinite(projected_acceleration)) return false;
  *free_acceleration = projected_acceleration;

  const auto* jacobian = jacobians + 6 * body * dof;
  // A point-velocity row pairs the contact direction and its moment about
  // the world origin with each column of the body's spatial Jacobian.
  for (std::size_t i = 0; i < dof; ++i) {
    double value = 0.0;
    for (std::size_t k = 0; k < 3; ++k) {
      value += direction[k] * jacobian[k * dof + i] +
               moment[k] * jacobian[(k + 3) * dof + i];
    }
    const auto* lower = mass_factor + i * dof;
    for (std::size_t j = 0; j < i; ++j) value -= lower[j] * contact_factor[j];
    const double diagonal = lower[i];
    if (!(diagonal > 0.0) || !::isfinite(diagonal)) return false;
    value /= diagonal;
    if (!::isfinite(value)) return false;
    contact_factor[i] = value;
  }
  return true;
}

__global__ void projectContactBatch(
    std::size_t body_count, std::size_t dof, std::size_t capacity,
    const double* velocities, const double* accelerations, const double* jacobians,
    const double* mass_factor, const RigidDynamicsStatus* dynamics_status,
    const DetectedContact* detections, const std::size_t* selected_indices,
    AdmmProblem* problems, double* inverse_inertia, double* free_acceleration,
    double* contact_factor) {
  const auto env = static_cast<std::size_t>(blockIdx.x);
  auto& problem = problems[env];
  const auto count = problem.contacts;
  if (dynamics_status[env] != RigidDynamicsStatus::Success || count > capacity) {
    if (threadIdx.x == 0) problem.step = 0.0;
    return;
  }
  if (count == 0 || !(problem.step > 0.0) || !::isfinite(problem.step)) return;

  const auto body_offset = env * 6 * body_count;
  const auto detection_offset = env * capacity;
  const auto stride = 6 * capacity;
  const auto active = 6 * count;
  auto* factor = contact_factor + env * stride * dof;
  auto* acceleration = free_acceleration + env * stride;
  bool failed = false;
  for (auto row = static_cast<std::size_t>(threadIdx.x); row < active; row += blockDim.x) {
    if (!projectContactRow(detections + detection_offset, selected_indices + detection_offset,
                           capacity, body_count, dof, row, velocities + body_offset,
                           accelerations + body_offset, jacobians + body_offset * dof,
                           mass_factor + env * dof * dof, factor + row * dof,
                           acceleration + row)) {
      failed = true;
      break;
    }
  }
  if (__syncthreads_or(failed)) {
    if (threadIdx.x == 0) problem.step = 0.0;
    return;
  }

  // Each solved row is the transpose of L^-1 J_i^T. Their Gram matrix is
  // J M^-1 J^T.
  auto* inertia = inverse_inertia + env * stride * stride;
  for (auto entry = static_cast<std::size_t>(threadIdx.x); entry < active * active;
       entry += blockDim.x) {
    const auto row = entry / active;
    const auto col = entry % active;
    const auto* left = factor + row * dof;
    const auto* right = factor + col * dof;
    double value = 0.0;
    for (std::size_t k = 0; k < dof; ++k) value += left[k] * right[k];
    inertia[row * stride + col] = value;
    failed |= !::isfinite(value);
  }
  if (__syncthreads_or(failed) && threadIdx.x == 0) problem.step = 0.0;
}

}  // namespace

auto launchRigidBodyContact(
    std::size_t batch, std::size_t body_count, std::size_t joint_count, std::size_t capacity,
    const double* velocities, const double* accelerations, const double* jacobians,
    const double* mass_factor, const RigidDynamicsStatus* dynamics_status,
    const DetectedContact* detections, const std::size_t* selected_indices,
    AdmmProblem* problems, double* inverse_inertia, double* free_acceleration,
    double* contact_factor, cudaStream_t stream) -> cudaError_t {
  if (batch == 0) return cudaSuccess;
  constexpr auto max_bytes = std::numeric_limits<std::size_t>::max();
  constexpr auto max_elements = max_bytes / sizeof(double);
  if (batch > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
      body_count == 0 || capacity == 0 ||
      capacity > max_elements / 6 || joint_count > max_elements - 6 ||
      !velocities || !accelerations || !jacobians || !mass_factor || !dynamics_status ||
      !detections || !selected_indices || !problems || !inverse_inertia ||
      !free_acceleration || !contact_factor) {
    return cudaErrorInvalidValue;
  }
  const auto dof = 6 + joint_count;
  const auto stride = 6 * capacity;
  if (body_count > max_elements / batch / dof / 6 ||
      dof > max_elements / batch / dof || stride > max_elements / batch / stride ||
      stride > max_elements / batch / dof ||
      capacity > max_bytes / batch / sizeof(DetectedContact)) {
    return cudaErrorInvalidValue;
  }
  projectContactBatch<<<static_cast<unsigned>(batch), threads_per_block, 0, stream>>>(
      body_count, dof, capacity, velocities, accelerations, jacobians, mass_factor,
      dynamics_status, detections, selected_indices, problems, inverse_inertia,
      free_acceleration, contact_factor);
  return cudaGetLastError();
}

}  // namespace sire::physics::contact::cuda
