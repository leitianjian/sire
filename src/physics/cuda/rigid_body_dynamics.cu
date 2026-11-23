#include "rigid_body_dynamics.hpp"

#include <cmath>
#include <limits>

#include "rigid_body_math.hpp"

namespace sire::physics::contact::cuda {
namespace {

constexpr unsigned block_size = 128;

__device__ auto jointScrew(const double* pose, const double* marker, double* screw) -> void {
  double world[12];
  detail::composePose(pose, marker, world);
  screw[0] = world[7] * world[10] - world[11] * world[6];
  screw[1] = world[11] * world[2] - world[3] * world[10];
  screw[2] = world[3] * world[6] - world[7] * world[2];
  screw[3] = world[2];
  screw[4] = world[6];
  screw[5] = world[10];
}

__device__ auto buildBodyTerms(RigidBodyTree model, std::size_t body, std::size_t dof,
                               const double* poses, const double* velocities,
                               const double* wrenches, double* inertia, double* jacobian,
                               double* bias, double* rhs) -> void {
  const bool moving = body != model.ground;
  for (std::size_t i = 0; i < 6; ++i) {
    bias[i] = 0.0;
    rhs[i] = 0.0;
    for (std::size_t j = 0; j < dof; ++j) jacobian[i * dof + j] = moving && i == j ? 1.0 : 0.0;
  }
  if (!moving) {
    for (std::size_t i = 0; i < 36; ++i) inertia[i] = 0.0;
    return;
  }

  for (auto child = body; model.bodies[child].parent != child; child = model.bodies[child].parent) {
    const auto& link = model.bodies[child];
    // The ARIS constraint equation uses the motion cross product of the
    // parent and child velocities. Any component along the free joint axis
    // changes only the particular solution's generalized coordinates.
    double cross[6];
    detail::crossMotion(velocities + 6 * link.parent, velocities + 6 * child, cross);
    for (std::size_t k = 0; k < 6; ++k) bias[k] += cross[k];
    if (link.joint >= 0) {
      double screw[6];
      jointScrew(poses + 12 * child, link.child_marker, screw);
      const auto column = 6 + static_cast<std::size_t>(link.joint);
      for (std::size_t k = 0; k < 6; ++k) jacobian[k * dof + column] = screw[k];
    }
  }

  detail::worldInertia(model.bodies[body].inertia, poses + 12 * body, inertia);
  const auto* v = velocities + 6 * body;
  double momentum[6]{};
  for (std::size_t i = 0; i < 6; ++i) {
    for (std::size_t j = 0; j < 6; ++j) momentum[i] += inertia[i * 6 + j] * v[j];
  }
  double gyroscopic[6];
  detail::crossForce(v, momentum, gyroscopic);
  for (std::size_t i = 0; i < 6; ++i) {
    rhs[i] = (wrenches ? wrenches[6 * body + i] : 0.0) - gyroscopic[i];
    for (std::size_t j = 0; j < 6; ++j) {
      rhs[i] += inertia[i * 6 + j] * (model.gravity[j] - bias[j]);
    }
  }
}

__device__ auto factorMass(double* lower, std::size_t dof) -> RigidDynamicsStatus {
  for (std::size_t i = 0; i < dof; ++i) {
    for (std::size_t j = 0; j <= i; ++j) {
      double entry = lower[i * dof + j];
      for (std::size_t k = 0; k < j; ++k) entry -= lower[i * dof + k] * lower[j * dof + k];
      if (!::isfinite(entry)) return RigidDynamicsStatus::NumericalFailure;
      if (i == j) {
        if (entry <= 0) return RigidDynamicsStatus::NonPositiveMass;
        lower[i * dof + i] = ::sqrt(entry);
      } else {
        lower[i * dof + j] = entry / lower[j * dof + j];
      }
    }
  }
  return RigidDynamicsStatus::Success;
}

__device__ auto solveMass(const double* lower, double* rhs, std::size_t dof) -> bool {
  for (std::size_t i = 0; i < dof; ++i) {
    for (std::size_t j = 0; j < i; ++j) rhs[i] -= lower[i * dof + j] * rhs[j];
    rhs[i] /= lower[i * dof + i];
  }
  for (std::size_t i = dof; i-- > 0;) {
    for (std::size_t j = i + 1; j < dof; ++j) rhs[i] -= lower[j * dof + i] * rhs[j];
    rhs[i] /= lower[i * dof + i];
    if (!::isfinite(rhs[i])) return false;
  }
  return true;
}

__device__ auto writeAcceleration(std::size_t rows, std::size_t dof, const double* jacobian,
                                   const double* acceleration, const double* base, double* output,
                                   RigidDynamicsStatus& status) -> void {
  bool valid = true;
  for (std::size_t row = threadIdx.x; row < rows; row += blockDim.x) {
    double value = base[row];
    for (std::size_t j = 0; j < dof; ++j) value += jacobian[row * dof + j] * acceleration[j];
    output[row] = value;
    valid = valid && ::isfinite(value);
  }
  if (__syncthreads_and(valid) == 0 && threadIdx.x == 0) status = RigidDynamicsStatus::NumericalFailure;
}

__global__ void solveTree(RigidBodyTree model, RigidDynamicsInputs inputs,
                          RigidDynamicsOutputs outputs, double* scratch) {
  const std::size_t env = blockIdx.x;
  if (inputs.configs && (inputs.configs[env].suspended ||
                         outputs.status[env] != RigidDynamicsStatus::Success)) return;
  const auto dof = 6 + model.joint_count;
  const auto rows = 6 * model.body_count;
  const auto* poses = inputs.poses + env * 2 * rows;
  const auto* vel = inputs.velocities + env * rows;
  const auto* wrench = inputs.wrenches ? inputs.wrenches + env * rows : nullptr;
  auto* jacobian = outputs.jacobians + env * rows * dof;
  auto* bias = outputs.accelerations + env * rows;
  auto* factor = outputs.mass_factor + env * dof * dof;
  auto* inertia = scratch + env * rows * (7 + dof);
  auto* inertia_jacobian = inertia + 6 * rows;
  auto* rhs = inertia_jacobian + rows * dof;
  extern __shared__ double generalized[];

  for (std::size_t body = threadIdx.x; body < model.body_count; body += blockDim.x) {
    const auto row = 6 * body;
    buildBodyTerms(model, body, dof, poses, vel, wrench, inertia + 6 * row,
                   jacobian + row * dof, bias + row, rhs + row);
  }
  __syncthreads();

  for (std::size_t index = threadIdx.x; index < rows * dof; index += blockDim.x) {
    const auto row = index / dof;
    const auto col = index % dof;
    const auto body = row / 6;
    double sum = 0.0;
    for (std::size_t k = 0; k < 6; ++k) {
      sum += inertia[row * 6 + k] * jacobian[(6 * body + k) * dof + col];
    }
    inertia_jacobian[index] = sum;
  }
  __syncthreads();

  for (std::size_t index = threadIdx.x; index < dof * dof; index += blockDim.x) {
    const auto row = index / dof;
    const auto col = index % dof;
    double value = 0.0;
    if (col <= row) {
      for (std::size_t k = 0; k < rows; ++k) {
        value += jacobian[k * dof + row] * inertia_jacobian[k * dof + col];
      }
    }
    factor[index] = value;
  }
  for (std::size_t col = threadIdx.x; col < dof; col += blockDim.x) {
    double value = inputs.torques && col >= 6 ? inputs.torques[env * model.joint_count + col - 6] : 0.0;
    for (std::size_t k = 0; k < rows; ++k) value += jacobian[k * dof + col] * rhs[k];
    generalized[col] = value;
  }
  __syncthreads();

  auto& status = outputs.status[env];
  if (threadIdx.x == 0) {
    status = factorMass(factor, dof);
    if (status == RigidDynamicsStatus::Success && !solveMass(factor, generalized, dof)) {
      status = RigidDynamicsStatus::NumericalFailure;
    }
  }
  __syncthreads();
  if (status == RigidDynamicsStatus::Success) {
    writeAcceleration(rows, dof, jacobian, generalized, bias, bias, status);
  }
}

__global__ void applyForces(std::size_t rows, std::size_t dof, const double* wrenches,
                            const double* reactions, const double* free, const double* jacobians,
                            const double* mass_factor, RigidDynamicsStatus* states,
                            double* accelerations, double* scratch, const ContactHistoryConfig* configs) {
  const std::size_t env = blockIdx.x;
  if (configs && configs[env].suspended) return;
  auto& status = states[env];
  if (status != RigidDynamicsStatus::Success) return;
  const auto* wrench = wrenches ? wrenches + env * rows : nullptr;
  const auto* jacobian = jacobians + env * rows * dof;
  auto* generalized = scratch + env * dof;
  bool nonzero = false;
  for (std::size_t col = threadIdx.x; col < dof; col += blockDim.x) {
    double value = 0.0;
    if (wrench) {
      for (std::size_t row = 0; row < rows; ++row) value += jacobian[row * dof + col] * wrench[row];
    } else if (col >= 6) {
      value = reactions[env * (dof - 6) + col - 6];
    }
    nonzero = nonzero || value != 0.0;
    generalized[col] = value;
  }
  if (__syncthreads_or(nonzero) == 0 && free == accelerations) return;
  if (threadIdx.x == 0 && !solveMass(mass_factor + env * dof * dof, generalized, dof)) {
    status = RigidDynamicsStatus::NumericalFailure;
  }
  __syncthreads();
  if (status == RigidDynamicsStatus::Success) {
    writeAcceleration(rows, dof, jacobian, generalized, free + env * rows,
                      accelerations + env * rows, status);
  }
}

auto validDimensions(std::size_t batch, std::size_t bodies, std::size_t joints) -> bool {
  if (batch > std::numeric_limits<int>::max() || bodies < 2 || joints > bodies - 2 ||
      joints > std::numeric_limits<std::size_t>::max() - 13) return false;
  constexpr auto max_doubles = std::numeric_limits<std::size_t>::max() / sizeof(double);
  const auto dof = 6 + joints;
  return dof <= max_doubles / batch / dof && bodies <= max_doubles / batch / 6 / (7 + dof);
}

}  // namespace

auto launchRigidBodyDynamics(
    std::size_t batch, RigidBodyTree model, RigidDynamicsInputs inputs,
    RigidDynamicsOutputs outputs, double* scratch, cudaStream_t stream) -> cudaError_t {
  if (batch == 0) return cudaSuccess;
  if (!validDimensions(batch, model.body_count, model.joint_count) || !model.bodies ||
      model.ground >= model.body_count || !inputs.poses || !inputs.velocities || !outputs.accelerations ||
      !outputs.jacobians || !outputs.mass_factor || !outputs.status || !scratch) return cudaErrorInvalidValue;
  const auto shared_bytes = (6 + model.joint_count) * sizeof(double);
  solveTree<<<static_cast<unsigned>(batch), block_size, shared_bytes, stream>>>(model, inputs, outputs, scratch);
  return cudaGetLastError();
}

auto launchApplyRigidBodyWrenches(
    std::size_t batch, std::size_t body_count, std::size_t joint_count,
    const double* wrenches, const double* free_accelerations, const double* jacobians,
    const double* mass_factor, RigidDynamicsStatus* status, double* accelerations,
    double* generalized_scratch, cudaStream_t stream, const ContactHistoryConfig* configs) -> cudaError_t {
  if (batch == 0) return cudaSuccess;
  if (!validDimensions(batch, body_count, joint_count) || !wrenches || !free_accelerations ||
      !jacobians || !mass_factor || !status || !accelerations || !generalized_scratch) return cudaErrorInvalidValue;
  applyForces<<<static_cast<unsigned>(batch), block_size, 0, stream>>>(
      6 * body_count, 6 + joint_count, wrenches, nullptr, free_accelerations, jacobians, mass_factor,
      status, accelerations, generalized_scratch, configs);
  return cudaGetLastError();
}

auto launchApplyRigidJointReactions(
    std::size_t batch, std::size_t body_count, std::size_t joint_count,
    const double* reactions, const double* jacobians, const double* mass_factor,
    RigidDynamicsStatus* status, double* accelerations, double* generalized_scratch,
    const ContactHistoryConfig* configs, cudaStream_t stream) -> cudaError_t {
  if (batch == 0 || joint_count == 0) return cudaSuccess;
  if (!validDimensions(batch, body_count, joint_count) || !reactions || !jacobians || !mass_factor ||
      !status || !accelerations || !generalized_scratch) return cudaErrorInvalidValue;
  applyForces<<<static_cast<unsigned>(batch), block_size, 0, stream>>>(
      6 * body_count, 6 + joint_count, nullptr, reactions, accelerations, jacobians, mass_factor,
      status, accelerations, generalized_scratch, configs);
  return cudaGetLastError();
}

}  // namespace sire::physics::contact::cuda
