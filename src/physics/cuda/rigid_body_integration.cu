#include "rigid_body_integration.hpp"

#include <limits>

#include "rigid_body_math.hpp"

namespace sire::physics::contact::cuda {
namespace {

constexpr int block_size = 32;
constexpr int max_projection_iterations = 100;
constexpr double position_tolerance = 1e-10;
constexpr double twist_limit = 1e4;

__device__ void relativePose(const double* from, const double* to, double* relative) {
  const double displacement[3] = {to[3] - from[3], to[7] - from[7], to[11] - from[11]};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      relative[4 * i + j] = from[i] * to[j] + from[4 + i] * to[4 + j] +
                            from[8 + i] * to[8 + j];
    }
    relative[4 * i + 3] = from[i] * displacement[0] + from[4 + i] * displacement[1] +
                          from[8 + i] * displacement[2];
  }
}

__device__ void adjoint(const double* pose, const double* local, double* world) {
  for (int i = 0; i < 3; ++i) {
    const auto* row = pose + 4 * i;
    world[i] = row[0] * local[0] + row[1] * local[1] + row[2] * local[2];
    world[i + 3] = row[0] * local[3] + row[1] * local[4] + row[2] * local[5];
  }
  const double translation[3] = {pose[3], pose[7], pose[11]};
  double cross[3];
  detail::rigidCross3(translation, world + 3, cross);
  for (int i = 0; i < 3; ++i) world[i] += cross[i];
}

__device__ void inverseAdjoint(const double* pose, const double* world, double* local) {
  const double translation[3] = {pose[3], pose[7], pose[11]};
  double offset[3];
  detail::rigidCross3(world + 3, translation, offset);
  for (int i = 0; i < 3; ++i) offset[i] += world[i];
  for (int i = 0; i < 3; ++i) {
    local[i] = pose[i] * offset[0] + pose[4 + i] * offset[1] + pose[8 + i] * offset[2];
    local[i + 3] = pose[i] * world[3] + pose[4 + i] * world[4] + pose[8 + i] * world[5];
  }
}

__device__ void rotationVector(const double* pose, double* rotation) {
  // ARIS s_rm2rq selects the largest squared quaternion component and makes w nonnegative.
  constexpr int row[4][4] = {{-1, 0, 0, 2}, {1, -1, 1, 0}, {2, 2, -1, 1}, {2, 0, 1, -1}};
  constexpr int col[4][4] = {{-1, 1, 2, 1}, {0, -1, 2, 2}, {0, 1, -1, 0}, {1, 2, 0, -1}};
  const double squared[4] = {
      (1 + pose[0] - pose[5] - pose[10]) / 4,
      (1 + pose[5] - pose[0] - pose[10]) / 4,
      (1 + pose[10] - pose[0] - pose[5]) / 4,
      (1 + pose[0] + pose[5] + pose[10]) / 4};
  int largest = 0;
  for (int i = 1; i < 4; ++i) {
    if (squared[i] > squared[largest]) largest = i;
  }
  double quaternion[4];
  quaternion[largest] = sqrt(squared[largest]);
  for (int i = 0; i < 4; ++i) {
    if (i == largest) continue;
    const int r = row[largest][i], c = col[largest][i];
    const double sign = largest == 3 || i == 3 ? -1.0 : 1.0;
    quaternion[i] = (pose[4 * r + c] + sign * pose[4 * c + r]) / 4.0 / quaternion[largest];
  }
  const double norm = sqrt(quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] +
                           quaternion[2] * quaternion[2]);
  const double half_angle = atan2(norm, fabs(quaternion[3]));
  const double scale = (quaternion[3] < 0 ? -2.0 : 2.0) / detail::rigidSinc(half_angle);
  for (int i = 0; i < 3; ++i) rotation[i] = scale * quaternion[i];
}

__device__ void fixedJointError(const double* relative, double* correction) {
  // ARIS s_pm2ps inverts the translation map of s_ps2pm with this small-angle cutoff.
  rotationVector(relative, correction + 3);
  const double x = correction[3], y = correction[4], z = correction[5];
  const double norm = sqrt(x * x + y * y + z * z);
  const double c = detail::rigidSinc(norm);
  const double a = norm < 1e-3 ? 1.0 / 6.0 : (1.0 - c) / norm / norm;
  const double half_sinc = detail::rigidSinc(0.5 * norm);
  const double b = 0.5 * half_sinc * half_sinc;
  const double columns[3][3] = {
      {a * x * x + c, a * y * x + b * z, a * z * x - b * y},
      {a * x * y - b * z, a * y * y + c, a * z * y + b * x},
      {a * x * z + b * y, a * y * z - b * x, a * z * z + c}};
  const double translation[3] = {relative[3], relative[7], relative[11]};
  double cofactor[3];
  detail::rigidCross3(columns[1], columns[2], cofactor);
  const double det = columns[0][0] * cofactor[0] + columns[0][1] * cofactor[1] +
                     columns[0][2] * cofactor[2];
  for (int i = 0; i < 3; ++i) {
    detail::rigidCross3(columns[(i + 1) % 3], columns[(i + 2) % 3], cofactor);
    correction[i] = (translation[0] * cofactor[0] + translation[1] * cofactor[1] +
                     translation[2] * cofactor[2]) / det;
  }
}

__device__ bool applyDisplacement(const double* displacement, double* pose) {
  double increment[12];
  detail::screwPose(displacement, increment);
  detail::composePose(increment, pose, pose);
  for (int i = 0; i < 12; ++i) {
    if (!isfinite(pose[i])) return false;
  }
  return true;
}

__device__ bool validKick(const double* velocities, const double* accelerations, double dt) {
  for (int i = 0; i < 6; ++i) {
    const double velocity = velocities[i];
    const double acceleration = accelerations[i];
    const double increment = dt * acceleration;
    const double integrated = velocity + increment;
    if (!isfinite(velocity) || !isfinite(acceleration) || !isfinite(increment) ||
        !isfinite(integrated) || fabs(velocity) > twist_limit ||
        fabs(increment) > twist_limit || fabs(integrated) > twist_limit) {
      return false;
    }
  }
  return true;
}

__device__ void accumulatedCorrection(RigidBodyTree model, std::size_t body,
                                      const double* corrections, double* result) {
  for (int k = 0; k < 6; ++k) result[k] = 0.0;
  while (model.bodies[body].parent != body) {
    for (int k = 0; k < 6; ++k) result[k] += corrections[6 * body + k];
    body = model.bodies[body].parent;
  }
}

__device__ bool projectPosition(RigidBodyTree model, const std::size_t* order,
                                double* poses, double* corrections) {
  const auto moving = model.body_count - 1;
  for (int iteration = 0; iteration <= max_projection_iterations; ++iteration) {
    bool valid = true, needs_correction = false;
    for (std::size_t i = threadIdx.x + 1; i < moving; i += blockDim.x) {
      const auto child = order[i];
      const auto& body = model.bodies[child];
      double child_marker[12], parent_marker[12], relative[12], error[6], world[6];
      detail::composePose(poses + 12 * child, body.child_marker, child_marker);
      detail::composePose(poses + 12 * body.parent, body.parent_marker, parent_marker);
      relativePose(child_marker, parent_marker, relative);
      if (body.joint < 0) {
        fixedJointError(relative, error);
      } else {
        error[0] = relative[3];
        error[1] = relative[7];
        error[2] = relative[11];
        error[3] = -relative[6];
        error[4] = relative[2];
        error[5] = 0.0;
      }
      adjoint(child_marker, error, world);
      for (int j = 0; j < 6; ++j) {
        valid = valid && isfinite(error[j]) && isfinite(world[j]);
        needs_correction = needs_correction || fabs(error[j]) >= position_tolerance;
        corrections[6 * child + j] = world[j];
      }
    }
    if (!__syncthreads_and(valid)) return false;
    if (!__syncthreads_or(needs_correction)) return true;
    if (iteration == max_projection_iterations) break;

    // The complete set of edge errors comes from the same iterate. Summing
    // the ancestor corrections gives each body's displacement.
    for (std::size_t i = threadIdx.x + 1; i < moving; i += blockDim.x) {
      const auto child = order[i];
      double correction[6];
      accumulatedCorrection(model, child, corrections, correction);
      const bool applied = applyDisplacement(correction, poses + 12 * child);
      valid = valid && applied;
    }
    if (!__syncthreads_and(valid)) return false;
  }
  return false;
}

__device__ bool projectVelocity(RigidBodyTree model, const std::size_t* order,
                                const double* poses, double* velocities, double* corrections) {
  const auto moving = model.body_count - 1;
  for (std::size_t i = threadIdx.x + 1; i < moving; i += blockDim.x) {
    const auto child = order[i];
    const auto& body = model.bodies[child];
    double child_marker[12], difference[6], local[6];
    detail::composePose(poses + 12 * child, body.child_marker, child_marker);
    for (int j = 0; j < 6; ++j) {
      difference[j] = velocities[6 * body.parent + j] - velocities[6 * child + j];
    }
    inverseAdjoint(child_marker, difference, local);
    if (body.joint >= 0) local[5] = 0.0;
    adjoint(child_marker, local, corrections + 6 * child);
  }
  __syncthreads();
  bool valid = true;
  for (std::size_t i = threadIdx.x + 1; i < moving; i += blockDim.x) {
    const auto child = order[i];
    double correction[6];
    accumulatedCorrection(model, child, corrections, correction);
    for (int j = 0; j < 6; ++j) {
      velocities[6 * child + j] += correction[j];
      valid = valid && isfinite(velocities[6 * child + j]);
    }
  }
  return __syncthreads_and(valid);
}

__global__ void integrateTree(
    RigidBodyTree model, const std::size_t* order,
    const double* steps, const double* accelerations, double* poses,
    double* velocities, RigidDynamicsStatus* status, double* corrections) {
  const auto env = static_cast<std::size_t>(blockIdx.x);
  if (status[env] != RigidDynamicsStatus::Success || steps[env] == 0.0) return;
  const double dt = steps[env];
  if (!isfinite(dt) || dt < 0.0) {
    if (threadIdx.x == 0) status[env] = RigidDynamicsStatus::InvalidInput;
    return;
  }

  const auto moving = model.body_count - 1;
  const auto body_offset = env * model.body_count;
  poses += 12 * body_offset;
  velocities += 6 * body_offset;
  accelerations += 6 * body_offset;
  corrections += 6 * body_offset;
  bool valid = true;
  for (std::size_t i = threadIdx.x; i < moving; i += blockDim.x) {
    const auto body = order[i];
    valid = valid && validKick(velocities + 6 * body, accelerations + 6 * body, dt);
  }
  if (!__syncthreads_and(valid)) {
    if (threadIdx.x == 0) status[env] = RigidDynamicsStatus::NumericalFailure;
    return;
  }

  for (std::size_t i = threadIdx.x; i < moving; i += blockDim.x) {
    const auto body = order[i];
    double displacement[6];
    for (int j = 0; j < 6; ++j) {
      velocities[6 * body + j] += dt * accelerations[6 * body + j];
      displacement[j] = dt * velocities[6 * body + j];
    }
    const bool applied = applyDisplacement(displacement, poses + 12 * body);
    valid = valid && applied;
  }
  if (!__syncthreads_and(valid)) {
    if (threadIdx.x == 0) status[env] = RigidDynamicsStatus::NumericalFailure;
    return;
  }

  if (!projectPosition(model, order, poses, corrections) ||
      !projectVelocity(model, order, poses, velocities, corrections)) {
    if (threadIdx.x == 0) status[env] = RigidDynamicsStatus::NumericalFailure;
  }
}

}  // namespace

auto launchIntegrateRigidBodyTree(
    std::size_t batch, RigidBodyTree model, const std::size_t* order,
    const double* steps, const double* accelerations, double* poses,
    double* velocities, RigidDynamicsStatus* status, double* corrections,
    cudaStream_t stream) -> cudaError_t {
  if (batch == 0) return cudaSuccess;
  constexpr auto max_doubles = std::numeric_limits<std::size_t>::max() / sizeof(double);
  if (batch > std::numeric_limits<int>::max() || model.body_count < 2 ||
      model.body_count > max_doubles / batch / 12 || model.ground >= model.body_count ||
      !model.bodies || !order || !steps || !accelerations || !poses || !velocities ||
      !status || !corrections) {
    return cudaErrorInvalidValue;
  }
  integrateTree<<<static_cast<unsigned>(batch), block_size, 0, stream>>>(
      model, order, steps, accelerations, poses, velocities, status, corrections);
  return cudaGetLastError();
}

}  // namespace sire::physics::contact::cuda
