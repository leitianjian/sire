#include "rigid_body_joint_state.hpp"

#include <cmath>
#include <limits>

#include "rigid_body_math.hpp"

namespace sire::physics::contact::cuda {
namespace {

constexpr unsigned block_size = 128;
constexpr unsigned joint_read_block_size = 32;

__device__ auto invertPose(const double* pose, double* inverse) -> void {
  for (int i = 0; i < 3; ++i) {
    inverse[4 * i] = pose[i];
    inverse[4 * i + 1] = pose[4 + i];
    inverse[4 * i + 2] = pose[8 + i];
    inverse[4 * i + 3] = -(pose[i] * pose[3] + pose[4 + i] * pose[7] + pose[8 + i] * pose[11]);
  }
}

__device__ auto readJointCoordinates(const RigidBody& link, std::size_t body,
                                     const double* poses, const double* velocities,
                                     double* positions, double* rates) -> void {
  double parent[12], child[12];
  detail::composePose(poses + 12 * link.parent, link.parent_marker, parent);
  detail::composePose(poses + 12 * body, link.child_marker, child);

  // ARIS s_sov_axis_distance for axis 5 uses the X and Y columns of both frames.
  double sine = 0.0, cosine = 0.0;
  for (int i = 0; i < 12; i += 4) {
    sine += parent[i + 1] * child[i] - parent[i] * child[i + 1];
    cosine += parent[i] * child[i] + parent[i + 1] * child[i + 1];
  }
  positions[link.joint] = ::atan2(sine, cosine);

  const auto* parent_angular = velocities + 6 * link.parent + 3;
  const auto* child_angular = velocities + 6 * body + 3;
  double rate = 0.0;
  for (int i = 0; i < 3; ++i) {
    rate += parent[4 * i + 2] * (child_angular[i] - parent_angular[i]);
  }
  rates[link.joint] = rate;
}

__device__ auto readJointAcceleration(const RigidBody& link, std::size_t body,
                                       const double* poses, const double* velocities,
                                       const double* accelerations, double* result) -> void {
  // The angular part of ARIS s_inv_as2as subtracts the moving-frame term.
  double derivative[3];
  detail::rigidCross3(velocities + 6 * link.parent + 3, velocities + 6 * body + 3, derivative);
  const auto* parent_angular = accelerations + 6 * link.parent + 3;
  const auto* child_angular = accelerations + 6 * body + 3;
  const auto* parent = poses + 12 * link.parent;
  const auto* marker = link.parent_marker;
  double acceleration = 0.0;
  for (int i = 0; i < 3; ++i) {
    const auto* row = parent + 4 * i;
    const double axis = row[0] * marker[2] + row[1] * marker[6] + row[2] * marker[10];
    acceleration += axis * (child_angular[i] - parent_angular[i] - derivative[i]);
  }
  result[link.joint] = acceleration;
}

__global__ void readJointStateBatch(
    RigidBodyTree model, const double* poses, const double* velocities, const double* accelerations,
    double* positions, double* rates, double* joint_accelerations,
    const ContactHistoryConfig* events, const bool* projected) {
  const auto env = static_cast<std::size_t>(blockIdx.x);
  if (events && events[env].suspended) return;
  const bool read_coordinates = !projected || projected[env];
  const auto position_offset = env * model.joint_count;
  const auto spatial_offset = env * model.body_count * 6;
  poses += 2 * spatial_offset;
  velocities += spatial_offset;
  for (auto body = static_cast<std::size_t>(threadIdx.x); body < model.body_count;
       body += blockDim.x) {
    const auto& link = model.bodies[body];
    if (link.joint < 0) continue;
    if (read_coordinates) {
      readJointCoordinates(link, body, poses, velocities, positions + position_offset,
                           rates + position_offset);
    }
    if (accelerations) {
      readJointAcceleration(link, body, poses, velocities, accelerations + spatial_offset,
                            joint_accelerations + position_offset);
    }
  }
}

__device__ auto rebuildBody(const RigidBody& link, std::size_t body, const double* positions,
                            const double* rates, double* poses, double* velocities) -> void {
  double parent[12], inverse[12];
  detail::composePose(poses + 12 * link.parent, link.parent_marker, parent);
  invertPose(link.child_marker, inverse);
  auto* child = poses + 12 * body;
  const auto* parent_velocity = velocities + 6 * link.parent;
  auto* child_velocity = velocities + 6 * body;
  if (link.joint < 0) {
    detail::composePose(parent, inverse, child);
    for (int i = 0; i < 6; ++i) child_velocity[i] = parent_velocity[i];
    return;
  }

  double sine, cosine;
  ::sincos(positions[link.joint], &sine, &cosine);
  const double rotation[12]{cosine, -sine, 0.0, 0.0, sine, cosine, 0.0, 0.0,
                             0.0, 0.0, 1.0, 0.0};
  double rotated_parent[12];
  detail::composePose(parent, rotation, rotated_parent);
  detail::composePose(rotated_parent, inverse, child);

  const double axis[3]{parent[2], parent[6], parent[10]};
  const double point[3]{parent[3], parent[7], parent[11]};
  double linear[3];
  detail::rigidCross3(point, axis, linear);
  const auto rate = rates[link.joint];
  for (int i = 0; i < 3; ++i) {
    child_velocity[i] = parent_velocity[i] + rate * linear[i];
    child_velocity[i + 3] = parent_velocity[i + 3] + rate * axis[i];
  }
}

__global__ void rebuildBodyStateBatch(
    std::size_t batch, RigidBodyTree model, const std::size_t* order, const bool* mask,
    const double* positions, const double* rates, double* poses, double* velocities) {
  const auto env = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (env >= batch || (mask && !mask[env])) return;
  const auto position_offset = env * model.joint_count;
  const auto spatial_offset = env * model.body_count * 6;
  for (std::size_t i = 1; i < model.body_count - 1; ++i) {
    const auto body = order[i];
    rebuildBody(model.bodies[body], body, positions ? positions + position_offset : nullptr,
                rates ? rates + position_offset : nullptr, poses + 2 * spatial_offset,
                velocities + spatial_offset);
  }
}

auto validBodyLayout(std::size_t batch, RigidBodyTree model) -> bool {
  constexpr auto max_bytes = std::numeric_limits<std::size_t>::max();
  return model.bodies && model.body_count >= 2 && model.ground < model.body_count &&
         model.joint_count <= model.body_count - 2 &&
         model.body_count <= max_bytes / sizeof(RigidBody) &&
         model.body_count <= max_bytes / (12 * sizeof(double)) / batch;
}

}  // namespace

auto launchReadRigidJointState(
    std::size_t batch, RigidBodyTree model, const double* poses, const double* velocities,
    const double* accelerations, double* positions, double* rates, double* joint_accelerations,
    cudaStream_t stream, const ContactHistoryConfig* events, const bool* projected) -> cudaError_t {
  if (batch == 0) return cudaSuccess;
  if (batch > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
      !validBodyLayout(batch, model) || !poses || !velocities ||
      static_cast<bool>(accelerations) != static_cast<bool>(joint_accelerations)) {
    return cudaErrorInvalidValue;
  }
  if (model.joint_count == 0) return cudaSuccess;
  if (!positions || !rates) return cudaErrorInvalidValue;
  readJointStateBatch<<<static_cast<unsigned>(batch), joint_read_block_size, 0, stream>>>(
      model, poses, velocities, accelerations, positions, rates, joint_accelerations,
      events, projected);
  return cudaGetLastError();
}

auto launchRebuildRigidBodyState(
    std::size_t batch, RigidBodyTree model, const std::size_t* order, const bool* mask,
    const double* positions, const double* rates, double* poses, double* velocities,
    cudaStream_t stream) -> cudaError_t {
  if (batch == 0) return cudaSuccess;
  const auto blocks = (batch - 1) / block_size + 1;
  if (blocks > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
      !validBodyLayout(batch, model) || !order || !poses || !velocities ||
      (model.joint_count != 0 && (!positions || !rates))) {
    return cudaErrorInvalidValue;
  }
  rebuildBodyStateBatch<<<static_cast<unsigned>(blocks), block_size, 0, stream>>>(
      batch, model, order, mask, positions, rates, poses, velocities);
  return cudaGetLastError();
}

}  // namespace sire::physics::contact::cuda
