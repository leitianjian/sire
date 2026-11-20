#include "flat_ground_collision.hpp"

#include <cmath>
#include <limits>

namespace sire::physics::contact::cuda {
namespace {

constexpr unsigned threads_per_block = 32;
constexpr double zero_depth = std::numeric_limits<double>::epsilon();
constexpr double collision_threshold = 1e-12;  // COAL's default for double.
constexpr double max_distance = std::numeric_limits<double>::max();

struct Witness {
  double point[3]{}, normal[3]{};
  double depth{0.0};
};

__device__ auto sphereWitness(const GroundBox& ground, const double* center,
                              double radius, Witness& result) -> void {
  // COAL's box-sphere witness clamps the center to the finite box. Inside
  // the box it chooses the nearest face, resolving ties by axis order.
  double toward_box[3];
  double nearest_face = max_distance;
  int axis = 0;
  double axis_sign = 1.0;
  bool outside = false;
  for (int k = 0; k < 3; ++k) {
    const double local = center[k] - ground.center[k];
    const double extent = ground.half_extent[k];
    const double face_distance = extent - ::fabs(local);
    if (face_distance < -radius) return;  // AABB separation.
    const double clamped = ::fmax(-extent, ::fmin(extent, local));
    toward_box[k] = clamped + ground.center[k] - center[k];
    outside = outside || face_distance < 0.0;
    if (face_distance < nearest_face) {
      nearest_face = face_distance;
      axis = k;
      axis_sign = local < 0.0 ? -1.0 : 1.0;
    }
  }
  double distance = -nearest_face - radius;
  if (outside) {
    const double norm = ::sqrt(toward_box[0] * toward_box[0] +
                               toward_box[1] * toward_box[1] +
                               toward_box[2] * toward_box[2]);
    if (norm == 0.0) return;
    distance = norm - radius;
    for (int k = 0; k < 3; ++k) result.normal[k] = toward_box[k] / -norm;
  } else {
    result.normal[axis] = axis_sign;
  }
  if (distance > collision_threshold || ::fabs(distance) <= zero_depth) return;
  result.depth = ::fabs(distance);
  for (int k = 0; k < 3; ++k) {
    const double on_sphere = center[k] - radius * result.normal[k];
    const double on_box = outside && distance > 0 ? center[k] + toward_box[k]
                                                  : on_sphere - distance * result.normal[k];
    result.point[k] = (on_sphere + on_box) * 0.5;
  }
}

__device__ auto boxExtent(const double* row, const double* size) -> double {
  return ::fabs(row[0]) * size[0] + ::fabs(row[1]) * size[1] + ::fabs(row[2]) * size[2];
}

__device__ auto cylinderExtent(const double* row, const double* size) -> double {
  return ::sqrt(row[0] * row[0] + row[1] * row[1]) * size[0] + ::fabs(row[2]) * size[1];
}

__device__ auto opposingExtent(double direction, double half_extent) -> double {
  // The center of a face or edge is a valid support point at an exact tie.
  // Suppress roundoff below the precision of the body pose.
  if (::fabs(direction) <= 1e-14) return 0.0;
  return -::copysign(half_extent, direction);
}

__device__ auto horizontalWitness(const GroundBox& ground, const GroundCollider& shape,
                                  const double* pose, Witness& result) -> GroundCollisionStatus {
  const bool box = shape.shape == GroundShape::Box;
  double offset[3], extent[3];
  for (int k = 0; k < 3; ++k) {
    offset[k] = pose[4 * k + 3] - ground.center[k];
    extent[k] = box ? boxExtent(pose + 4 * k, shape.size)
                    : cylinderExtent(pose + 4 * k, shape.size);
    if (::fabs(offset[k]) > ground.half_extent[k] + extent[k]) return GroundCollisionStatus::Success;
  }
  const double sign = offset[2] < 0 ? -1.0 : 1.0;
  const double face = ground.center[2] + sign * ground.half_extent[2];
  const double depth = ground.half_extent[2] + extent[2] - ::fabs(offset[2]);
  if (depth <= zero_depth) return GroundCollisionStatus::Success;
  // The projected support remains on the slab for translations no longer
  // than this penetration. A side or edge can be closer in other scenes.
  if (::fabs(offset[0]) + extent[0] + depth >= ground.half_extent[0] ||
      ::fabs(offset[1]) + extent[1] + depth >= ground.half_extent[1]) {
    return GroundCollisionStatus::UnsupportedBoundary;
  }

  double support[3];
  if (box) {
    for (int k = 0; k < 3; ++k) support[k] = opposingExtent(sign * pose[8 + k], shape.size[k]);
  } else {
    const double radial_norm = ::sqrt(pose[8] * pose[8] + pose[9] * pose[9]);
    const double radial = radial_norm > 1e-14 ? -sign * shape.size[0] / radial_norm : 0.0;
    support[0] = radial * pose[8];
    support[1] = radial * pose[9];
    support[2] = opposingExtent(sign * pose[10], shape.size[1]);
  }
  for (int k = 0; k < 2; ++k) {
    const auto* row = pose + 4 * k;
    result.point[k] = row[3] + row[0] * support[0] + row[1] * support[1] + row[2] * support[2];
  }
  result.point[2] = face - sign * 0.5 * depth;
  result.normal[2] = sign;
  result.depth = depth;
  return GroundCollisionStatus::Success;
}

__device__ auto collide(const GroundBox& ground, const GroundCollider& shape,
                        const double* body, Witness& result) -> GroundCollisionStatus {
  double pose[12];
  for (int i = 0; i < 3; ++i) {
    const auto* row = body + 4 * i;
    for (int j = 0; j < 4; ++j) {
      pose[4 * i + j] = row[0] * shape.local_pose[j] +
                        row[1] * shape.local_pose[4 + j] +
                        row[2] * shape.local_pose[8 + j];
      if (j == 3) pose[4 * i + j] += row[3];
      if (!::isfinite(pose[4 * i + j])) return GroundCollisionStatus::InvalidInput;
    }
  }
  if (shape.shape == GroundShape::Sphere) {
    const double center[3]{pose[3], pose[7], pose[11]};
    sphereWitness(ground, center, shape.size[0], result);
    return GroundCollisionStatus::Success;
  }
  if (shape.shape != GroundShape::Box && shape.shape != GroundShape::Cylinder) {
    return GroundCollisionStatus::InvalidInput;
  }
  return horizontalWitness(ground, shape, pose, result);
}

__global__ void groundCollisions(
    std::size_t batch, std::size_t body_count, std::size_t capacity, GroundBox ground,
    const GroundCollider* colliders, std::size_t collider_count, const double* poses,
    ContactHistoryConfig* configs, WorldContact* contacts, GroundCollisionStatus* status) {
  const auto env = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (env >= batch || configs[env].suspended) return;
  const auto* bodies = poses + env * body_count * 12;
  auto* output = contacts + env * capacity;
  std::size_t count = 0;
  auto result = GroundCollisionStatus::Success;
  for (std::size_t i = 0; i < collider_count; ++i) {
    const auto& shape = colliders[i];
    if (shape.body >= body_count || shape.geometry <= ground.geometry) {
      result = GroundCollisionStatus::InvalidInput;
      break;
    }
    Witness witness;
    result = collide(ground, shape, bodies + 12 * shape.body, witness);
    if (result != GroundCollisionStatus::Success) break;
    if (witness.depth == 0.0) continue;
    if (count == capacity) {
      result = GroundCollisionStatus::CapacityExceeded;
      break;
    }
    output[count++] = {ground.geometry, shape.geometry, ground.body, shape.body,
                       {witness.point[0], witness.point[1], witness.point[2]},
                       {witness.normal[0], witness.normal[1], witness.normal[2]},
                       witness.depth, shape.stiffness, shape.damping, shape.friction};
  }
  configs[env].detected = result == GroundCollisionStatus::Success ? count : capacity + 1;
  status[env] = result;
}

}  // namespace

auto launchFlatGroundCollision(
    std::size_t batch, std::size_t body_count, std::size_t capacity, GroundBox ground,
    const GroundCollider* colliders, std::size_t collider_count, const double* body_poses,
    ContactHistoryConfig* configs, WorldContact* contacts, GroundCollisionStatus* status,
    cudaStream_t stream) -> cudaError_t {
  if (batch == 0) return cudaSuccess;
  constexpr auto max_bytes = std::numeric_limits<std::size_t>::max();
  const auto blocks = (batch - 1) / threads_per_block + 1;
  if (!capacity || capacity > max_bytes / sizeof(WorldContact) / batch ||
      !body_count || body_count > max_bytes / (12 * sizeof(double)) / batch ||
      ground.body >= body_count || blocks > std::numeric_limits<int>::max() ||
      (collider_count && !colliders) || !body_poses || !configs || !contacts || !status) {
    return cudaErrorInvalidValue;
  }
  for (int k = 0; k < 3; ++k) {
    if (!std::isfinite(ground.center[k]) || !std::isfinite(ground.half_extent[k]) || ground.half_extent[k] <= 0) {
      return cudaErrorInvalidValue;
    }
  }
  groundCollisions<<<static_cast<unsigned>(blocks), threads_per_block, 0, stream>>>(
      batch, body_count, capacity, ground, colliders, collider_count, body_poses, configs, contacts, status);
  return cudaGetLastError();
}

}  // namespace sire::physics::contact::cuda
