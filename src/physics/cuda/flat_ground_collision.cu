#include "flat_ground_collision.hpp"

#include "coal_epa.cuh"
#include "coal_shapes.cuh"

#include <cmath>
#include <limits>
#include <new>

namespace sire::physics::contact::cuda {
namespace {

using coal_port::Mat3;
using coal_port::Vec3;

// Each block is one warp, divided into independent environment subgroups.
constexpr unsigned threads_per_block = 32;
constexpr unsigned lanes_per_env = 8;
constexpr unsigned envs_per_block = threads_per_block / lanes_per_env;
constexpr double zero_depth = std::numeric_limits<double>::epsilon();
constexpr double collision_threshold = 1e-12;

struct Witness {
  Vec3 point, normal;
  double depth{0.0};
};

__device__ auto convexDistance(const coal_port::MinkowskiDifference& shape,
                               const Vec3& ground_center, coal_port::EpaWorkspace& workspace,
                               double& distance, Vec3& point_box, Vec3& point_shape,
                               Vec3& normal) -> bool {
  const Vec3 guess(1.0, 0.0, 0.0);  // COAL CollisionRequest's default guess.
  coal_port::GJK gjk;
  gjk.evaluate(shape, guess);
  if (gjk.status != coal_port::GJK::Collision) {
    // Without swept spheres, separated queries have a distance at least the
    // GJK tolerance, and on iteration exhaustion COAL leaves distance at zero.
    // Sire emits neither result, so we do not need their witnesses.
    distance = gjk.distance;
    return gjk.status != coal_port::GJK::DidNotRun;
  }

  auto* storage = new (&workspace) coal_port::EpaWorkspace;
  coal_port::EpaSolver epa(*storage);
  const auto contact = epa.evaluate(gjk, -guess);
  if (contact.status == coal_port::EpaStatus::FallBack) return false;
  distance = ::fmin(0.0, -contact.depth);
  normal = contact.normal;
  // COAL projects the two witnesses along the solved normal around their
  // midpoint before constructing the Contact object.
  const Vec3 midpoint = 0.5 * (contact.w0 + contact.w1) + ground_center;
  point_box = midpoint - 0.5 * distance * normal;
  point_shape = midpoint + 0.5 * distance * normal;
  return true;
}

__device__ auto boundsOverlap(const GroundBox& ground, const GroundCollider& shape,
                              const Mat3& rotation, const Vec3& center, bool identity) -> bool {
  // COAL's CollisionObject computes the world AABB from the local AABB.
  // A cylinder's local bounds use its radius on both radial axes.
  const Vec3 local_extent = shape.shape == GroundShape::Box
      ? Vec3(shape.size) : shape.shape == GroundShape::Sphere
      ? Vec3::Constant(shape.size[0]).eval()
      : Vec3(shape.size[0], shape.size[0], shape.size[1]);
  const Vec3 extent = identity ? local_extent
      : (rotation.cwiseAbs() * local_extent).eval();
  for (int axis = 0; axis < 3; ++axis) {
    if (center[axis] - extent[axis] > ground.center[axis] + ground.half_extent[axis] ||
        center[axis] + extent[axis] < ground.center[axis] - ground.half_extent[axis]) {
      return false;
    }
  }
  return true;
}

__device__ auto collide(const GroundBox& ground, const GroundCollider& shape,
                        const double* body, coal_port::EpaWorkspace& epa_workspace,
                        Witness& result) -> GroundCollisionStatus {
  Mat3 rotation;
  Vec3 center;
  bool identity = true;
  for (int i = 0; i < 3; ++i) {
    const auto* row = body + 4 * i;
    for (int j = 0; j < 3; ++j) {
      rotation(i, j) = row[0] * shape.local_pose[j] + row[1] * shape.local_pose[4 + j] +
                       row[2] * shape.local_pose[8 + j];
      if (!::isfinite(rotation(i, j))) return GroundCollisionStatus::InvalidInput;
      identity = identity && ::fabs(rotation(i, j) - (i == j ? 1.0 : 0.0)) <= 1e-12;
    }
    center[i] = row[0] * shape.local_pose[3] + row[1] * shape.local_pose[7] +
                row[2] * shape.local_pose[11] + row[3];
    if (!::isfinite(center[i])) return GroundCollisionStatus::InvalidInput;
  }
  if (!boundsOverlap(ground, shape, rotation, center, identity)) return GroundCollisionStatus::Success;

  const Vec3 ground_center(ground.center);
  const Vec3 half_extent(ground.half_extent);
  Vec3 point_box, point_shape;
  double distance;
  if (shape.shape == GroundShape::Sphere) {
    distance = coal_port::boxSphereDistance(half_extent, ground_center, center, shape.size[0],
                                           point_box, point_shape, result.normal);
  } else if (shape.shape == GroundShape::Box || shape.shape == GroundShape::Cylinder) {
    const Vec3 translation = center - ground_center;
    const coal_port::MinkowskiDifference difference{
        half_extent, shape.shape, Vec3(shape.size), rotation, translation,
        identity && translation.isZero(), ground.negative_support_scale};
    if (!convexDistance(difference, ground_center, epa_workspace, distance,
                        point_box, point_shape, result.normal)) {
      return GroundCollisionStatus::NumericalFailure;
    }
  } else {
    return GroundCollisionStatus::InvalidInput;
  }
  if (!::isfinite(distance)) return GroundCollisionStatus::NumericalFailure;
  if (distance <= collision_threshold && ::fabs(distance) > zero_depth) {
    result.depth = ::fabs(distance);
    result.point = (point_box + point_shape) / 2.0;
    for (int axis = 0; axis < 3; ++axis) {
      if (!::isfinite(result.point[axis]) || !::isfinite(result.normal[axis])) {
        return GroundCollisionStatus::NumericalFailure;
      }
    }
  }
  return GroundCollisionStatus::Success;
}

template <bool VaryFriction>
__global__ void groundCollisions(
    std::size_t batch, std::size_t body_count, std::size_t capacity, GroundBox ground,
    const GroundCollider* colliders, std::size_t collider_count, const double* poses,
    ContactHistoryConfig* configs, WorldContact* contacts, GroundCollisionStatus* status,
    coal_port::EpaWorkspace* workspace, const double* friction_override) {
  const unsigned group_base = threadIdx.x / lanes_per_env * lanes_per_env;
  const unsigned lane = threadIdx.x % lanes_per_env;
  const unsigned group_mask = ((1u << lanes_per_env) - 1u) << group_base;
  const auto env = static_cast<std::size_t>(blockIdx.x) * envs_per_block + threadIdx.x / lanes_per_env;
  if (env >= batch || configs[env].suspended) return;
  const auto* bodies = poses + env * body_count * 12;
  auto* output = contacts + env * capacity;
  const double override_value = VaryFriction ? friction_override[env] : 0.0;
  if constexpr (VaryFriction) {
    if (!::isfinite(override_value) || override_value < 0.0) {
      if (lane == 0) {
        configs[env].detected = capacity + 1;
        status[env] = GroundCollisionStatus::InvalidInput;
      }
      return;
    }
  }
  std::size_t count = 0;
  auto result = GroundCollisionStatus::Success;
  auto& epa_workspace = workspace[env * lanes_per_env + lane];
  for (std::size_t base = 0; base < collider_count;) {
    const auto remaining = collider_count - base;
    const auto* shape = lane < remaining ? colliders + base + lane : nullptr;
    Witness witness;
    auto pair_status = GroundCollisionStatus::Success;
    if (shape) {
      if (shape->body >= body_count || shape->geometry <= ground.geometry) {
        pair_status = GroundCollisionStatus::InvalidInput;
      } else {
        pair_status = collide(ground, *shape, bodies + 12 * shape->body, epa_workspace, witness);
      }
    }

    const auto errors = __ballot_sync(group_mask, pair_status != GroundCollisionStatus::Success);
    const auto hits = __ballot_sync(group_mask, pair_status == GroundCollisionStatus::Success &&
                                               witness.depth != 0.0);
    const unsigned error_mask = (errors & group_mask) >> group_base;
    unsigned contact_mask = (hits & group_mask) >> group_base;
    const unsigned first_error = error_mask ? __ffs(error_mask) - 1 : lanes_per_env;
    // Keep the serial order both for contact slots and for the first failure.
    contact_mask &= (1u << first_error) - 1u;
    const auto available = capacity - count;
    if (contact_mask & (1u << lane)) {
      const auto rank = static_cast<unsigned>(__popc(contact_mask & ((1u << lane) - 1u)));
      if (rank < available) {
        const double friction = VaryFriction && shape->variable_friction ? override_value : shape->friction;
        output[count + rank] = {ground.geometry, shape->geometry, ground.body, shape->body,
                                {witness.point[0], witness.point[1], witness.point[2]},
                                {witness.normal[0], witness.normal[1], witness.normal[2]},
                                witness.depth, shape->stiffness, shape->damping, friction};
      }
    }

    const auto chunk_count = static_cast<unsigned>(__popc(contact_mask));
    if (chunk_count > available) {
      result = GroundCollisionStatus::CapacityExceeded;
      break;
    }
    if (error_mask) {
      result = static_cast<GroundCollisionStatus>(
          __shfl_sync(group_mask, static_cast<int>(pair_status), first_error, lanes_per_env));
      break;
    }
    count += chunk_count;
    if (remaining <= lanes_per_env) break;
    base += lanes_per_env;
  }
  if (lane == 0) {
    configs[env].detected = result == GroundCollisionStatus::Success ? count : capacity + 1;
    status[env] = result;
  }
}

}  // namespace

auto groundCollisionWorkspaceSize(std::size_t batch) -> std::size_t {
  constexpr auto stride = lanes_per_env * sizeof(coal_port::EpaWorkspace);
  return batch <= std::numeric_limits<std::size_t>::max() / stride ? batch * stride : 0;
}

auto launchFlatGroundCollision(
    std::size_t batch, std::size_t body_count, std::size_t capacity, GroundBox ground,
    const GroundCollider* colliders, std::size_t collider_count, const double* body_poses,
    ContactHistoryConfig* configs, WorldContact* contacts, GroundCollisionStatus* status,
    unsigned char* workspace, cudaStream_t stream, const double* friction_override) -> cudaError_t {
  if (batch == 0) return cudaSuccess;
  constexpr auto max_bytes = std::numeric_limits<std::size_t>::max();
  const auto blocks = (batch - 1) / envs_per_block + 1;
  if (!capacity || capacity > max_bytes / sizeof(WorldContact) / batch ||
      !body_count || body_count > max_bytes / (12 * sizeof(double)) / batch ||
      ground.body >= body_count || blocks > std::numeric_limits<int>::max() ||
      (collider_count && !colliders) || !body_poses || !configs || !contacts || !status ||
      !workspace || !groundCollisionWorkspaceSize(batch)) {
    return cudaErrorInvalidValue;
  }
  for (int k = 0; k < 3; ++k) {
    if (!std::isfinite(ground.center[k]) || !std::isfinite(ground.half_extent[k]) || ground.half_extent[k] <= 0) {
      return cudaErrorInvalidValue;
    }
  }
  auto* scratch = reinterpret_cast<coal_port::EpaWorkspace*>(workspace);
  if (friction_override) {
    groundCollisions<true><<<static_cast<unsigned>(blocks), threads_per_block, 0, stream>>>(
        batch, body_count, capacity, ground, colliders, collider_count, body_poses, configs, contacts,
        status, scratch, friction_override);
  } else {
    groundCollisions<false><<<static_cast<unsigned>(blocks), threads_per_block, 0, stream>>>(
        batch, body_count, capacity, ground, colliders, collider_count, body_poses, configs, contacts,
        status, scratch, nullptr);
  }
  return cudaGetLastError();
}

}  // namespace sire::physics::contact::cuda
