#pragma once

#include "contact_history.hpp"

namespace sire::physics::contact::cuda {

struct GroundBox {
  std::uint64_t geometry;
  std::size_t body;
  double center[3], half_extent[3];
};

enum class GroundShape { Sphere, Box, Cylinder };

struct GroundCollider {
  std::uint64_t geometry;
  std::size_t body;
  GroundShape shape;
  double local_pose[12];  // Row-major body-to-shape pose, first three rows.
  double size[3];         // Sphere radius, box half extents, or cylinder radius and half length.
  double stiffness, damping, friction;
};

enum class GroundCollisionStatus { Success, InvalidInput, CapacityExceeded, UnsupportedBoundary };

/// Generate one witness per enabled ground pair. The ground and collider
/// table are shared by all environments. Body poses are [batch, body_count,
/// 12] row-major world poses (first three rows). Contacts are [batch, capacity]
/// in collider-table order. Counts in configs and status are [batch]. A failed
/// environment publishes a count above capacity to invalidate downstream
/// preparation. Inputs and outputs reside on the launch stream's device.
///
/// Spheres support the entire finite box. Boxes and cylinders use a horizontal
/// face where the separation can be resolved away from lateral boundaries.
/// Possible intersections near a lateral boundary are reported explicitly.
auto launchFlatGroundCollision(
    std::size_t batch, std::size_t body_count, std::size_t capacity, GroundBox ground,
    const GroundCollider* colliders, std::size_t collider_count, const double* body_poses,
    ContactHistoryConfig* configs, WorldContact* contacts, GroundCollisionStatus* status,
    cudaStream_t stream) -> cudaError_t;

}  // namespace sire::physics::contact::cuda
