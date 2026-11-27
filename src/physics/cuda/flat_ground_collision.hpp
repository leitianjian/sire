#pragma once

#include "contact_history.hpp"

namespace sire::physics::contact::cuda {

struct GroundBox {
  std::uint64_t geometry;
  std::size_t body;
  double center[3], half_extent[3];
  double negative_support_scale{1.0 + 1e-10};
};

enum class GroundShape { Sphere, Box, Cylinder };

struct GroundCollider {
  std::uint64_t geometry;
  std::size_t body;
  GroundShape shape;
  bool variable_friction{false};
  double local_pose[12];  // Row-major body-to-shape pose, first three rows.
  double size[3];         // Sphere radius, box half extents, or cylinder radius and half length.
  double stiffness, damping, friction;
};

enum class GroundCollisionStatus { Success, InvalidInput, CapacityExceeded, NumericalFailure };

/// Bytes of device scratch for the concurrent GJK/EPA queries in a batch.
auto groundCollisionWorkspaceSize(std::size_t batch) -> std::size_t;

/// Generate one witness per enabled ground pair. The ground and collider
/// table are shared by all environments. Body poses are [batch, body_count,
/// 12] row-major world poses (first three rows). Contacts are [batch, capacity]
/// in collider-table order. Counts in configs and status are [batch]. A failed
/// environment publishes a count above capacity to invalidate downstream
/// preparation. Inputs and outputs reside on the launch stream's device.
///
/// Box-sphere uses COAL's analytic query. Box-box and box-cylinder use its
/// GJK/EPA query, including contacts on the sides and edges of the ground box.
auto launchFlatGroundCollision(
    std::size_t batch, std::size_t body_count, std::size_t capacity, GroundBox ground,
    const GroundCollider* colliders, std::size_t collider_count, const double* body_poses,
    ContactHistoryConfig* configs, WorldContact* contacts, GroundCollisionStatus* status,
    unsigned char* workspace, cudaStream_t stream,
    const double* friction_override = nullptr) -> cudaError_t;

}  // namespace sire::physics::contact::cuda
