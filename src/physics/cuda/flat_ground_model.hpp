#pragma once

#include <vector>

#include "flat_ground_collision.hpp"

namespace sire::physics {
class PhysicsEngine;

namespace contact::cuda {

struct FlatGroundModel {
  GroundBox ground;
  std::size_t body_count;
  std::vector<GroundCollider> colliders;
};

/// Copy static collision metadata from an initialized Sire model. The
/// supported scene has one horizontal anchored box, with enabled dynamic
/// shape pairs only against that box.
auto loadFlatGroundModel(PhysicsEngine& engine) -> FlatGroundModel;

}  // namespace contact::cuda
}  // namespace sire::physics
