#pragma once

#include <cstddef>
#include <vector>

namespace aris::dynamic {
class Model;
}

namespace sire::physics::contact::cuda {

struct RigidBody {
  // Root and ground have themselves as parent.
  std::size_t parent;
  // Motion pool index, or -1 for fixed joints, root, and ground.
  int joint;
  // Row-major 3-by-4 transforms from each joint marker to its part frame.
  double parent_marker[12];
  double child_marker[12];
  // ARIS inertia vector expressed in the part frame.
  double inertia[10];
};

struct RigidBodyModel {
  std::size_t root;
  std::size_t ground;
  std::vector<RigidBody> bodies;
  // Moving part IDs in parent-before-child order, starting with the root.
  std::vector<std::size_t> order;
  std::size_t joint_count;
  double gravity[6];
};

auto loadRigidBodyModel(aris::dynamic::Model& model) -> RigidBodyModel;

}  // namespace sire::physics::contact::cuda
