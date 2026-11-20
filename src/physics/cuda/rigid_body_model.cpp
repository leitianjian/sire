#include "rigid_body_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>

#include <aris/dynamic/model.hpp>

#include "sire/core/sire_fixed_joint.hpp"

namespace sire::physics::contact::cuda {
namespace {

template <std::size_t N>
auto copyFinite(const double* source, double (&destination)[N], const char* error) -> void {
  if (!std::all_of(source, source + N, [](double value) { return std::isfinite(value); })) {
    throw std::invalid_argument(error);
  }
  std::copy_n(source, N, destination);
}

}  // namespace

auto loadRigidBodyModel(aris::dynamic::Model& model) -> RigidBodyModel {
  const auto& parts = model.partPool();
  const auto body_count = parts.size();
  if (body_count < 2 || parts[0].model() != &model) {
    throw std::invalid_argument("CUDA rigid-body dynamics requires an initialized floating model");
  }
  if (std::any_of(model.generalMotionPool().begin(), model.generalMotionPool().end(),
                  [](const auto& motion) { return motion.active(); })) {
    throw std::invalid_argument("CUDA rigid-body dynamics does not support active general motions");
  }

  RigidBodyModel result{};
  result.ground = model.ground().id();
  if (result.ground >= body_count || &parts[result.ground] != &model.ground()) {
    throw std::invalid_argument("Invalid CUDA rigid-body ground part");
  }
  result.bodies.resize(body_count);
  copyFinite(model.environment().gravity(), result.gravity, "Invalid CUDA rigid-body gravity");
  for (std::size_t i = 0; i < body_count; ++i) {
    const auto& part = parts[i];
    if (part.id() != i || part.model() != &model || (i != result.ground && !part.active())) {
      throw std::invalid_argument("CUDA rigid-body dynamics requires initialized active parts");
    }
    auto& body = result.bodies[i];
    body.parent = i;
    body.joint = -1;
    copyFinite(part.prtIv(), body.inertia, "Invalid CUDA rigid-body inertia");
  }

  const auto& motions = model.motionPool();
  result.joint_count = motions.size();
  if (motions.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    throw std::invalid_argument("Too many CUDA rigid-body motions");
  }
  std::unordered_map<const aris::dynamic::Marker*, std::size_t> motion_by_child_marker;
  for (std::size_t i = 0; i < motions.size(); ++i) {
    const auto& motion = motions[i];
    if (!motion.active() || motion.axis() != 5 || motion.pitch() != 0 ||
        motion.mpOffset() != 0 || motion.mpFactor() != 1 || motion.rotateRange() != 0 ||
        !motion.makI() || !motion.makJ() ||
        !motion_by_child_marker.emplace(motion.makI(), i).second) {
      throw std::invalid_argument(
          "CUDA rigid-body dynamics requires distinct active rotational Z motions in radians");
    }
  }

  std::vector<std::vector<std::size_t>> children(body_count);
  for (const auto& joint : model.jointPool()) {
    if (!joint.active()) continue;
    const bool revolute = dynamic_cast<const aris::dynamic::RevoluteJoint*>(&joint);
    if (!revolute && !dynamic_cast<const core::FixedJoint*>(&joint)) {
      throw std::invalid_argument("CUDA rigid-body dynamics supports revolute and fixed joints");
    }
    // ARIS measures the angle of marker I in marker J. In the supported tree,
    // I belongs to the child and J belongs to the parent.
    const auto* child_marker = joint.makI();
    const auto* parent_marker = joint.makJ();
    if (!child_marker || !parent_marker || child_marker->model() != &model ||
        parent_marker->model() != &model) {
      throw std::invalid_argument("Invalid CUDA rigid-body joint markers");
    }
    const auto child = child_marker->fatherPart().id();
    const auto parent = parent_marker->fatherPart().id();
    if (child >= body_count || parent >= body_count || child == parent ||
        child == result.ground || parent == result.ground ||
        &parts[child] != &child_marker->fatherPart() || &parts[parent] != &parent_marker->fatherPart() ||
        result.bodies[child].parent != child) {
      throw std::invalid_argument(
          "CUDA rigid-body dynamics requires a floating tree with child markers at joint I");
    }

    auto& body = result.bodies[child];
    body.parent = parent;
    copyFinite(*parent_marker->prtPm(), body.parent_marker, "Invalid CUDA parent marker pose");
    copyFinite(*child_marker->prtPm(), body.child_marker, "Invalid CUDA child marker pose");
    children[parent].push_back(child);

    if (revolute) {
      const auto motion = motion_by_child_marker.find(child_marker);
      if (motion == motion_by_child_marker.end() || motions[motion->second].makJ() != parent_marker) {
        throw std::invalid_argument("Every CUDA revolute joint requires a motion on its exact markers");
      }
      body.joint = static_cast<int>(motion->second);
      motion_by_child_marker.erase(motion);
    }
  }
  if (!motion_by_child_marker.empty()) {
    throw std::invalid_argument("Every CUDA rigid-body motion must drive a revolute joint");
  }

  for (std::size_t i = 0; i < body_count; ++i) {
    if (i == result.ground || result.bodies[i].parent != i) continue;
    if (!result.order.empty()) {
      throw std::invalid_argument("CUDA rigid-body dynamics requires a single floating root");
    }
    result.root = i;
    result.order.push_back(i);
  }
  for (std::size_t i = 0; i < result.order.size(); ++i) {
    const auto& links = children[result.order[i]];
    result.order.insert(result.order.end(), links.begin(), links.end());
  }
  if (result.order.size() != body_count - 1) {
    throw std::invalid_argument("CUDA rigid-body dynamics requires an acyclic connected tree");
  }
  return result;
}

}  // namespace sire::physics::contact::cuda
