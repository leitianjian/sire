#include "sire/physics/contact/contact_solver.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_set>

#include <aris/core/reflection.hpp>

#include "sire/physics/collision/collision_detection.hpp"
#include "sire/physics/collision/collision_exists_callback.hpp"
#include "sire/physics/physics_engine.hpp"
#include "sire/physics/utils.hpp"
#include "sire/simulator/simulation_loop.hpp"

namespace sire::physics::contact {
ARIS_DEFINE_BIG_FOUR_CPP(ContactSolver);

auto ContactSolver::init(physics::PhysicsEngine* engine_ptr) -> void {
  engine_ptr_ = engine_ptr;
  if (singlePointContactMode() && engine_ptr_ && engine_ptr_->simLoopPtr()) {
    engine_ptr_->simLoopPtr()->contactPairManager()->clear();
  }
  doInit(engine_ptr);
}

auto ContactSolver::setContactModelMode(const std::string& mode) -> void {
  if (mode != "height_field" && mode != "single_point") {
    throw std::invalid_argument(
        "contact_model_mode must be height_field or single_point");
  }
  if (mode == "single_point" && !supportsSinglePointContactMode()) {
    throw std::invalid_argument(
        "single_point is supported by PsVsSolver3 and V5-derived solvers");
  }
  if (mode == contact_model_mode_) return;
  contact_model_mode_ = mode;
  if (engine_ptr_ && engine_ptr_->simLoopPtr()) {
    engine_ptr_->simLoopPtr()->contactPairManager()->clear();
  }
}

auto ContactSolver::setContactTimeMethod(const std::string& method) -> void {
  if (method != "exponential" && method != "polynomial")
    throw std::invalid_argument("contact_time_method must be exponential or polynomial");
  contact_time_method_ = method;
}

auto ContactSolver::prepareSinglePointContacts(
    std::vector<common::PenetrationAsPointPair>& pairs,
    std::vector<std::array<double, 16>>& frames,
    std::vector<sire::Size>& preserved) -> void {
  // A newly detected contact may already be noticeably inside the surface
  // when the outer simulation step is large. Previously the whole detected
  // depth became the baseline, so the contact model started at exactly zero
  // compression and did not try to recover any of that penetration. Retain a
  // small, bounded amount for the contact dynamics to resolve. Shallower new
  // contacts keep their complete physical depth.
  constexpr double kNewContactRetainedDepth = 1e-4;
  // These IDs identify geometries, even though ContactPairManager uses the
  // PartId alias. Several geometries on one rigid body remain separate pairs.
  using Key = core::SortedPair<sire::PartId>;
  std::unordered_set<Key> detected;
  for (const auto& pair : pairs) {
    if (!detected.emplace(pair.id_A, pair.id_B).second) {
      throw std::invalid_argument(
          "single_point requires one contact point per geometry pair; pair (" +
          std::to_string(pair.id_A) + ", " + std::to_string(pair.id_B) +
          ") has multiple points. Use height_field mode for this scene.");
    }
  }

  auto& records =
      engine_ptr_->simLoopPtr()->contactPairManager()->contactPairMap();
  for (auto it = records.begin(); it != records.end();) {
    if (detected.count(it->first) == 0) {
      it = records.erase(it);
    } else {
      ++it;
    }
  }
  for (auto& pair : pairs) {
    const double newContactBaseline =
        std::max(0.0, pair.depth - kNewContactRetainedDepth);
    auto entry = records.emplace(
        Key(pair.id_A, pair.id_B),
        core::ContactPairValue(newContactBaseline, false));
    auto& initialDepth = entry.first->second.init_penetration_depth_;
    pair.modifiedDepth = pair.depth - initialDepth;
    // Preserve set6 semantics: a negative corrected depth is filtered this
    // step, while lowering the baseline for the next collision detection.
    if (pair.modifiedDepth < 0) initialDepth = pair.depth;
  }

  cptContactFrame(pairs, frames);
  preserved.clear();
  for (sire::Size i = 0; i < pairs.size(); ++i) {
    if (pairs[i].modifiedDepth < 0) continue;
    if (std::abs(pairs[i].modifiedDepth) < 1e-8) {
      std::array<double, 3> velocity;
      engine_ptr_->cptContactVelocityB2A(pairs[i], frames[i], velocity);
      if (velocity[2] >= 0) continue;
    }
    preserved.push_back(i);
  }
}
ARIS_REGISTRATION {
  typedef sire::physics::collision::CollisionFilter& (
      ContactSolver::*CollisionFilterPoolFunc)();
  aris::core::class_<ContactSolver>("ContactSolver")
      .prop("contact_model_mode", &ContactSolver::setContactModelMode,
            &ContactSolver::contactModelMode)
      .prop("contact_time_method", &ContactSolver::setContactTimeMethod,
            &ContactSolver::contactTimeMethod);
}
}  // namespace sire::physics::contact
