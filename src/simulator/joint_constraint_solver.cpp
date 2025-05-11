#include "sire/simulator/joint_constraint_solver.hpp"

#include <aris/dynamic/model.hpp>

namespace sire::solver {
class HelpResetRAII {
 public:
  std::vector<bool> prt_active_, jnt_active_, mot_active_, gm_active_,
      fce_active_;
  aris::dynamic::Model* model_;

  HelpResetRAII(aris::dynamic::Model* model) : model_(model) {
    for (auto& prt : model_->partPool()) prt_active_.push_back(prt.active());
    for (auto& jnt : model_->jointPool()) jnt_active_.push_back(jnt.active());
    for (auto& mot : model_->motionPool()) mot_active_.push_back(mot.active());
    for (auto& gm : model_->generalMotionPool())
      gm_active_.push_back(gm.active());
    for (auto& fce : model_->forcePool()) fce_active_.push_back(fce.active());
  }
  ~HelpResetRAII() {
    for (auto& prt : model_->partPool()) prt.activate(prt_active_[prt.id()]);
    for (auto& jnt : model_->jointPool()) jnt.activate(jnt_active_[jnt.id()]);
    for (auto& mot : model_->motionPool()) mot.activate(mot_active_[mot.id()]);
    for (auto& gm : model_->generalMotionPool())
      gm.activate(gm_active_[gm.id()]);
    for (auto& fce : model_->forcePool()) fce.activate(fce_active_[fce.id()]);
  }
};
auto JointConstraintSolver::allocateMemory() -> void {
  HelpResetRAII help_reset(this->model());

  for (auto& m : model()->motionPool()) m.activate(false);
  for (auto& gm : model()->generalMotionPool()) gm.activate(false);

  UniversalSolver::allocateMemory();
}

auto JointConstraintSolver::kinPos() -> int {
  UniversalSolver::kinPos();
  if (error() < maxError())
    for (auto& m : model()->generalMotionPool()) m.updP();
  return error() < maxError() ? 0 : -1;
}
auto JointConstraintSolver::kinVel() -> int {
  UniversalSolver::kinVel();
  for (auto& m : model()->generalMotionPool()) m.updV();
  return 0;
}
auto JointConstraintSolver::dynAccAndFce() -> int {
  UniversalSolver::dynAccAndFce();
  for (auto& m : model()->generalMotionPool()) m.updA();
  return 0;
}
JointConstraintSolver::~JointConstraintSolver() = default;
JointConstraintSolver::JointConstraintSolver(Size max_iter_count,
                                             double max_error)
    : UniversalSolver(max_iter_count, max_error) {}
ARIS_DEFINE_BIG_FOUR_CPP(JointConstraintSolver);

}  // namespace sire::solver