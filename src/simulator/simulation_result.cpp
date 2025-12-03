#include "sire/simulator/simulation_result.hpp"

#include <array>
#include <deque>

#include <aris/dynamic/model.hpp>
#include <aris/dynamic/screw.hpp>

namespace sire::simulator {
struct SimResult::TimeResult::Imp {
  std::deque<double> time_;
};
auto SimResult::TimeResult::record() -> void {
  imp_->time_.push_back(model()->time());
}
auto SimResult::TimeResult::restore(sire::Size pos) -> void {
  model()->setTime(imp_->time_.at(pos));
}
SimResult::TimeResult::~TimeResult() = default;
SimResult::TimeResult::TimeResult(const std::string& name) : imp_(new Imp) {}
SimResult::TimeResult::TimeResult(const SimResult::TimeResult&) = default;
SimResult::TimeResult::TimeResult(SimResult::TimeResult&&) noexcept = default;
SimResult::TimeResult& SimResult::TimeResult::operator=(const TimeResult&) =
    default;
SimResult::TimeResult& SimResult::TimeResult::operator=(TimeResult&&) noexcept =
    default;

struct SimResult::PartResult::Imp {
  aris::dynamic::Part* part_;
  std::deque<std::array<double, sire::kTwistSize>> pe_;
  std::deque<std::array<double, sire::kTwistSize>> vs_;
  std::deque<std::array<double, sire::kTwistSize>> as_;

  Imp(aris::dynamic::Part* part) : part_(part){};
};
auto SimResult::PartResult::part() const -> const aris::dynamic::Part& {
  return *imp_->part_;
}
auto SimResult::PartResult::record() -> void {
  std::array<double, sire::kTwistSize> buf{};
  aris::dynamic::s_pm2pe(*part().pm(), buf.data());
  imp_->pe_.push_back(buf);
  std::copy(static_cast<const double*>(part().vs()),
            static_cast<const double*>(part().vs()) + sire::kTwistSize, buf.data());
  imp_->vs_.push_back(buf);
  std::copy(static_cast<const double*>(part().as()),
            static_cast<const double*>(part().as()) + sire::kTwistSize, buf.data());
  imp_->as_.push_back(buf);
}
auto SimResult::PartResult::restore(sire::Size pos) -> void {
  part().setPe(imp_->pe_.at(pos).data());
  part().setVs(imp_->vs_.at(pos).data());
  part().setAs(imp_->as_.at(pos).data());
}
SimResult::PartResult::~PartResult() = default;
SimResult::PartResult::PartResult(const std::string& name,
                                  aris::dynamic::Part* part)
    : imp_(new Imp(part)) {}
SimResult::PartResult::PartResult(const SimResult::PartResult&) = default;
SimResult::PartResult::PartResult(SimResult::PartResult&&) noexcept = default;
SimResult::PartResult& SimResult::PartResult::operator=(const PartResult&) =
    default;
SimResult::PartResult& SimResult::PartResult::operator=(PartResult&&) noexcept =
    default;

struct SimResult::ConstraintResult::Imp {
  aris::dynamic::Constraint* constraint_;
  std::deque<std::array<double, sire::kTwistSize>> cf_;

  Imp(aris::dynamic::Constraint* constraint) : constraint_(constraint){};
};
auto SimResult::ConstraintResult::constraint() const
    -> const aris::dynamic::Constraint& {
  return *imp_->constraint_;
}
auto SimResult::ConstraintResult::record() -> void {
  std::array<double, sire::kTwistSize> result{};
  std::copy(constraint().cf(), constraint().cf() + constraint().dim(),
            result.data());
  imp_->cf_.push_back(result);
}
auto SimResult::ConstraintResult::restore(sire::Size pos) -> void {
  constraint().setCf(imp_->cf_.at(pos).data());
  if (dynamic_cast<aris::dynamic::Motion*>(&constraint())) {
    dynamic_cast<aris::dynamic::Motion*>(&constraint())->updP();
    dynamic_cast<aris::dynamic::Motion*>(&constraint())->updV();
    dynamic_cast<aris::dynamic::Motion*>(&constraint())->updA();
  }
  if (dynamic_cast<aris::dynamic::GeneralMotion*>(&constraint())) {
    dynamic_cast<aris::dynamic::GeneralMotion*>(&constraint())->updP();
    dynamic_cast<aris::dynamic::GeneralMotion*>(&constraint())->updV();
    dynamic_cast<aris::dynamic::GeneralMotion*>(&constraint())->updA();
  }
}
SimResult::ConstraintResult::~ConstraintResult() = default;
SimResult::ConstraintResult::ConstraintResult(
    const std::string& name, aris::dynamic::Constraint* constraint)
    : imp_(new Imp(constraint)) {}
SimResult::ConstraintResult::ConstraintResult(
    const SimResult::ConstraintResult&) = default;
SimResult::ConstraintResult::ConstraintResult(
    SimResult::ConstraintResult&&) noexcept = default;
SimResult::ConstraintResult& SimResult::ConstraintResult::operator=(
    const ConstraintResult&) = default;
SimResult::ConstraintResult& SimResult::ConstraintResult::operator=(
    ConstraintResult&&) noexcept = default;

struct SimResult::ForceResult::Imp {
  sire::physics::Force* force_;
  std::deque<std::array<double, sire::kTwistSize>> fces_;

  Imp(sire::physics::Force* force) : force_(force){};
};
auto SimResult::ForceResult::force() const -> const sire::physics::Force& {
  return *imp_->force_;
}
auto SimResult::ForceResult::record() -> void {
  std::array<double, sire::kTwistSize> result{};
  std::copy(force().fs(), force().fs() + sire::kTwistSize, result.data());
  imp_->fces_.push_back(result);
}
auto SimResult::ForceResult::restore(sire::Size pos) -> void {
  force().setFs(imp_->fces_.at(pos).data());
}
SimResult::ForceResult::~ForceResult() = default;
// SimResult::ForceResult::ForceResult(const std::string& name,
//                                     sire::physics::Force* force)
//     : imp_(new Imp(force)) {}
// SimResult::ForceResult::ForceResult(const SimResult::ForceResult&) = default;
// SimResult::ForceResult::ForceResult(SimResult::ForceResult&&) noexcept =
//     default;
// SimResult::ForceResult& SimResult::ForceResult::operator=(const ForceResult&) =
//     default;
// SimResult::ForceResult& SimResult::ForceResult::operator=(
//     ForceResult&&) noexcept = default;

struct SimResult::Imp {
  TimeResult time_result_;
  std::vector<PartResult> part_result_pool_;
  std::vector<ConstraintResult> constraint_result_pool_;
  // std::vector<ForceResult> force_result_pool_;
};
auto SimResult::timeResult() const -> const TimeResult& {
  return imp_->time_result_;
}
// auto SimResult::partResultPool() const
//     -> const std::vector<SimResult::PartResult>& {
//   return imp_->part_result_pool_;
// }
// auto SimResult::constraintResultPool() const
//     -> const std::vector<SimResult::ConstraintResult>& {
//   return imp_->constraint_result_pool_;
// }
// auto SimResult::forceResultPool() const
//     -> const std::vector<SimResult::ForceResult>& {
//   return imp_->force_result_pool_;
// }
auto SimResult::allocateMemory() -> void {
  imp_->part_result_pool_.clear();
  for (auto& p : model()->partPool())
    imp_->part_result_pool_.emplace_back(p.name() + "_result", &p);
  imp_->constraint_result_pool_.clear();
  for (auto& j : model()->jointPool())
    imp_->constraint_result_pool_.emplace_back(j.name() + "_result", &j);
  for (auto& m : model()->motionPool())
    imp_->constraint_result_pool_.emplace_back(m.name() + "_result", &m);
  for (auto& g : model()->generalMotionPool())
    imp_->constraint_result_pool_.emplace_back(g.name() + "_result", &g);
  // imp_->force_result_pool_.clear();
  // for (auto& f : model()->forcePool())
  //   imp_->force_result_pool_.emplace_back(f.name() + "_result", &f);

  timeResult().resetModel(model());
  for (auto& p : imp_->part_result_pool_) p.resetModel(model());
  for (auto& c : imp_->constraint_result_pool_) c.resetModel(model());
  // for (auto& f : imp_->force_result_pool_) f.resetModel(model());
}
auto SimResult::record() -> void {
  timeResult().record();
  for (auto& p : imp_->part_result_pool_) p.record();
  for (auto& c : imp_->constraint_result_pool_) c.record();
  // for (auto& f : imp_->force_result_pool_) f.record();
}
auto SimResult::restore(sire::Size pos) -> void {
  timeResult().restore(pos);
  for (auto& p : imp_->part_result_pool_) p.restore(pos);
  for (auto& r : imp_->constraint_result_pool_) r.restore(pos);
  // for (auto& f : imp_->force_result_pool_) f.restore(pos);
}
auto SimResult::size() const -> Size {
  return timeResult().imp_->time_.size() == 0
             ? 0
             : timeResult().imp_->time_.size() - 1;
}
auto SimResult::clear() -> void {
  timeResult().imp_->time_.clear();
  for (auto& p : imp_->part_result_pool_) {
    p.imp_->pe_.clear();
    p.imp_->vs_.clear();
    p.imp_->as_.clear();
  }
  for (auto& c : imp_->constraint_result_pool_) c.imp_->cf_.clear();
  // for (auto& f : imp_->force_result_pool_) f.imp_->fces_.clear();
}
SimResult::~SimResult() = default;
SimResult::SimResult(const std::string& name) : imp_(new Imp()) {}
SimResult::SimResult(const SimResult& other)
    : Element(other), imp_(other.imp_) {}
SimResult::SimResult(SimResult&& other) noexcept
    : Element(std::move(other)), imp_(std::move(other.imp_)) {}
SimResult& SimResult::operator=(const SimResult& other) = default;
SimResult& SimResult::operator=(SimResult&& other) noexcept = default;
}  // namespace sire::simulator