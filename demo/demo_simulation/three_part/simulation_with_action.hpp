#ifndef SIRE_SIMULATION_WITH_ACTION_COMMAND_H_
#define SIRE_SIMULATION_WITH_ACTION_COMMAND_H_

#include <aris.hpp>

namespace sire::plan {
class SimulationWithAction
    : public aris::core::CloneObject<SimulationWithAction, aris::plan::Plan> {
 public:
  auto virtual prepareNrt() -> void override;
  virtual ~SimulationWithAction() = default;
  explicit SimulationWithAction(
      const std::string& name = "SimulationWithAction_plan");
  ARIS_DECLARE_BIG_FOUR(SimulationWithAction);

 private:
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;
};
}  // namespace sire::plan
#endif