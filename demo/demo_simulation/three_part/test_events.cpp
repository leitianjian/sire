#include "./test_events.hpp"

#include <algorithm>
#include <iterator>
#include <limits>
#include <set>

#include <aris/dynamic/model.hpp>
#include <aris/dynamic/screw.hpp>

#include "sire/core/base_factory.hpp"
#include "sire/core/event_manager.hpp"
#include "sire/physics/common/penetration_as_point_pair.hpp"

namespace sire::three_part {
using namespace sire::physics;
auto InitEvent1::init() -> void {}
auto InitHandler::init(simulator::SimulatorBase* simulator) -> void {
  simulator_ptr = simulator;
}
auto InitHandler::handle(core::EventBase* e) -> bool {
  InitEvent1* event_ptr = dynamic_cast<InitEvent1*>(e);
  // 之后就可以正常积分

  std::unique_ptr<core::EventBase> step_event =
      simulator_ptr->createEventById(1);
  step_event->eventProp().addProp("dt", simulator_ptr->deltaT());
  simulator_ptr->eventManager().addEvent(std::move(step_event));

  return true;
}
auto StepEvent::init() -> void {}
auto StepHandler::init(simulator::SimulatorBase* simulator) -> void {
  simulator_ptr = simulator;
}
auto StepHandler::handle(core::EventBase* e) -> bool {
  // 积分到当前event记录的时间
  double dt = e->eventProp().getPropValue("dt");
  simulator_ptr->integratorPoolPtr()->at(0).step(dt);
  simulator_ptr->timer().updateSimTime(dt);
  // if (dt == 0.0000001) std::cout << "dt=" << dt << " ";

  std::unique_ptr<core::EventBase> step_event =
      simulator_ptr->createEventById(1);
  step_event->eventProp().addProp("dt", simulator_ptr->deltaT());
  // if (!manager_ptr->impactedPrtSet().empty()) {
  //   std::cout
  //       << "shrinked dt "
  //       << manager_ptr->contactPairMap().at({0, 1}).init_penetration_depth_
  //       << std::endl;
  // }
  simulator_ptr->eventManager().addEvent(std::move(step_event));

  return true;
}
ARIS_REGISTRATION {
  core::EventBaseFactory::instance().clear();
  core::HandlerBaseFactory::instance().clear();
  core::EventRegister<InitEvent1>::registration("initial", 0);
  core::EventRegister<StepEvent>::registration("step", 1);
  core::HandlerRegister<InitHandler>::registration("initial", 0);
  core::HandlerRegister<StepHandler>::registration("step", 1);
}
}  // namespace sire::three_part