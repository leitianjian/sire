#ifndef SIRE_INIT_EVENT_HPP_
#define SIRE_INIT_EVENT_HPP_
#include <sire_lib_export.h>

#include "sire/core/event_base.hpp"
#include "sire/core/handler_base.hpp"
#include "sire/core/trigger_base.hpp"
#include "sire/simulator/event_manager.hpp"
#include "sire/simulator/simulation_loop.hpp"
namespace sire::simulator {
const core::EventId kInitEventId = 0;
const core::EventId kStepEventId = 1;
const core::EventId kCtrlEventId = 2;
class SIRE_API InitTrigger final : public core::TriggerBase {
 public:
  InitTrigger() : TriggerBase() {}
  ~InitTrigger() = default;
  auto trigger(simulator::SimulationLoop*) -> void override;
};
class SIRE_API InitEvent final : public core::EventBase {
 public:
  InitEvent() : EventBase() {}
  ~InitEvent() = default;
  auto init() -> void override;
  simulator::SimulationLoop* simulator_ptr;
  physics::PhysicsEngine* engine_ptr;
  simulator::EventManager* manager_ptr;
};
class SIRE_API InitHandler final : public core::HandlerBase {
 public:
  InitHandler() : HandlerBase() {}
  ~InitHandler() = default;
  auto init(simulator::SimulationLoop* simulator) -> void override;
  auto handle(core::EventBase* e) -> bool override;
  simulator::SimulationLoop* simulator_ptr;
};
class SIRE_API StepTrigger final : public core::TriggerBase {};
class SIRE_API StepEvent final : public core::EventBase {
 public:
  StepEvent() : EventBase() {}
  ~StepEvent() = default;
  auto init() -> void override;
  simulator::SimulationLoop* simulator_ptr;
  physics::PhysicsEngine* engine_ptr;
  simulator::EventManager* manager_ptr;
};
class SIRE_API StepHandler final : public core::HandlerBase {
 public:
  StepHandler() : HandlerBase() {}
  ~StepHandler() = default;
  auto init(simulator::SimulationLoop* simulator) -> void override;
  auto handle(core::EventBase* e) -> bool override;
  simulator::SimulationLoop* simulator_ptr;
};

class SIRE_API InitEvent1 final : public core::EventBase {
 public:
  InitEvent1() : EventBase() {}
  ~InitEvent1() = default;
  auto init() -> void override;
  simulator::SimulationLoop* simulator_ptr;
  physics::PhysicsEngine* engine_ptr;
  simulator::EventManager* manager_ptr;
};
class SIRE_API InitHandler1 final : public core::HandlerBase {
 public:
  InitHandler1() : HandlerBase() {}
  ~InitHandler1() = default;
  auto init(simulator::SimulationLoop* simulator) -> void override;
  auto handle(core::EventBase* e) -> bool override;
  simulator::SimulationLoop* simulator_ptr;
};
class SIRE_API StepEvent1 final : public core::EventBase {
 public:
  StepEvent1() : EventBase() {
    setEventType("Step1");
    setEventId(kStepEventId);
  }
  ~StepEvent1() = default;
  auto init() -> void override;
  simulator::SimulationLoop* simulator_ptr;
  physics::PhysicsEngine* engine_ptr;
  simulator::EventManager* manager_ptr;
};
class SIRE_API StepHandler1 final : public core::HandlerBase {
 public:
  StepHandler1() : HandlerBase() {}
  ~StepHandler1() = default;
  auto init(simulator::SimulationLoop* simulator) -> void override;
  auto handle(core::EventBase* e) -> bool override;
  simulator::SimulationLoop* simulator_ptr;
};
class SIRE_API CtrlEvent1 final : public core::EventBase {
  public:
  CtrlEvent1() : EventBase() {
    setEventType("Ctrl1");
    setEventId(kCtrlEventId);
  }
   ~CtrlEvent1() = default;
   auto init() -> void override;
   simulator::SimulationLoop* simulator_ptr;
   physics::PhysicsEngine* engine_ptr;
   simulator::EventManager* manager_ptr;
 };
 class SIRE_API CtrlHandler1 final : public core::HandlerBase {
  public:
  CtrlHandler1() : HandlerBase() {}
   ~CtrlHandler1() = default;
   auto init(simulator::SimulationLoop* simulator) -> void override;
   auto handle(core::EventBase* e) -> bool override;
   simulator::SimulationLoop* simulator_ptr;
 };
}  // namespace sire::simulator
#endif