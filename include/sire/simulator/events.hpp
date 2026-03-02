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
  auto integrate(core::EventBase* e) -> void override{};
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
  auto integrate(core::EventBase* e) -> void override{};
  auto handle(core::EventBase* e) -> bool override;
  simulator::SimulationLoop* simulator_ptr;
};

class SIRE_API InitEvent1 final : public core::EventBase {
 public:
  InitEvent1() : EventBase()  {
    setEventType("Init1");
    setEventId(kInitEventId);
  }
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
  auto integrate(core::EventBase* e) -> void override{};
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
  auto integrate(core::EventBase* e) -> void override{};
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
  auto integrate(core::EventBase* e) -> void override{};
  auto handle(core::EventBase* e) -> bool override;
  simulator::SimulationLoop* simulator_ptr;
};

// without initial penetration elimination
class SIRE_API InitEvent2 final : public core::EventBase {
 public:
  InitEvent2() : EventBase() {}
  ~InitEvent2() = default;
  auto init() -> void override;
  simulator::SimulationLoop* simulator_ptr;
  physics::PhysicsEngine* engine_ptr;
  simulator::EventManager* manager_ptr;
};
class SIRE_API InitHandler2 final : public core::HandlerBase {
 public:
  InitHandler2() : HandlerBase() {}
  ~InitHandler2() = default;
  auto init(simulator::SimulationLoop* simulator) -> void override;
  auto integrate(core::EventBase* e) -> void override{};
  auto handle(core::EventBase* e) -> bool override;
  simulator::SimulationLoop* simulator_ptr;
};
class SIRE_API StepEvent2 final : public core::EventBase {
 public:
  StepEvent2() : EventBase() {
    setEventType("Step2");
    setEventId(kStepEventId);
  }
  ~StepEvent2() = default;
  auto init() -> void override;
  simulator::SimulationLoop* simulator_ptr;
  physics::PhysicsEngine* engine_ptr;
  simulator::EventManager* manager_ptr;
};
class SIRE_API StepHandler2 final : public core::HandlerBase {
 public:
  StepHandler2() : HandlerBase() {}
  ~StepHandler2() = default;
  auto init(simulator::SimulationLoop* simulator) -> void override;
  auto integrate(core::EventBase* e) -> void override{};
  auto handle(core::EventBase* e) -> bool override;
  simulator::SimulationLoop* simulator_ptr;
};
class SIRE_API CtrlEvent2 final : public core::EventBase {
 public:
  CtrlEvent2() : EventBase() {
    setEventType("Ctrl2");
    setEventId(kCtrlEventId);
  }
  ~CtrlEvent2() = default;
  auto init() -> void override;
  simulator::SimulationLoop* simulator_ptr;
  physics::PhysicsEngine* engine_ptr;
  simulator::EventManager* manager_ptr;
};
class SIRE_API CtrlHandler2 final : public core::HandlerBase {
 public:
  CtrlHandler2() : HandlerBase() {}
  ~CtrlHandler2() = default;
  auto init(simulator::SimulationLoop* simulator) -> void override;
  auto integrate(core::EventBase* e) -> void override{};
  auto handle(core::EventBase* e) -> bool override;
  simulator::SimulationLoop* simulator_ptr;
};
// without adjustPenetrationPosition
class SIRE_API InitHandler3 final : public core::HandlerBase {
 public:
  InitHandler3() : HandlerBase() {}
  ~InitHandler3() = default;
  auto init(simulator::SimulationLoop* simulator) -> void override;
  auto integrate(core::EventBase* e) -> void override;
  auto handle(core::EventBase* e) -> bool override;
  simulator::SimulationLoop* simulator_ptr;
};
class SIRE_API StepHandler3 final : public core::HandlerBase {
 public:
  StepHandler3() : HandlerBase() {}
  ~StepHandler3() = default;
  auto init(simulator::SimulationLoop* simulator) -> void override;
  auto integrate(core::EventBase* e) -> void override;
  auto handle(core::EventBase* e) -> bool override;
  simulator::SimulationLoop* simulator_ptr;
};
class SIRE_API CtrlHandler3 final : public core::HandlerBase {
 public:
  CtrlHandler3() : HandlerBase() {}
  ~CtrlHandler3() = default;
  auto init(simulator::SimulationLoop* simulator) -> void override;
  auto integrate(core::EventBase* e) -> void override;
  auto handle(core::EventBase* e) -> bool override;
  simulator::SimulationLoop* simulator_ptr;
};
// without adjustPenetrationPosition
class SIRE_API InitHandler4 final : public core::HandlerBase {
 public:
  InitHandler4() : HandlerBase() {}
  ~InitHandler4() = default;
  auto init(simulator::SimulationLoop* simulator) -> void override;
  auto integrate(core::EventBase* e) -> void override;
  auto handle(core::EventBase* e) -> bool override;
  simulator::SimulationLoop* simulator_ptr;
};
class SIRE_API StepHandler4 final : public core::HandlerBase {
 public:
  StepHandler4() : HandlerBase() {}
  ~StepHandler4() = default;
  auto init(simulator::SimulationLoop* simulator) -> void override;
  auto integrate(core::EventBase* e) -> void override;
  auto handle(core::EventBase* e) -> bool override;
  simulator::SimulationLoop* simulator_ptr;
};
class SIRE_API CtrlHandler4 final : public core::HandlerBase {
 public:
  CtrlHandler4() : HandlerBase() {}
  ~CtrlHandler4() = default;
  auto init(simulator::SimulationLoop* simulator) -> void override;
  auto integrate(core::EventBase* e) -> void override;
  auto handle(core::EventBase* e) -> bool override;
  simulator::SimulationLoop* simulator_ptr;
};
}  // namespace sire::simulator
#endif