#ifndef SIRE_INIT_EVENT_HPP_
#define SIRE_INIT_EVENT_HPP_
#include <sire_lib_export.h>

#include "sire/core/event_base.hpp"
#include "sire/core/event_manager.hpp"
#include "sire/core/handler_base.hpp"
#include "sire/core/trigger_base.hpp"
#include "sire/simulator/simulator_base.hpp"
namespace sire::simulator {
class SIRE_API InitTrigger final : public core::TriggerBase {
 public:
  InitTrigger() : TriggerBase() {}
  ~InitTrigger() = default;
  auto trigger(simulator::SimulatorBase*) -> void override;
};
class SIRE_API InitEvent final : public core::EventBase {
 public:
  InitEvent() : EventBase() { }
  ~InitEvent() = default;
  auto init() -> void override;
  simulator::SimulatorBase* simulator_ptr;
  physics::PhysicsEngine* engine_ptr;
  core::EventManager* manager_ptr;
};
class SIRE_API InitHandler final : public core::HandlerBase {
 public:
  InitHandler() : HandlerBase() {}
  ~InitHandler() = default;
  auto init(simulator::SimulatorBase* simulator) -> void override;
  auto handle(core::EventBase* e) -> bool override;
  simulator::SimulatorBase* simulator_ptr;
};
class SIRE_API StepTrigger final : public core::TriggerBase {};
class SIRE_API StepEvent final : public core::EventBase {
 public:
  StepEvent() : EventBase() {}
  ~StepEvent() = default;
  auto init() -> void override;
  simulator::SimulatorBase* simulator_ptr;
  physics::PhysicsEngine* engine_ptr;
  core::EventManager* manager_ptr;
};
class SIRE_API StepHandler final : public core::HandlerBase {
 public:
  StepHandler() : HandlerBase() {}
  ~StepHandler() = default;
  auto init(simulator::SimulatorBase* simulator) -> void override;
  auto handle(core::EventBase* e) -> bool override;
  simulator::SimulatorBase* simulator_ptr;
};
}  // namespace sire::simulator
#endif