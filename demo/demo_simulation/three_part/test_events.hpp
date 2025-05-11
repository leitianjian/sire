#ifndef THREE_PART_EVENT_HPP_
#define THREE_PART_EVENT_HPP_

#include "sire/core/event_base.hpp"
#include "sire/simulator/event_manager.hpp"
#include "sire/core/handler_base.hpp"
#include "sire/core/trigger_base.hpp"
#include "sire/simulator/simulation_loop.hpp"
namespace sire::three_part {
class InitEvent1 final : public core::EventBase {
 public:
  InitEvent1() : EventBase() { }
  ~InitEvent1() = default;
  auto init() -> void override;
  simulator::SimulationLoop* simulator_ptr;
  physics::PhysicsEngine* engine_ptr;
  simulator::EventManager* manager_ptr;
};
class InitHandler final : public core::HandlerBase {
 public:
  InitHandler() : HandlerBase() {}
  ~InitHandler() = default;
  auto init(simulator::SimulationLoop* simulator) -> void override;
  auto handle(core::EventBase* e) -> bool override;
  simulator::SimulationLoop* simulator_ptr;
};
class StepEvent final : public core::EventBase {
 public:
  StepEvent() : EventBase() {}
  ~StepEvent() = default;
  auto init() -> void override;
  simulator::SimulationLoop* simulator_ptr;
  physics::PhysicsEngine* engine_ptr;
  simulator::EventManager* manager_ptr;
};
class StepHandler final : public core::HandlerBase {
 public:
  StepHandler() : HandlerBase() {}
  ~StepHandler() = default;
  auto init(simulator::SimulationLoop* simulator) -> void override;
  auto handle(core::EventBase* e) -> bool override;
  simulator::SimulationLoop* simulator_ptr;
};
}  // namespace sire::simulator
#endif