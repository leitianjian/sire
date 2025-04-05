#ifndef SIRE_EVENT_GENERATOR_HPP_
#define SIRE_EVENT_GENERATOR_HPP_
#include <sire_lib_export.h>

#include <aris/core/object.hpp>
#include <aris/dynamic/model.hpp>
#include <aris/server/control_server.hpp>

#include "sire/core/contact_pair_manager.hpp"
#include "sire/core/event_base.hpp"
#include "sire/core/event_manager.hpp"
#include "sire/core/prop_map.hpp"
#include "sire/integrator/integrator_base.hpp"
#include "sire/physics/physics_engine.hpp"
#include "sire/sensor/sensor.hpp"
#include "sire/simulator/events.hpp"
#include "sire/simulator/simulation_loop.hpp"

namespace sire {
namespace simulator {
// Under Model node, using pointer to get useful resource
class SIRE_API EventGenerator {
  using IntegratorPool = aris::core::PointerArray<IntegratorBase>;
  using SensorPool = aris::core::PointerArray<sensor::SensorBase>;

 public:
  // Event 相关
  auto createEventById(sire::Size event_id) -> std::unique_ptr<core::EventBase>;
  auto createHandlerById(sire::Size handler_id)
      -> std::unique_ptr<core::HandlerBase>;

  auto createHandlerByEventId(sire::Size event_id)
      -> std::unique_ptr<core::HandlerBase>;

  auto resetSimulationLoopPtr(SimulationLoop* loop) -> void;
  auto simulationLoopPtr() const -> const SimulationLoop*;
  auto simulationLoopPtr() -> SimulationLoop* {
    return const_cast<SimulationLoop*>(
        static_cast<const EventGenerator*>(this)->simulationLoopPtr());
  }

  auto resetIntegratorPoolPtr(IntegratorPool* new_ptr) -> void;
  auto integratorPoolPtr() const -> const IntegratorPool*;
  auto integratorPoolPtr() -> IntegratorPool* {
    return const_cast<IntegratorPool*>(
        static_cast<const EventGenerator*>(this)->integratorPoolPtr());
  }

  auto resetSensorPoolPtr(SensorPool* new_ptr) -> void;
  auto sensorPoolPtr() const -> const SensorPool*;
  auto sensorPoolPtr() -> SensorPool* {
    return const_cast<SensorPool*>(
        static_cast<const EventGenerator*>(this)->sensorPoolPtr());
  }

  // PhysicsEngine //
  auto resetPhysicsEnginePtr(physics::PhysicsEngine* engine) -> void;
  auto physicsEnginePtr() const -> const physics::PhysicsEngine*;
  auto physicsEnginePtr() -> physics::PhysicsEngine* {
    return const_cast<physics::PhysicsEngine*>(
        static_cast<const EventGenerator*>(this)->physicsEnginePtr());
  }

  auto setGlobalVariablePool(core::PropMap& map) -> void;
  auto getGlobalVariablePool() const -> const core::PropMap&;
  auto getGlobalVariablePool() -> core::PropMap& {
    return const_cast<core::PropMap&>(
        static_cast<const EventGenerator&>(*this).getGlobalVariablePool());
  }

  auto resetEventManager(core::EventManager* manager) -> void;
  auto eventManager() const -> const core::EventManager&;
  auto eventManager() -> core::EventManager& {
    return const_cast<core::EventManager&>(
        static_cast<const EventGenerator*>(this)->eventManager());
  }

  auto generateEvents() -> std::unique_ptr<core::EventBase>;

  auto targetRealtimeRate() -> double;
  auto realtimeRate() -> double;
  auto setRealtimeRate(double rate) -> void;

  auto contactPairManager() noexcept -> core::ContactPairManager*;

  auto timer() -> core::Timer&;
  auto simDuration() -> double;
  auto setSimDuration(double simDuration) -> void;
  // operation to control simulator outside //
  auto init() -> void;
  auto reset() -> void;

  EventGenerator();
  virtual ~EventGenerator();
  SIRE_DECLARE_MOVE_CTOR(EventGenerator);

 protected:
  auto collisionDetection() -> void;
  auto integrateAs2Ps() -> void;
  auto updateSysTime() -> void;

 private:
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;
};
}  // namespace simulator
}  // namespace sire
#endif