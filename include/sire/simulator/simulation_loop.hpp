#ifndef SIRE_SIMULATION_LOOP_HPP_
#define SIRE_SIMULATION_LOOP_HPP_
#include <sire_lib_export.h>

#include <aris/core/object.hpp>
#include <aris/dynamic/model.hpp>
#include <aris/server/interface.hpp>

#include "sire/core/contact_pair_manager.hpp"
#include "sire/core/event_base.hpp"
#include "sire/core/handler_base.hpp"
#include "sire/core/module_base.hpp"
#include "sire/core/sire_decl_def_macro.hpp"
#include "sire/core/timer.hpp"
#include "sire/core/trigger_base.hpp"
#include "sire/ext/json.hpp"
#include "sire/integrator/integrator_base.hpp"
#include "sire/physics/physics_engine.hpp"
#include "sire/sensor/sensor.hpp"
#include "sire/simulator/controller.hpp"
#include "sire/simulator/event_manager.hpp"
#include "sire/simulator/recorder.hpp"

namespace sire {
namespace middleware {
class SireMiddleware;
}
namespace simulator {
// Under Model node, using pointer to get useful resource
class SIRE_API SimulationLoop {
  using IntegratorPool = aris::core::PointerArray<IntegratorBase>;
  using SensorPool = aris::core::PointerArray<sensor::SensorBase>;

 public:
  auto createHandlerByEventId(sire::Size event_id)
      -> std::unique_ptr<core::HandlerBase>;

  auto resetIntegratorPoolPtr(IntegratorPool* new_ptr) -> void;
  auto integratorPoolPtr() const -> const IntegratorPool*;
  auto integratorPoolPtr() -> IntegratorPool* {
    return const_cast<IntegratorPool*>(
        static_cast<const SimulationLoop*>(this)->integratorPoolPtr());
  }

  auto resetSensorPoolPtr(SensorPool* new_ptr) -> void;
  auto sensorPoolPtr() const -> const SensorPool*;
  auto sensorPoolPtr() -> SensorPool* {
    return const_cast<SensorPool*>(
        static_cast<const SimulationLoop*>(this)->sensorPoolPtr());
  }

  // PhysicsEngine //
  auto resetPhysicsEnginePtr(physics::PhysicsEngine* engine) -> void;
  auto physicsEnginePtr() const -> const physics::PhysicsEngine*;
  auto physicsEnginePtr() -> physics::PhysicsEngine* {
    return const_cast<physics::PhysicsEngine*>(
        static_cast<const SimulationLoop*>(this)->physicsEnginePtr());
  }

  auto setGlobalVariablePool(core::PropMap& map) -> void;
  auto getGlobalVariablePool() const -> const core::PropMap&;
  auto getGlobalVariablePool() -> core::PropMap& {
    return const_cast<core::PropMap&>(
        static_cast<const SimulationLoop&>(*this).getGlobalVariablePool());
  }

  auto resetEventManager(simulator::EventManager* manager) -> void;
  auto eventManager() const -> const simulator::EventManager&;
  auto eventManager() -> simulator::EventManager& {
    return const_cast<simulator::EventManager&>(
        static_cast<const SimulationLoop*>(this)->eventManager());
  }

  auto resetController(simulator::Controller* ctrlPtr) -> void;
  auto controller() const -> const simulator::Controller&;
  auto controller() -> simulator::Controller& {
    return const_cast<simulator::Controller&>(
        static_cast<const SimulationLoop*>(this)->controller());
  }

  auto deltaT() -> double;
  auto setDeltaT(double delta_t_in) -> void;
  auto ctrlT() -> double;
  auto setCtrlT(double ctrlt_) -> void;
  auto targetRealtimeRate() -> double;
  auto realtimeRate() -> double;
  auto setRealtimeRate(double rate) -> void;
  // auto getModelState(const std::function<void(aris::server::ControlServer&,
  //                                             Simulator&, std::any&)>&
  //                                             get_func,
  //                    std::any& get_data) -> void;

  auto model() noexcept -> aris::dynamic::Model*;
  auto contactPairManager() noexcept -> core::ContactPairManager*;

  auto getModelState(
      const std::function<void(aris::server::ControlServer&, SimulationLoop&,
                               std::any&)>& get_func,
      std::any& get_data) -> void;

  auto timer() -> core::Timer&;
  auto simTime() -> double;
  auto recorder() -> simulator::Recorder&;
  auto recordsToJson() -> nlohmann::json;
  auto simDuration() -> double;
  auto setSimDuration(double simDuration) -> void;
  // TODO(leitianjian)：
  //   可以使用更有效率的方式，restore只需要更换Model的指针就可以，
  //   但是实现比较复杂，涉及到全局的Model的指针更换，暂时不考虑这个方法
  auto backupModel() -> void;
  auto restoreModel() -> void;

  // operation to control simulator outside //
  auto isTimeout() -> bool;
  auto isEventListEmpty() -> bool;
  auto init(middleware::SireMiddleware* middleware) -> void;
  auto start() -> void;
  auto isRunning() -> bool;
  auto step(sire::Size frame_skip, bool pause_if_fast = false) -> void;
  auto pause() -> void;
  auto playback() -> void {};
  auto stop() -> void {};
  auto reset() -> void;

  SimulationLoop();
  virtual ~SimulationLoop();
  SIRE_DECLARE_MOVE_CTOR(SimulationLoop);

 protected:
  auto resolveContact() -> void;
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