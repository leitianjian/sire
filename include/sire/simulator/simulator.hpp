#ifndef SIRE_SIMULATOR_HPP_
#define SIRE_SIMULATOR_HPP_
#include <sire_lib_export.h>

#include <aris/core/object.hpp>
#include <aris/dynamic/model.hpp>
#include <aris/server/interface.hpp>

#include "sire/core/module_base.hpp"
#include "sire/core/sire_decl_def_macro.hpp"
#include "sire/integrator/integrator_base.hpp"
#include "sire/physics/physics_engine.hpp"
#include "sire/sensor/sensor.hpp"

namespace sire {
namespace middleware {
class SireMiddleware;
}
namespace simulator {
// Under Model node, using pointer to get useful resource
class SIRE_API Simulator {
  using IntegratorPool = aris::core::PointerArray<IntegratorBase>;
  using SensorPool = aris::core::PointerArray<sensor::SensorBase>;

 public:
  auto resetIntegratorPoolPtr(IntegratorPool* new_ptr) -> void;
  auto integratorPoolPtr() const -> const IntegratorPool*;
  auto integratorPoolPtr() -> IntegratorPool* {
    return const_cast<IntegratorPool*>(
        static_cast<const Simulator*>(this)->integratorPoolPtr());
  }

  auto resetSensorPoolPtr(SensorPool* new_ptr) -> void;
  auto sensorPoolPtr() const -> const SensorPool*;
  auto sensorPoolPtr() -> SensorPool* {
    return const_cast<SensorPool*>(
        static_cast<const Simulator*>(this)->sensorPoolPtr());
  }

  // PhysicsEngine //
  auto resetPhysicsEnginePtr(physics::PhysicsEngine* engine) -> void;
  auto physicsEnginePtr() const -> const physics::PhysicsEngine*;
  auto physicsEnginePtr() -> physics::PhysicsEngine* {
    return const_cast<physics::PhysicsEngine*>(
        static_cast<const Simulator*>(this)->physicsEnginePtr());
  }

  auto getModelState(const std::function<void(aris::server::ControlServer&,
                                              Simulator&, std::any&)>& get_func,
                     std::any& get_data) -> void;

  inline auto model() noexcept -> aris::dynamic::Model*;

  auto deltaT() -> double;
  auto setDeltaT(double delta_t_in) -> void;

  // TODO(leitianjian)：
  //   可以使用更有效率的方式，restore只需要更换Model的指针就可以，
  //   但是实现比较复杂，涉及到全局的Model的指针更换，暂时不考虑这个方法
  auto backupModel() -> void;
  auto restoreModel() -> void;

  // operation to control simulator outside //
  auto tickOnStep() -> void {
    // model -> forwardDynamics()
    // model -> integratePartAs()
    // model -> integrateMotionAs()
    // model -> forwardKinematics() 最小二乘
  }
  auto init(middleware::SireMiddleware* middleware) -> void;
  auto start() -> void;
  auto pause() -> void{};
  auto playback() -> void{};
  auto stop() -> void{};

  Simulator();
  virtual ~Simulator();
  SIRE_DECLARE_MOVE_CTOR(Simulator);

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