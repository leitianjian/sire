#ifndef SIRE_MIDDLEWARE_HPP_
#define SIRE_MIDDLEWARE_HPP_

#include <functional>
#include <iostream>
#include <string_view>

#include <sire_lib_export.h>

#include <aris/server/interface.hpp>
#include <aris/server/middle_ware.hpp>

#include "sire/core/module_base.hpp"
#include "sire/integrator/integrator_base.hpp"
#include "sire/physics/physics_engine.hpp"
#include "sire/sensor/sensor.hpp"
#include "sire/server/interface.hpp"
#include "sire/simulator/simulator_base.hpp"
#include "sire/simulator/simulator_modules.hpp"

// TODO(leitianjian): SireMiddleware和Programming middleware功能上需要合并
namespace sire::middleware {
using namespace std;
class SIRE_API SireMiddleware : public aris::server::MiddleWare {
 public:
  auto virtual init() -> void override;
  auto virtual executeCmd(std::string_view str,
                          std::function<void(std::string)> send_ret,
                          aris::server::Interface* interface) -> int override;
  // Simulator
  auto resetSimulatorBase(simulator::SimulatorBase* simulator) -> void;
  auto simulatorBase() const -> const simulator::SimulatorBase&;
  auto simulatorBase() -> simulator::SimulatorBase& {
    return const_cast<simulator::SimulatorBase&>(
        static_cast<const SireMiddleware&>(*this).simulatorBase());
  }

  // Physics Engine
  auto resetPhysicsEngine(physics::PhysicsEngine* engine) -> void;
  auto physicsEngine() const -> const physics::PhysicsEngine&;
  auto physicsEngine() -> physics::PhysicsEngine& {
    return const_cast<physics::PhysicsEngine&>(
        static_cast<const SireMiddleware&>(*this).physicsEngine());
  }

  // Simulator Modules
  auto resetSimulatorModules(simulator::SimulatorModules* pool) -> void;
  auto simulatorModules() const -> const simulator::SimulatorModules&;
  auto simulatorModules() -> simulator::SimulatorModules& {
    return const_cast<simulator::SimulatorModules&>(
        static_cast<const SireMiddleware&>(*this).simulatorModules());
  }

  SireMiddleware();
  virtual ~SireMiddleware();
  SireMiddleware(SireMiddleware&& other);
  SireMiddleware& operator=(SireMiddleware&& other);

 private:
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;
};

}  // namespace sire::middleware

#endif  // SIRE_MIDDLEWARE_HPP_