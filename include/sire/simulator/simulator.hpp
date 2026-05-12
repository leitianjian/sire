#ifndef SIRE_SIMULATOR_HPP_
#define SIRE_SIMULATOR_HPP_

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
#include "sire/simulator/simulation_loop.hpp"
#include "sire/simulator/simulator_modules.hpp"

// TODO(leitianjian): SireMiddleware和Programming middleware功能上需要合并
namespace sire::simulator {
class SIRE_API Simulator {
 public:
  auto init() -> void;
  auto resetModel(aris::dynamic::Model* model) -> void;
  auto model() -> aris::dynamic::Model&;
  // Simulator
  auto resetSimulationLoop(simulator::SimulationLoop* simulator) -> void;
  auto simulationLoop() const -> const simulator::SimulationLoop&;
  auto simulationLoop() -> simulator::SimulationLoop& {
    return const_cast<simulator::SimulationLoop&>(
        static_cast<const Simulator&>(*this).simulationLoop());
  }

  // Physics Engine
  auto resetPhysicsEngine(physics::PhysicsEngine* engine) -> void;
  auto physicsEngine() const -> const physics::PhysicsEngine&;
  auto physicsEngine() -> physics::PhysicsEngine& {
    return const_cast<physics::PhysicsEngine&>(
        static_cast<const Simulator&>(*this).physicsEngine());
  }

  // Simulator Modules
  auto resetSimulatorModules(simulator::SimulatorModules* pool) -> void;
  auto simulatorModules() const -> const simulator::SimulatorModules&;
  auto simulatorModules() -> simulator::SimulatorModules& {
    return const_cast<simulator::SimulatorModules&>(
        static_cast<const Simulator&>(*this).simulatorModules());
  }
  auto simReset() -> void;

  Simulator();
  ~Simulator();
  Simulator(Simulator&& other);
  Simulator& operator=(Simulator&& other);

 private:
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;
};

}  // namespace sire::simulator

#endif  // SIRE_SIMULATOR_HPP_