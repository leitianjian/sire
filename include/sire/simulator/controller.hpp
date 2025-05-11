#ifndef SIRE_CONTROLLER_HPP_
#define SIRE_CONTROLLER_HPP_
#include <array>
#include <vector>

#include <sire_lib_export.h>

#include <aris/core/object.hpp>
#include <aris/dynamic/model.hpp>

#include "sire/core/constants.hpp"
#include "sire/physics/physics_engine.hpp"

namespace sire::simulator {
class SIRE_API Controller : public aris::core::NamedObject {
 public:
  auto virtual control() -> void { THROW_FILE_LINE("non-implemented Controller"); };
  auto virtual init(simulator::SimulationLoop* loopPtr) -> void;
  // PhysicsEngine //
  auto resetPhysicsEnginePtr(physics::PhysicsEngine* engine) -> void;
  auto physicsEnginePtr() const -> const physics::PhysicsEngine*;
  auto physicsEnginePtr() -> physics::PhysicsEngine* {
    return const_cast<physics::PhysicsEngine*>(
        static_cast<const Controller*>(this)->physicsEnginePtr());
  }
  auto resetModelPtr(aris::dynamic::Model* modelPtr) -> void;
  auto modelPtr() const -> const aris::dynamic::Model*;
  auto modelPtr() -> aris::dynamic::Model* {
    return const_cast<aris::dynamic::Model*>(
        static_cast<const Controller*>(this)->modelPtr());
  }
  auto resetSimulationLoopPtr(simulator::SimulationLoop* loopPtr) -> void;
  auto simulationLoopPtr() const -> const simulator::SimulationLoop*;
  auto simulationLoopPtr() -> simulator::SimulationLoop* {
    return const_cast<simulator::SimulationLoop*>(
        static_cast<const Controller*>(this)->simulationLoopPtr());
  }
  Controller();
  virtual ~Controller();

 private:
  struct Imp;
  std::unique_ptr<Imp> imp_;
};
class SIRE_API ZeroForce final : public Controller {
  auto virtual control() -> void;
};
class SIRE_API ZeroPosition final : public Controller {
  auto virtual control() -> void;
};
}  // namespace sire::simulator
#endif