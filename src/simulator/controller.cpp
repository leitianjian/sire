#include "sire/simulator/controller.hpp"

#include <aris/core/reflection.hpp>
#include <aris/dynamic/model.hpp>

#include "sire/actuator/actuator.hpp"
#include "sire/core/sire_assert.hpp"
#include "sire/physics/physics_engine.hpp"
#include "sire/simulator/simulation_loop.hpp"

namespace sire::simulator {
struct Controller::Imp {
  aris::dynamic::Model* modelPtr_{nullptr};
  physics::PhysicsEngine* physicsEnginePtr_{nullptr};
  simulator::SimulationLoop* simulationLoopPtr_{nullptr};
};
Controller::Controller() : imp_(std::make_unique<Imp>()) {}
Controller::~Controller() = default;
auto Controller::init(simulator::SimulationLoop* loopPtr) -> void {
  resetSimulationLoopPtr(loopPtr);
  resetPhysicsEnginePtr(loopPtr->physicsEnginePtr());
  resetModelPtr(loopPtr->model());
}
auto Controller::resetPhysicsEnginePtr(physics::PhysicsEngine* engine) -> void {
  SIRE_ASSERT(engine != nullptr);
  imp_->physicsEnginePtr_ = engine;
}
auto Controller::physicsEnginePtr() const -> const physics::PhysicsEngine* {
  return imp_->physicsEnginePtr_;
}
auto Controller::resetModelPtr(aris::dynamic::Model* modelPtr) -> void {
  SIRE_ASSERT(modelPtr != nullptr);
  imp_->modelPtr_ = modelPtr;
}
auto Controller::modelPtr() const -> const aris::dynamic::Model* {
  return imp_->modelPtr_;
}
auto Controller::resetSimulationLoopPtr(simulator::SimulationLoop* loopPtr)
    -> void {
  SIRE_ASSERT(loopPtr != nullptr);
  imp_->simulationLoopPtr_ = loopPtr;
}
auto Controller::simulationLoopPtr() const -> const simulator::SimulationLoop* {
  return imp_->simulationLoopPtr_;
}
auto ZeroForce::control() -> void {
  auto& motionPool = Controller::modelPtr()->motionPool();
  for (sire::Size i{0}; i < motionPool.size(); ++i) {
    motionPool[i].setMf(0 - motionPool[i].mp());
  }
}
auto ZeroPosition::control() -> void {
  // auto& motionPool = Controller::modelPtr()->motionPool();
  // std::vector<double> target_q(motionPool.size());
  // for (sire::Size i{0}; i < motionPool.size(); ++i) {
  //   target_q[i] = 0.0;
  // }
  // // target_q[0] = 0.01;
  // for (sire::Size i{0}; i < motionPool.size(); ++i) {
  //   if (auto* actuator = dynamic_cast<actuator::ActuatorSISO*>(&motionPool[i]);
  //       actuator != nullptr) {
  //     actuator->setDesiredValue(target_q[i]);
  //   }
  //   // motionPool[i].setP(dq.data() + i);
  // }
}
ARIS_REGISTRATION {
  aris::core::class_<Controller>("SireController");
  aris::core::class_<ZeroForce>("ZeroForce").inherit<Controller>();
  aris::core::class_<ZeroPosition>("ZeroPosition").inherit<Controller>();
}
}  // namespace sire::simulator