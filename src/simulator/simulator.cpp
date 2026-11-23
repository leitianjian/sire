#include "sire/simulator/simulator.hpp"

#include <aris/core/core.hpp>
#include <aris/core/object.hpp>
#include <aris/server/control_server.hpp>

#include "sire/ext/json.hpp"
#include "sire/server/api.hpp"

namespace sire::simulator {
struct Simulator::Imp {
  std::unique_ptr<aris::dynamic::Model> model_{new aris::dynamic::Model()};
  std::unique_ptr<simulator::SimulationLoop> simulation_loop_{
      new simulator::SimulationLoop()};
  std::unique_ptr<physics::PhysicsEngine> physics_engine_{
      new physics::PhysicsEngine()};
  std::unique_ptr<simulator::SimulatorModules> simulator_modules_{
      new simulator::SimulatorModules()};
};
Simulator::Simulator() : imp_(new Imp) {}
Simulator::~Simulator() = default;
Simulator::Simulator(Simulator&& other) = default;
Simulator& Simulator::operator=(Simulator&& other) = default;

auto Simulator::init() -> void {
  // generalForce没有办法在model没有inti的情况下用代码add，所以要两次init
  imp_->model_->init();
  imp_->physics_engine_->init(imp_->model_.get(), imp_->simulation_loop_.get());
  imp_->simulator_modules_->init(imp_->physics_engine_.get());
  imp_->simulation_loop_->init(imp_->model_.get(), imp_->physics_engine_.get(),
                               imp_->simulator_modules_.get());
  imp_->model_->init();
}
auto Simulator::resetModel(aris::dynamic::Model* model) -> void {
  imp_->model_.reset(model);
}
auto Simulator::model() -> aris::dynamic::Model& { return *imp_->model_; }
auto Simulator::resetSimulationLoop(simulator::SimulationLoop* simulator_loop)
    -> void {
  imp_->simulation_loop_.reset(simulator_loop);
}
auto Simulator::simulationLoop() const -> const simulator::SimulationLoop& {
  return *imp_->simulation_loop_;
}
auto Simulator::resetPhysicsEngine(physics::PhysicsEngine* engine) -> void {
  imp_->physics_engine_.reset(engine);
}
auto Simulator::physicsEngine() const -> const physics::PhysicsEngine& {
  return *imp_->physics_engine_;
}
auto Simulator::resetSimulatorModules(simulator::SimulatorModules* pool)
    -> void {
  imp_->simulator_modules_.reset(pool);
}
auto Simulator::simulatorModules() const -> const simulator::SimulatorModules& {
  return *imp_->simulator_modules_;
}
auto Simulator::simReset() -> void {
  imp_->simulation_loop_->reset();
  imp_->physics_engine_->reset();
  imp_->simulator_modules_->reset();
}

ARIS_REGISTRATION {
  typedef simulator::SimulationLoop& (Simulator::*SimulationLoopFunc)();
  typedef physics::PhysicsEngine& (Simulator::*PhysicsEngineFunc)();
  typedef simulator::SimulatorModules& (Simulator::*SimulatorMudulesFunc)();
  typedef aris::dynamic::Model& (Simulator::*ModelFunc)();
  aris::core::class_<Simulator>("Simulator")
      .prop("model", &Simulator::resetModel, ModelFunc(&Simulator::model))
      .prop("simulation_loop", &Simulator::resetSimulationLoop,
            SimulationLoopFunc(&Simulator::simulationLoop))
      .prop("physics_engine", &Simulator::resetPhysicsEngine,
            PhysicsEngineFunc(&Simulator::physicsEngine))
      .prop("simulator_modules", &Simulator::resetSimulatorModules,
            SimulatorMudulesFunc(&Simulator::simulatorModules));
}
}  // namespace sire::simulator