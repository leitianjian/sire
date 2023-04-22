#include "sire/simulator/simulator.hpp"

#include <aris/core/object.hpp>
#include <aris/core/reflection.hpp>
#include <aris/dynamic/model_base.hpp>

#include "sire/core/constants.hpp"
#include "sire/sensor/sensor.hpp"

namespace sire::simulator {
struct Simulator::Imp {
  std::unique_ptr<aris::dynamic::ModelBase> model_;
  std::unique_ptr<physics::PhysicsEngine> physics_engine_;
  std::unique_ptr<aris::core::PointerArray<aris::server::Interface>>
      interface_pool_{new aris::core::PointerArray<aris::server::Interface>};
  std::unique_ptr<aris::core::PointerArray<Integrator>> integrator_pool_{
      new aris::core::PointerArray<Integrator>};
  std::unique_ptr<aris::core::PointerArray<sensor::SensorBase>> sensor_pool_{
      new aris::core::PointerArray<sensor::SensorBase>};
};
auto Simulator::instance() noexcept -> Simulator& {
  static Simulator instance;
  return instance;
}
auto Simulator::resetModel(aris::dynamic::ModelBase* model) -> void {
  imp_->model_.reset(model);
}
auto Simulator::model() -> aris::dynamic::ModelBase& { return *imp_->model_; }

auto Simulator::resetIntegratorPool(aris::core::PointerArray<Integrator>* pool)
    -> void {
  imp_->integrator_pool_.reset(pool);
}
auto Simulator::integratorPool() -> aris::core::PointerArray<Integrator>& {
  return *imp_->integrator_pool_;
}

auto Simulator::resetPhysicsEngine(physics::PhysicsEngine* engine) -> void {
  imp_->physics_engine_.reset(engine);
}
auto Simulator::physicsEngine() -> physics::PhysicsEngine& {
  return *imp_->physics_engine_;
}

auto Simulator::resetInterfacePool(
    aris::core::PointerArray<aris::server::Interface>* pool) -> void {
  imp_->interface_pool_.reset(pool);
}
auto Simulator::interfacePool()
    -> aris::core::PointerArray<aris::server::Interface>& {
  return *imp_->interface_pool_;
}

auto Simulator::resetSensorPool(
    aris::core::PointerArray<sensor::SensorBase>* pool) -> void {
  imp_->sensor_pool_.reset(pool);
}
auto Simulator::sensorPool() -> aris::core::PointerArray<sensor::SensorBase>& {
  return *imp_->sensor_pool_;
}

}  // namespace sire::simulator