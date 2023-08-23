#include "sire/simulator/simulator_modules.hpp"

#include "sire/core/sire_assert.hpp"
#include "sire/middleware/sire_middleware.hpp"
#include "sire/physics/physics_engine.hpp"

namespace sire::simulator {
struct SimulatorModules::Imp {
  physics::PhysicsEngine* engine_ptr_;
  std::unique_ptr<IntegratorPool> integrator_pool_;
  std::unique_ptr<SensorPool> sensor_pool_;
};
SimulatorModules::SimulatorModules() : imp_(new Imp) {}
SimulatorModules::~SimulatorModules() = default;
SIRE_DEFINE_MOVE_CTOR_CPP(SimulatorModules);

auto SimulatorModules::init(middleware::SireMiddleware* middleware_ptr)
    -> void {
  imp_->engine_ptr_ = &middleware_ptr->physicsEngine();
  SIRE_ASSERT(imp_->engine_ptr_ != nullptr);
  for (auto& integrator : *imp_->integrator_pool_) {
    integrator.init(&(middleware_ptr->physicsEngine()));
  }
  for (auto& sensor : *imp_->sensor_pool_) {
    sensor.init();
  }
}

auto SimulatorModules::resetIntegratorPool(IntegratorPool* pool) -> void {
  imp_->integrator_pool_.reset(pool);
}
auto SimulatorModules::integratorPool() const -> const IntegratorPool& {
  return *imp_->integrator_pool_;
}
auto SimulatorModules::resetSensorPool(SensorPool* pool) -> void {
  imp_->sensor_pool_.reset(pool);
}
auto SimulatorModules::sensorPool() const -> const SensorPool& {
  return *imp_->sensor_pool_;
}

ARIS_REGISTRATION {
  using IntegratorPool = aris::core::PointerArray<IntegratorBase>;
  using SensorPool = aris::core::PointerArray<sensor::SensorBase>;

  aris::core::class_<IntegratorPool>("IntegratorPoolObject").asRefArray();
  aris::core::class_<SensorPool>("SensorPoolObject").asRefArray();

  typedef IntegratorPool& (SimulatorModules::*IntegratorPoolFunc)();
  typedef SensorPool& (SimulatorModules::*SensorPoolFunc)();

  aris::core::class_<SimulatorModules>("SimulatorModules")
      .prop("integrator_pool", &SimulatorModules::resetIntegratorPool,
            IntegratorPoolFunc(&SimulatorModules::integratorPool))
      .prop("sensor_pool", &SimulatorModules::resetSensorPool,
            SensorPoolFunc(&SimulatorModules::sensorPool));
}
}  // namespace sire::simulator