#ifndef SIRE_SIMULATOR_MODULES_HPP_
#define SIRE_SIMULATOR_MODULES_HPP_

#include <aris/core/object.hpp>

#include "sire/core/module_base.hpp"
#include "sire/integrator/integrator_base.hpp"
#include "sire/sensor/sensor.hpp"
#include "sire/simulator/simulator.hpp"

namespace sire {
namespace middleware {
class SireMiddleware;
}
namespace simulator {
class SimulatorModules {
  using IntegratorPool = aris::core::PointerArray<IntegratorBase>;
  using SensorPool = aris::core::PointerArray<sensor::SensorBase>;

 public:
  auto init(middleware::SireMiddleware* middleware_ptr) -> void;

  // Integrator //
  auto resetIntegratorPool(IntegratorPool* pool) -> void;
  auto integratorPool() const -> const IntegratorPool&;
  auto integratorPool() -> IntegratorPool& {
    return const_cast<IntegratorPool&>(
        static_cast<const SimulatorModules*>(this)->integratorPool());
  }

  // Sensors //
  auto resetSensorPool(SensorPool* pool) -> void;
  auto sensorPool() const -> const SensorPool&;
  auto sensorPool() -> SensorPool& {
    return const_cast<SensorPool&>(
        static_cast<const SimulatorModules*>(this)->sensorPool());
  }

  SimulatorModules();
  ~SimulatorModules();
  SIRE_DECLARE_MOVE_CTOR(SimulatorModules);

 private:
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;
};
}  // namespace simulator
}  // namespace sire
#endif