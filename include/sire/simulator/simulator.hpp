#ifndef SIRE_SIMULATOR_HPP_
#define SIRE_SIMULATOR_HPP_
#include <sire_lib_export.h>

#include "sire/integrator/integrator.hpp"

#include <aris/dynamic/model.hpp>
#include <aris/server/interface.hpp>

#include "sire/physics/physics_engine.hpp"
#include "sire/sensor/sensor.hpp"

namespace sire::simulator {
class SIRE_API Simulator {
 public:
  static auto instance() noexcept -> Simulator&;
  // model base //
  template <typename T = aris::dynamic::Model, typename... Args>
  auto makeModel(Args&&... args) noexcept -> void {
    this->resetModel(new T(std::forward<Args>(args)...));
  }
  auto resetModel(aris::dynamic::ModelBase* model) -> void;
  auto model() -> aris::dynamic::ModelBase&;
  auto model() const -> const aris::dynamic::ModelBase& {
    return const_cast<Simulator*>(this)->model();
  }

  // Integrator //
  template <typename T = Integrator, typename... Args>
  auto makeInterface(Args&&... args) noexcept -> void {
    this->resetIntegratorPool(new T(std::forward<Args>(args)...));
  }
  auto resetIntegratorPool(aris::core::PointerArray<Integrator>* pool) -> void;
  auto integratorPool() -> aris::core::PointerArray<Integrator>&;
  auto integratorPool() const -> const aris::core::PointerArray<Integrator>& {
    return const_cast<Simulator*>(this)->integratorPool();
  }

  // PhysicsEngine //
  template <typename T = physics::PhysicsEngine, typename... Args>
  auto makePhysicsEngine(Args&&... args) noexcept -> void {
    this->resetPhysicsEngine(new T(std::forward<Args>(args)...));
  }
  auto resetPhysicsEngine(physics::PhysicsEngine* engine) -> void;
  auto physicsEngine() -> physics::PhysicsEngine&;
  auto physicsEngine() const -> const physics::PhysicsEngine& {
    return const_cast<Simulator*>(this)->physicsEngine();
  }

  // Interfaces //
  template <typename T = aris::server::Interface, typename... Args>
  auto makeInterfacePool(Args&&... args) noexcept -> void {
    this->resetModel(new T(std::forward<Args>(args)...));
  }
  auto resetInterfacePool(
      aris::core::PointerArray<aris::server::Interface>* pool) -> void;
  auto interfacePool() -> aris::core::PointerArray<aris::server::Interface>&;
  auto interfacePool() const
      -> const aris::core::PointerArray<aris::server::Interface>& {
    return const_cast<Simulator*>(this)->interfacePool();
  }

  // Sensors //
  template <typename T = sensor::SensorBase, typename... Args>
  auto makeSensorPool(Args&&... args) noexcept -> void {
    this->resetModel(new T(std::forward<Args>(args)...));
  }
  auto resetSensorPool(aris::core::PointerArray<sensor::SensorBase>* pool)
      -> void;
  auto sensorPool() -> aris::core::PointerArray<sensor::SensorBase>&;
  auto sensorPool() const
      -> const aris::core::PointerArray<sensor::SensorBase>& {
    return const_cast<Simulator*>(this)->sensorPool();
  }

  // operation to control simulator outside //
  auto init() -> void{};
  auto start() -> void{};
  auto pause() -> void{};
  auto playback() -> void{};
  auto stop() -> void{};

 private:
  struct Imp;
  std::unique_ptr<Imp> imp_;
};
}  // namespace sire::simulator
#endif