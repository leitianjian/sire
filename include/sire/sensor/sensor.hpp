#ifndef SIRE_SENSOR_HPP_
#define SIRE_SENSOR_HPP_

#include <functional>
#include <memory>
#include <string>

#include <sire_lib_export.h>

#include <aris/core/basic_type.hpp>
#include <aris/core/object.hpp>

#include "sire/core/constants.hpp"

namespace sire::sensor {
class SIRE_API SensorData {
 public:
  virtual ~SensorData() = default;
  SensorData() = default;
  auto virtual to_json_string(std::string& str) -> void{};
  auto virtual from_json_string(const std::string& j) -> void{};
};

class SIRE_API SensorBase {
 public:
  auto virtual copiedDataPtr() -> std::unique_ptr<SensorData> = 0;
  auto virtual init() -> void = 0;
  auto virtual start() -> void = 0;
  auto virtual stop() -> void = 0;
  auto isVirtual() const -> bool;
  auto setVirtual(bool is_virtual) -> void;
  auto activate() const -> bool;
  auto setActivate(bool is_activate) -> void;
  auto frequency() const -> aris::Size;
  auto setFrequency(aris::Size frequency) -> void;
  auto name() -> std::string&;
  auto description() -> std::string&;

  virtual ~SensorBase();
  SensorBase(std::function<SensorData*()> sensor_data_ctor,
             const std::string& name = "sensor_base",
             const std::string& desc = "base class of all sensor",
             bool is_virtual = true, bool activate = true,
             aris::Size frequency = 0);
  SensorBase(const SensorBase& other) = delete;
  SensorBase(SensorBase&& other) = delete;
  SensorBase& operator=(const SensorBase& other) = delete;
  SensorBase& operator=(SensorBase&& other) = delete;

 private:
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;
};

// template <class DataType>
// class SIRE_API VirtualSensor : public SensorBase<DataType> {
//  public:
//   // ×ÓÄ£¿éË÷Òý //
//   auto setControlServer(aris::server::ControlServer*) noexcept -> void;
//   auto controlServer() noexcept -> aris::server::ControlServer*;
//   auto setModelBase(aris::dynamic::ModelBase*) noexcept -> void;
//   auto modelBase() noexcept -> aris::dynamic::ModelBase*;
//   auto setMaster(aris::control::Master*) noexcept -> void;
//   auto master() noexcept -> aris::control::Master*;
//   auto setController(aris::control::Controller*) noexcept -> void;
//   auto controller() noexcept -> aris::control::Controller*;
//   auto virtual init() -> void{};
//
//   virtual ~VirtualSensor();
//   VirtualSensor(const std::string& name = "virtual_sensor");
//   VirtualSensor(VirtualSensor& other);
//   VirtualSensor(VirtualSensor&& other) = delete;
//   VirtualSensor& operator=(const VirtualSensor& other);
//   VirtualSensor& operator=(VirtualSensor&& other) = delete;
//
//  protected:
//   auto virtual release() -> void{};
//   auto virtual updateData(aris::sensor::SensorData& data,
//                           SensorBase::UpdateDataCallback callback = nullptr)
//       -> void{};
//
//  private:
//   aris::dynamic::ModelBase* model_base_{nullptr};
//   aris::control::Master* master_{nullptr};
//   aris::control::Controller* controller_{nullptr};
//   aris::control::EthercatMaster* ec_master_{nullptr};
//   aris::server::ControlServer* cs_{nullptr};
// };
//
// struct MotorForceData : aris::sensor::SensorData {
//   double force_;
//   virtual ~MotorForceData() = default;
//   MotorForceData(double force = 0) : force_(force) {}
// };
// class SIRE_API MotorForceVirtualSensor final
//     : public VirtualSensor<MotorForceData> {
//  public:
//   auto motorIndex() const -> sire::Size;
//   auto setMotorIndex(sire::Size index) -> void;
//   auto frequency() const -> sire::Size;
//   auto setFrequency(sire::Size frequency) -> void;
//   auto bufferSize() const -> sire::Size;
//   auto setBufferSize(sire::Size buffer_size) -> void;
//   auto retrieveBufferData(
//       std::vector<std::unique_ptr<aris::sensor::SensorData>>& vec,
//       sire::Size& count) -> void;
//   void virtual init(aris::server::ControlServer* cs) override;
//   virtual ~MotorForceVirtualSensor();
//   MotorForceVirtualSensor(
//       const std::string& name = "motor_force_virtual_sensor",
//       sire::Size frequency = 10, sire::Size moter_index = 0);
//   MotorForceVirtualSensor(MotorForceVirtualSensor& other);
//   MotorForceVirtualSensor(MotorForceVirtualSensor&& other) = delete;
//   MotorForceVirtualSensor& operator=(const MotorForceVirtualSensor& other);
//   MotorForceVirtualSensor& operator=(MotorForceVirtualSensor&& other) =
//   delete;
//
//  protected:
//   auto release() -> void;
//   void updateData(aris::sensor::SensorData& data,
//                   SensorBase::UpdateDataCallback callback) override;
//
//  private:
//   auto updateBufferData(std::unique_ptr<aris::sensor::SensorData> data) ->
//   void; struct Imp; std::unique_ptr<Imp> imp_;
// };

};  // namespace sire::sensor

#endif