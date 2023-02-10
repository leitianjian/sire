#ifndef SIRE_SENSOR_HPP_
#define SIRE_SENSOR_HPP_

// #include <sire_lib_export.h>
// #include <aris.hpp>
//
// namespace sire::sensor {
// template <class DataType>
// class SIRE_API SensorBase : public aris::sensor::SensorTemplate<DataType> {
//  public:
//   using UpdateDataCallback = aris::sensor::Sensor::UpdateDataCallback;
//   auto isVirtual() const -> bool;
//   auto setVirtual(bool is_virtual) -> void;
//   auto activate() const -> bool;
//   auto setActivate(bool is_activate = true) -> void;
//   auto description() -> std::string& { return description_; };
//
//   virtual ~SensorBase();
//   SensorBase(const std::string& name = "sensor_base", bool is_virtual =
//   false,
//              bool activate = false,
//              const std::string& description = "Virtual sensor base");
//   SensorBase(SensorBase& other);
//   SensorBase(SensorBase&& other) = delete;
//   SensorBase& operator=(const SensorBase& other);
//   SensorBase& operator=(SensorBase&& other) = delete;
//
//  protected:
//   auto virtual init() -> void{};
//   auto virtual release() -> void{};
//   auto virtual updateData(aris::sensor::SensorData& data,
//                           UpdateDataCallback callback = nullptr) -> void{};
//
//  private:
//   bool is_virtual_;
//   bool activate_;
//   std::string description_;
// };
//
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
//   auto motorIndex() const -> aris::Size;
//   auto setMotorIndex(aris::Size index) -> void;
//   auto frequency() const -> aris::Size;
//   auto setFrequency(aris::Size frequency) -> void;
//   auto bufferSize() const -> aris::Size;
//   auto setBufferSize(aris::Size buffer_size) -> void;
//   auto retrieveBufferData(
//       std::vector<std::unique_ptr<aris::sensor::SensorData>>& vec,
//       aris::Size& count) -> void;
//   void virtual init(aris::server::ControlServer* cs) override;
//   virtual ~MotorForceVirtualSensor();
//   MotorForceVirtualSensor(
//       const std::string& name = "motor_force_virtual_sensor",
//       aris::Size frequency = 10, aris::Size moter_index = 0);
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
//
// };  // namespace sire::sensor

#endif