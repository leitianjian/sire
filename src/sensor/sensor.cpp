#include "sire/sensor/sensor.hpp"

#include <aris/core/object.hpp>
#include <aris/core/reflection.hpp>

namespace sire::sensor {
struct SensorBase::Imp {
  std::string name_;
  std::string description_;
  bool activate_;
  bool is_virtual_;
  aris::Size frequency_;
  std::function<SensorData*()> sensor_data_ctor_;
};
auto SensorBase::isVirtual() const -> bool { return imp_->is_virtual_; }
auto SensorBase::setVirtual(bool is_virtual) -> void {
  imp_->is_virtual_ = is_virtual;
}
auto SensorBase::activate() const -> bool { return imp_->activate_; }
auto SensorBase::setActivate(bool is_activate) -> void {
  imp_->activate_ = is_activate;
}
auto SensorBase::frequency() const -> aris::Size { return imp_->frequency_; }
auto SensorBase::setFrequency(aris::Size frequency) -> void {
  imp_->frequency_ = frequency;
}
auto SensorBase::name() -> std::string& { return imp_->name_; }
auto SensorBase::description() -> std::string& { return imp_->description_; }
SensorBase::~SensorBase() = default;
SensorBase::SensorBase(std::function<SensorData*()> sensor_data_ctor,
                       const std::string& name, const std::string& desc,
                       bool is_virtual, bool activate, aris::Size frequency)
    : imp_(new Imp) {
  imp_->sensor_data_ctor_ = sensor_data_ctor;
  imp_->name_ = name;
  imp_->description_ = desc;
  imp_->activate_ = activate;
  imp_->is_virtual_ = is_virtual;
  imp_->frequency_ = frequency;
}

// template <class DataType>
// auto VirtualSensor<DataType>::setControlServer(
//     aris::server::ControlServer* cs) noexcept -> void {
//   cs_ = cs;
// }
// template <class DataType>
// auto VirtualSensor<DataType>::controlServer() noexcept
//     -> aris::server::ControlServer* {
//   return cs_;
// }
// template <class DataType>
// auto VirtualSensor<DataType>::setModelBase(aris::dynamic::ModelBase* m)
// noexcept
//     -> void {
//   model_base_ = m;
// }
// template <class DataType>
// auto VirtualSensor<DataType>::modelBase() noexcept
//     -> aris::dynamic::ModelBase* {
//   return model_base_;
// }
// template <class DataType>
// auto VirtualSensor<DataType>::setMaster(aris::control::Master* m) noexcept
//     -> void {
//   master_ = m;
// }
// template <class DataType>
// auto VirtualSensor<DataType>::master() noexcept -> aris::control::Master* {
//   return master_;
// }
// template <class DataType>
// auto VirtualSensor<DataType>::setController(
//     aris::control::Controller* c) noexcept -> void {
//   controller_ = c;
// }
// template <class DataType>
// auto VirtualSensor<DataType>::controller() noexcept
//     -> aris::control::Controller* {
//   return controller_;
// }
// template <class DataType>
// VirtualSensor<DataType>::~VirtualSensor() = default;
// template <class DataType>
// VirtualSensor<DataType>::VirtualSensor(const std::string& name)
//     : SensorBase<DataType>(name){};
//
// struct MotorForceVirtualSensor::Imp {
//   sire::Size frequency_{10};
//   sire::Size motor_index_{0};
//   sire::Size buffer_size_{50};
//   std::atomic_bool buffer_is_full_;
//   std::atomic_int data_to_read{-1};
//   std::atomic_int data_to_write{0};
//   std::recursive_mutex imp_property_mutex_;
//   std::vector<std::unique_ptr<aris::sensor::SensorData>> buffer_;
//   std::vector<std::recursive_mutex> buffer_mutex_;
//   Imp()
//       : frequency_{10},
//         motor_index_{0},
//         buffer_size_{50},
//         buffer_(buffer_size_),
//         buffer_mutex_(buffer_size_) {}
// };
// auto MotorForceVirtualSensor::motorIndex() const -> sire::Size {
//   return imp_->motor_index_;
// }
// auto MotorForceVirtualSensor::setMotorIndex(sire::Size index) -> void {
//   imp_->motor_index_ = index;
// }
// auto MotorForceVirtualSensor::frequency() const -> sire::Size {
//   return imp_->frequency_;
// }
// auto MotorForceVirtualSensor::setFrequency(sire::Size frequency) -> void {
//   imp_->frequency_ = frequency;
// }
// auto MotorForceVirtualSensor::bufferSize() const -> sire::Size {
//   return imp_->buffer_size_;
// }
// auto MotorForceVirtualSensor::setBufferSize(sire::Size buffer_size) -> void {
//   imp_->buffer_size_ = buffer_size;
// }
//
// auto MotorForceVirtualSensor::init(aris::server::ControlServer* cs) -> void {
//   std::vector<std::recursive_mutex> new_mutex_list(imp_->buffer_size_);
//   imp_->buffer_mutex_.swap(new_mutex_list);
//   imp_->buffer_.resize(imp_->buffer_size_);
//   if (cs) {
//     setControlServer(cs);
//     setModelBase(&cs->model());
//     setMaster(&cs->master());
//     setController(&cs->controller());
//     std::cout << "motor force sensor cs init" << std::endl;
//   } else {
//     std::cout << "virtual sensor cs is null" << std::endl;
//   }
// };
// auto MotorForceVirtualSensor::release() -> void{};
// auto MotorForceVirtualSensor::updateData(
//     aris::sensor::SensorData& data, SensorBase::UpdateDataCallback callback)
//     -> void {
//   auto start = std::chrono::high_resolution_clock::now();
//   auto period_time =
//       std::chrono::duration<double, std::milli>(1000.0 / imp_->frequency_);
//   MotorForceData& mfData = static_cast<MotorForceData&>(data);
//   auto& cs = *controlServer();
//   std::any force = 0.0;
//   if (cs.running()) {
//     cs.getRtData(
//         [this](aris::server::ControlServer& cs, const aris::plan::Plan*
//         target,
//                std::any& data) -> void {
//           auto& force = std::any_cast<double&>(data);
//           auto m = dynamic_cast<aris::dynamic::Model*>(&cs.model());
//           force = m->motionPool()[imp_->motor_index_].mf();
//         },
//         force);
//   }
//   updateBufferData(
//       std::make_unique<MotorForceData>(std::any_cast<double>(force)));
//   mfData.force_ = std::any_cast<double>(force);
//   // std::cout << mfData.force_ << std::endl;
//   std::this_thread::sleep_until(start + period_time);
// };
//
// auto MotorForceVirtualSensor::updateBufferData(
//     std::unique_ptr<aris::sensor::SensorData> data) -> void {
//   std::unique_lock<std::recursive_mutex> lock_property(
//       imp_->imp_property_mutex_, std::defer_lock);
//   std::unique_lock<std::recursive_mutex> lock_data_write(
//       imp_->buffer_mutex_[imp_->data_to_write], std::defer_lock);
//   std::lock(lock_property, lock_data_write);
//   if (imp_->buffer_is_full_) {
//     imp_->buffer_[imp_->data_to_write] = std::move(data);
//     imp_->data_to_read = (imp_->data_to_read + 1) % imp_->buffer_size_;
//     imp_->data_to_write = (imp_->data_to_write + 1) % imp_->buffer_size_;
//   } else {
//     imp_->buffer_[imp_->data_to_write] = std::move(data);
//     imp_->data_to_write = (imp_->data_to_write + 1) % imp_->buffer_size_;
//     if (imp_->data_to_read == imp_->data_to_write) {
//       imp_->buffer_is_full_ = true;
//     }
//     if (imp_->data_to_read == -1) {
//       imp_->data_to_read = 0;
//     }
//   }
// };
//
// auto MotorForceVirtualSensor::retrieveBufferData(
//     std::vector<std::unique_ptr<aris::sensor::SensorData>>& vec,
//     sire::Size& count) -> void {
//   std::cout << vec.size() << std::endl;
//   for (int i = 0; i < vec.size(); ++i) {
//     std::unique_lock<std::recursive_mutex> lock_property(
//         imp_->imp_property_mutex_, std::defer_lock);
//     std::unique_lock<std::recursive_mutex> lock_data_read(
//         imp_->buffer_mutex_[imp_->data_to_read], std::defer_lock);
//     std::lock(lock_property, lock_data_read);
//     if (imp_->buffer_is_full_ || imp_->data_to_read != imp_->data_to_write) {
//       vec[i] = std::move(imp_->buffer_[imp_->data_to_read]);
//       imp_->data_to_read = (imp_->data_to_read + 1) % imp_->buffer_size_;
//       imp_->buffer_is_full_ = false;
//       ++count;
//     } else {
//       break;
//     }
//   }
// }
//
// MotorForceVirtualSensor::~MotorForceVirtualSensor() = default;
// MotorForceVirtualSensor::MotorForceVirtualSensor(const std::string& name,
//                                                  sire::Size frequency,
//                                                  sire::Size motor_index)
//     : VirtualSensor<MotorForceData>(name), imp_(new Imp) {
//   imp_->frequency_ = frequency;
//   imp_->motor_index_ = motor_index;
// }

ARIS_REGISTRATION {

}
};  // namespace sire::sensor
