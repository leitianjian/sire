#ifndef SIRE_CONTROLLER_SENSOR_HPP_
#define SIRE_CONTROLLER_SENSOR_HPP_

#include <sire_lib_export.h>

#include <aris/control/controller_motion.hpp>

#include "sire/core/constants.hpp"

namespace sire::controller {
class NrtSensor;

class SIRE_API NrtSensorDataProtector {
 public:
  auto get() const -> const aris::control::SensorData* { return data_; }
  auto data() const -> const aris::control::SensorData& { return *data_; }
  auto operator->() const -> const aris::control::SensorData* { return data_; }
  auto operator*() const -> const aris::control::SensorData& {
    return std::ref(*data_);
  }
  // auto operator=(SensorDataProtector && other)->SensorDataProtector & {
  // std::swap(*this, other); return *this; }

  ~NrtSensorDataProtector() = default;
  NrtSensorDataProtector() : nrt_sensor_(nullptr), data_(nullptr) {}
  NrtSensorDataProtector(NrtSensorDataProtector&& other) = default;

 private:
  explicit NrtSensorDataProtector(NrtSensor* sensor);
  NrtSensorDataProtector(const NrtSensorDataProtector&) = delete;
  NrtSensorDataProtector& operator=(const NrtSensorDataProtector&) = delete;

  NrtSensor* nrt_sensor_;
  const aris::control::SensorData* data_;
  std::unique_lock<std::recursive_mutex> lock_;

  friend class NrtSensor;
};

class SIRE_API NrtSensor : public aris::control::SensorBase {
 public:
  auto virtual copiedDataPtr()
      -> std::unique_ptr<aris::control::SensorData> override;
  auto virtual init() -> void override;
  auto virtual start() -> void override;
  auto virtual stop() -> void override;
  auto dataProtector() -> NrtSensorDataProtector;
  virtual ~NrtSensor();
  NrtSensor(std::function<aris::control::SensorData*()> sensor_data_ctor,
            const std::string& name = "nrt_sensor",
            const std::string& desc = "not real time sensor",
            bool is_virtual = false, bool activate = true,
            sire::Size frequency = 0);
  NrtSensor(const NrtSensor& other) = delete;
  NrtSensor(NrtSensor&& other) = delete;
  NrtSensor& operator=(const NrtSensor& other) = delete;
  NrtSensor& operator=(NrtSensor&& other) = delete;

 protected:
  auto virtual release() -> void = 0;
  auto virtual updateData(aris::control::SensorData& data) -> void = 0;

 private:
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;

  friend class NrtSensorDataProtector;
};

class SIRE_API RtSensor : public aris::control::SensorBase {
 public:
  auto virtual copiedDataPtr()
      -> std::unique_ptr<aris::control::SensorData> = 0;
  auto virtual init() -> void = 0;
  auto virtual start() -> void = 0;
  auto virtual stop() -> void = 0;
  virtual ~RtSensor();
  RtSensor(std::function<aris::control::SensorData*()> sensor_data_ctor,
           const std::string& name = "rt_sensor",
           const std::string& desc = "real time sensor",
           bool is_virtual = false, bool activate = true,
           sire::Size freqency = 0);

 protected:
  auto virtual getRtData(aris::control::SensorData*) -> void = 0;
};

template <class DataType>
class RtSensorTemplate : public RtSensor {
 public:
  virtual ~RtSensorTemplate() = default;
  explicit RtSensorTemplate()
      : RtSensor([]() -> aris::control::SensorData* { return new DataType; }) {}
  RtSensorTemplate(const RtSensorTemplate& other) noexcept = default;
  RtSensorTemplate(RtSensorTemplate&& other) noexcept = default;
  RtSensorTemplate& operator=(const RtSensorTemplate& other) noexcept = default;
  RtSensorTemplate& operator=(RtSensorTemplate&& other) noexcept = default;
};

class SIRE_API SensorDataBuffer {
 public:
  auto virtual updateBufferData(std::unique_ptr<aris::control::SensorData> data)
      -> void;
  auto virtual retrieveBufferData(
      std::vector<std::unique_ptr<aris::control::SensorData>>& vec,
      sire::Size& count) -> void;
  auto virtual retrieveData() -> std::unique_ptr<aris::control::SensorData>;
  auto bufferSize() const -> sire::Size;
  auto setBufferSize(sire::Size buffer_size) -> void;
  virtual ~SensorDataBuffer();
  SensorDataBuffer(sire::Size buffer_size = 50);
  SensorDataBuffer(const SensorDataBuffer& other) = delete;
  SensorDataBuffer(SensorDataBuffer&& other) = delete;
  SensorDataBuffer& operator=(const SensorDataBuffer& other) = delete;
  SensorDataBuffer& operator=(SensorDataBuffer&& other) = delete;

 private:
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;
};

class SIRE_API BufferedSensorInterface {
 protected:
  auto virtual updateBufferData(std::unique_ptr<aris::control::SensorData> data)
      -> void = 0;
  auto virtual retrieveBufferData(
      std::vector<std::unique_ptr<aris::control::SensorData>>& vec,
      sire::Size& count) -> void = 0;
  auto virtual retrieveData() -> std::unique_ptr<aris::control::SensorData> = 0;
};

class SIRE_API BufferedRtSensor : public RtSensor,
                                  public BufferedSensorInterface {
 public:
  enum { MAX_MSG_SIZE = 500 };
  auto virtual copiedDataPtr()
      -> std::unique_ptr<aris::control::SensorData> = 0;
  auto virtual init() -> void{};
  auto virtual start() -> void;
  auto virtual stop() -> void;
  auto virtual updateData(std::unique_ptr<aris::control::SensorData> data)
      -> void;
  auto virtual lockFreeUpdateData(aris::control::SensorData& data) -> void;
  auto bufferSize() const -> sire::Size;
  auto setBufferSize(sire::Size buffer_size) -> void;
  auto updateBufferData(std::unique_ptr<aris::control::SensorData> data)
      -> void override;
  auto retrieveBufferData(
      std::vector<std::unique_ptr<aris::control::SensorData>>& vec,
      sire::Size& count) -> void override;
  auto retrieveData() -> std::unique_ptr<aris::control::SensorData> override;
  virtual ~BufferedRtSensor();
  BufferedRtSensor(std::function<aris::control::SensorData*()> sensor_data_ctor,
                   const std::string& name = "buffered_rt_sensor",
                   const std::string& desc = "real time sensor with buffer",
                   bool is_virtual = false, bool activate = true,
                   sire::Size frequency = 0, sire::Size buffer_size = 50);
  BufferedRtSensor(const BufferedRtSensor& other) = delete;
  BufferedRtSensor(BufferedRtSensor&& other) = delete;
  BufferedRtSensor& operator=(const BufferedRtSensor& other) = delete;
  BufferedRtSensor& operator=(BufferedRtSensor&& other) = delete;

 protected:
  auto virtual getRtData(aris::control::SensorData*) -> void override = 0;

 private:
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;
};

template <class DataType>
class BufferedRtSensorTemplate : public BufferedRtSensor {
 public:
  virtual ~BufferedRtSensorTemplate() = default;
  explicit BufferedRtSensorTemplate()
      : BufferedRtSensor(
            []() -> aris::control::SensorData* { return new DataType; }) {}
  BufferedRtSensorTemplate(const BufferedRtSensorTemplate& other) noexcept =
      default;
  BufferedRtSensorTemplate(BufferedRtSensorTemplate&& other) noexcept = default;
  BufferedRtSensorTemplate& operator=(
      const BufferedRtSensorTemplate& other) noexcept = default;
  BufferedRtSensorTemplate& operator=(
      BufferedRtSensorTemplate&& other) noexcept = default;
};

class SIRE_API MotorForceData : public aris::control::SensorData {
 public:
  double force_;
  virtual ~MotorForceData() = default;
  MotorForceData(double force = 0)
      : aris::control::SensorData(), force_(force){};
  auto virtual to_json_string(std::string& j) -> void override;
  auto virtual from_json_string(const std::string& j) -> void override;
};

class SIRE_API BufferedMotorForceVirtualSensor
    : public BufferedRtSensorTemplate<MotorForceData> {
 public:
  auto motorIndex() const -> sire::Size;
  auto setMotorIndex(sire::Size index) -> void;
  auto virtual copiedDataPtr()
      -> std::unique_ptr<aris::control::SensorData> override;
  auto virtual init() -> void override;
  auto virtual start() -> void override;
  auto virtual stop() -> void override;
  auto virtual updateData(std::unique_ptr<aris::control::SensorData> data)
      -> void override;
  virtual ~BufferedMotorForceVirtualSensor();
  BufferedMotorForceVirtualSensor(sire::Size moter_index = 0);
  BufferedMotorForceVirtualSensor(BufferedMotorForceVirtualSensor& other);
  BufferedMotorForceVirtualSensor(BufferedMotorForceVirtualSensor&& other) =
      delete;
  BufferedMotorForceVirtualSensor& operator=(
      const BufferedMotorForceVirtualSensor& other);
  BufferedMotorForceVirtualSensor& operator=(
      BufferedMotorForceVirtualSensor&& other) = delete;

 protected:
  auto virtual getRtData(aris::control::SensorData*) -> void override;

 private:
  struct Imp;
  std::unique_ptr<Imp> imp_;
};
class SIRE_API MotorForceVirtualSensor
    : public RtSensorTemplate<MotorForceData> {
 public:
  auto motorIndex() const -> sire::Size;
  auto setMotorIndex(sire::Size index) -> void;
  auto virtual copiedDataPtr()
      -> std::unique_ptr<aris::control::SensorData> override;
  auto virtual init() -> void override;
  auto virtual start() -> void override;
  auto virtual stop() -> void override;
  auto virtual updateData(std::unique_ptr<aris::control::SensorData> data)
      -> void;
  virtual ~MotorForceVirtualSensor();
  MotorForceVirtualSensor(sire::Size moter_index = 0);
  MotorForceVirtualSensor(MotorForceVirtualSensor& other);
  MotorForceVirtualSensor(MotorForceVirtualSensor&& other) = delete;
  MotorForceVirtualSensor& operator=(const MotorForceVirtualSensor& other);
  MotorForceVirtualSensor& operator=(MotorForceVirtualSensor&& other) = delete;

 protected:
  auto virtual getRtData(aris::control::SensorData*) -> void override;

 private:
  struct Imp;
  std::unique_ptr<Imp> imp_;
};

};  // namespace sire::controller

#endif