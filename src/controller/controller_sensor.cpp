#include "sire/controller/controller_sensor.hpp"

namespace sire::controller {
struct NrtSensor::Imp {
  std::atomic_bool is_stopping_;
  std::thread sensor_thread_;
  std::atomic_int data_to_read_;
  std::recursive_mutex data_mutex_[3];
  std::unique_ptr<aris::control::SensorData> data_[3];

  static auto update_thread(NrtSensor* sensor) -> void {
    std::unique_lock<std::recursive_mutex> lock_(sensor->imp_->data_mutex_[0]);

    while (!sensor->imp_->is_stopping_) {
      for (int i = 0; i < 3; ++i) {
        sensor->imp_->data_to_read_ = (i + 2) % 3;
        sensor->updateData(std::ref(*sensor->imp_->data_[i]));
        std::unique_lock<std::recursive_mutex> nextLock(
            sensor->imp_->data_mutex_[(i + 1) % 3]);
        lock_.swap(nextLock);
      }
    }
  };
};
auto NrtSensor::copiedDataPtr() -> std::unique_ptr<aris::control::SensorData> {
  return std::make_unique<aris::control::SensorData>(dataProtector().data());
}
auto NrtSensor::dataProtector() -> NrtSensorDataProtector {
  return std::move(NrtSensorDataProtector(this));
}
auto NrtSensor::init() -> void{};
auto NrtSensor::start() -> void {
  if (!imp_->sensor_thread_.joinable()) {
    init();

    std::lock(imp_->data_mutex_[0], imp_->data_mutex_[1], imp_->data_mutex_[2]);

    std::unique_lock<std::recursive_mutex> lock1(imp_->data_mutex_[0],
                                                 std::adopt_lock);
    std::unique_lock<std::recursive_mutex> lock2(imp_->data_mutex_[1],
                                                 std::adopt_lock);
    std::unique_lock<std::recursive_mutex> lock3(imp_->data_mutex_[2],
                                                 std::adopt_lock);

    for (auto& d : imp_->data_) {
      this->updateData(std::ref(*d));
    }

    imp_->is_stopping_ = false;
    imp_->data_to_read_ = 2;

    imp_->sensor_thread_ = std::thread(Imp::update_thread, this);
  }
};

auto NrtSensor::stop() -> void {
  if (imp_->sensor_thread_.joinable()) {
    imp_->is_stopping_ = true;
    imp_->sensor_thread_.join();
    release();
  }
};
NrtSensor::~NrtSensor() = default;
NrtSensor::NrtSensor(
    std::function<aris::control::SensorData*()> sensor_data_ctor,
    const std::string& name, const std::string& desc, bool is_virtual,
    bool activate, aris::Size frequency)
    : SensorBase(sensor_data_ctor, name, desc, is_virtual, activate, frequency),
      imp_(new Imp) {}

NrtSensorDataProtector::NrtSensorDataProtector(NrtSensor* nrt_sensor)
    : nrt_sensor_(nrt_sensor), data_(nullptr) {
  //这里data_to_read_指向最新的内存,例如如果data_to_read_为2,那么有两种情况：
  // 1.此时正在写内存0,内存1空闲。
  // 2.在某些极端特殊时刻下,sensor正好刚刚写到内存1,正准备释放dataMutex0,并且之后准备将data_to_read_置为0。
  //无论以上哪种情况,dataMutex2都会被锁住。
  //紧接着以上两种情况,继而会发生以下情况：
  // 1.正在写内存0,内存1空闲,dataMutex2都会被锁住后data_to_read_依然为2,那么此后数据一直在操作内存2,安全。
  // 2.dataMutex2被锁住的同时,data_to_read_被更新到0,此时传感器开始写内存1,由于dataMutex2被锁,因此传感器一直无法
  //更新到内存2；但是数据读取的是内存0,安全。

  do {
    lock_ = std::unique_lock<std::recursive_mutex>(
        nrt_sensor_->imp_->data_mutex_[nrt_sensor->imp_->data_to_read_],
        std::try_to_lock);
  } while (!lock_.owns_lock());

  data_ = nrt_sensor_->imp_->data_[nrt_sensor->imp_->data_to_read_].get();
};

RtSensor::~RtSensor() = default;
RtSensor::RtSensor(std::function<aris::control::SensorData*()> sensor_data_ctor,
                   const std::string& name, const std::string& desc,
                   bool is_virtual, bool activate, aris::Size frequency)
    : SensorBase(sensor_data_ctor, name, desc, is_virtual, activate,
                 frequency) {}

struct SensorDataBuffer::Imp {
  aris::Size buffer_size_{50};
  std::atomic_bool buffer_is_full_{false};
  std::atomic_int data_to_read{-1};
  std::atomic_int data_to_write{0};
  std::recursive_mutex data_to_read_mutex_;
  std::recursive_mutex data_to_write_mutex_;
  std::vector<std::unique_ptr<aris::control::SensorData>> buffer_;
  std::vector<std::recursive_mutex> buffer_mutex_;
  ~Imp() = default;
  Imp(aris::Size buffer_size = 50)
      : buffer_size_(buffer_size),
        buffer_(buffer_size),
        buffer_mutex_(buffer_size) {}
};
auto SensorDataBuffer::setBufferSize(aris::Size buffer_size) -> void {
  imp_->buffer_size_ = buffer_size;
  std::vector<std::recursive_mutex> new_mutex_list(imp_->buffer_size_);
  imp_->buffer_mutex_.swap(new_mutex_list);
  imp_->buffer_.resize(imp_->buffer_size_);
}
auto SensorDataBuffer::bufferSize() const -> aris::Size {
  return imp_->buffer_size_;
}
auto SensorDataBuffer::updateBufferData(
    std::unique_ptr<aris::control::SensorData> data) -> void {
  std::unique_lock<std::recursive_mutex> data_to_read_lock(
      imp_->data_to_read_mutex_, std::defer_lock);
  std::unique_lock<std::recursive_mutex> data_to_write_lock(
      imp_->data_to_write_mutex_, std::defer_lock);
  std::lock(data_to_read_lock, data_to_write_lock);
  std::unique_lock<std::recursive_mutex> lock_data_write(
      imp_->buffer_mutex_[imp_->data_to_write]);
  if (imp_->buffer_is_full_) {
    imp_->buffer_[imp_->data_to_write] = std::move(data);
    imp_->data_to_read = (imp_->data_to_read + 1) % imp_->buffer_size_;
    imp_->data_to_write = (imp_->data_to_write + 1) % imp_->buffer_size_;
  } else {
    imp_->buffer_[imp_->data_to_write] = std::move(data);
    imp_->data_to_write = (imp_->data_to_write + 1) % imp_->buffer_size_;
    if (imp_->data_to_read == imp_->data_to_write) {
      imp_->buffer_is_full_ = true;
    }
    if (imp_->data_to_read == -1) {
      imp_->data_to_read = 0;
    }
  }
};
auto SensorDataBuffer::retrieveBufferData(
    std::vector<std::unique_ptr<aris::control::SensorData>>& vec,
    aris::Size& count) -> void {
  for (int i = 0; i < vec.size(); ++i) {
    // 这个上锁的方式有问题
    std::unique_lock<std::recursive_mutex> data_to_read_lock(
        imp_->data_to_read_mutex_, std::defer_lock);
    std::unique_lock<std::recursive_mutex> data_to_write_lock(
        imp_->data_to_write_mutex_, std::defer_lock);
    std::lock(data_to_read_lock, data_to_write_lock);
    std::unique_lock<std::recursive_mutex> lock_data_read(
        imp_->buffer_mutex_[imp_->data_to_read]);
    if (imp_->buffer_is_full_ || imp_->data_to_read != imp_->data_to_write) {
      vec[i] = std::move(imp_->buffer_[imp_->data_to_read]);
      imp_->data_to_read = (imp_->data_to_read + 1) % imp_->buffer_size_;
      imp_->buffer_is_full_ = false;
      ++count;
    } else {
      break;
    }
  }
}
auto SensorDataBuffer::retrieveData()
    -> std::unique_ptr<aris::control::SensorData> {
  std::unique_lock<std::recursive_mutex> data_to_read_lock(
      imp_->data_to_read_mutex_, std::defer_lock);
  std::unique_lock<std::recursive_mutex> data_to_write_lock(
      imp_->data_to_write_mutex_, std::defer_lock);
  std::lock(data_to_read_lock, data_to_write_lock);
  std::unique_lock<std::recursive_mutex> lock_data_read(
      imp_->buffer_mutex_[(imp_->data_to_read + imp_->buffer_size_ - 1) %
                          imp_->buffer_size_]);
  if (imp_->data_to_read != imp_->data_to_write) {
    std::unique_ptr<aris::control::SensorData> ptr(
        new aris::control::SensorData{
            *imp_->buffer_[imp_->data_to_write - aris::Size(1)]});
    return std::move(ptr);
  }
  return nullptr;
}
SensorDataBuffer::~SensorDataBuffer() = default;
SensorDataBuffer::SensorDataBuffer(aris::Size buffer_size)
    : imp_(new Imp(buffer_size)) {}

struct BufferedRtSensor::Imp {
  std::unique_ptr<SensorDataBuffer> buffer_;
};
auto BufferedRtSensor::setBufferSize(aris::Size buffer_size) -> void {
  imp_->buffer_->setBufferSize(buffer_size);
}
auto BufferedRtSensor::bufferSize() const -> aris::Size {
  return imp_->buffer_->bufferSize();
}
auto BufferedRtSensor::updateBufferData(
    std::unique_ptr<aris::control::SensorData> data) -> void {
  imp_->buffer_->updateBufferData(std::move(data));
}
auto BufferedRtSensor::retrieveBufferData(
    std::vector<std::unique_ptr<aris::control::SensorData>>& vec,
    aris::Size& count) -> void {
  imp_->buffer_->retrieveBufferData(vec, count);
}
auto BufferedRtSensor::retrieveData()
    -> std::unique_ptr<aris::control::SensorData> {
  return imp_->buffer_->retrieveData();
}
BufferedRtSensor::~BufferedRtSensor() = default;
BufferedRtSensor::BufferedRtSensor(
    std::function<aris::control::SensorData*()> sensor_data_ctor,
    const std::string& name, const std::string& desc, bool is_virtual,
    bool activate, aris::Size frequency, aris::Size buffer_size)
    : RtSensor(sensor_data_ctor, name, desc, is_virtual, activate, frequency),
      imp_(new Imp) {
  imp_->buffer_.reset(new SensorDataBuffer(buffer_size));
};

struct MotorForceVirtualSensor::Imp {
  aris::Size motor_index_{0};
  aris::Size count_{0};
  Imp() : motor_index_(0), count_(0) {}
};
auto MotorForceVirtualSensor::motorIndex() const -> aris::Size {
  return imp_->motor_index_;
}
auto MotorForceVirtualSensor::setMotorIndex(aris::Size index) -> void {
  imp_->motor_index_ = index;
}
auto MotorForceVirtualSensor::init() -> void {}
auto MotorForceVirtualSensor::start() -> void {}
auto MotorForceVirtualSensor::stop() -> void {}
auto MotorForceVirtualSensor::updateData(
    std::unique_ptr<aris::control::SensorData> data) -> void {
  if (frequency() == 0) {
    updateBufferData(std::move(data));
  } else {
    if (imp_->count_ % (1000 / frequency()) == 0) {
      updateBufferData(std::move(data));
    }
    imp_->count_ = (imp_->count_ + 1) % (1000 / frequency());
  }
}
auto MotorForceVirtualSensor::copiedDataPtr()
    -> std::unique_ptr<aris::control::SensorData> {
  return std::move(retrieveData());
}
auto MotorForceVirtualSensor::getRtData(aris::control::SensorData* data)
    -> void{};
MotorForceVirtualSensor::~MotorForceVirtualSensor() = default;
MotorForceVirtualSensor::MotorForceVirtualSensor(aris::Size motor_index)
    : BufferedRtSensorTemplate<MotorForceData>(), imp_(new Imp) {
  imp_->motor_index_ = motor_index;
}

ARIS_REGISTRATION {
  aris::core::class_<NrtSensor>("NrtSensor")
      .inherit<aris::control::SensorBase>();
  aris::core::class_<RtSensor>("RtSensor").inherit<aris::control::SensorBase>();
  aris::core::class_<BufferedRtSensor>("BufferedRtSensor")
      .inherit<RtSensor>()
      .prop("buffer_size", &BufferedRtSensor::setBufferSize,
            &BufferedRtSensor::bufferSize);
  aris::core::class_<MotorForceVirtualSensor>(
      "MotorForceControllerVirtualSensor")
      .inherit<BufferedRtSensor>()
      .prop("motor_index", &MotorForceVirtualSensor::setMotorIndex,
            &MotorForceVirtualSensor::motorIndex);
}
};  // namespace sire::controller
