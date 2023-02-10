#include "sire/transfer/virtual_force_sensor_transfer.hpp"

#include "sire/controller/controller_sensor.hpp"

namespace sire::transfer {
auto VirtualForceSensorTransfer::updateDataController2Model(
    const std::vector<std::uint64_t>& options,
    const aris::control::Controller* controller,
    aris::dynamic::ModelBase* model) -> void {
  // 读取（虚拟）力传感器数据作为关节力矩
  // for (std::size_t i = 0; i < controller->sensorPool().size(); ++i) {
  //   try {
  //     auto& sensor =
  //         dynamic_cast<controller::MotorForceVirtualSensor&>(
  //         const_cast<aris::control::SensorBase&>(controller->sensorPool().at(i)));
  //     std::unique_ptr<aris::control::SensorData> ptr =
  //     sensor.copiedDataPtr(); auto* force_data =
  //     dynamic_cast<controller::MotorForceData*>(ptr.get());
  //     motion_force_[sensor.motorIndex()] = force_data->force_;
  //   } catch (std::bad_cast& err) {
  //     continue;
  //   }
  // }
  for (std::size_t i = 0; i < controller->motorPool().size(); ++i) {
    auto& cm = controller->motorPool()[i];
    if ((options[i] & aris::plan::Plan::UPDATE_MODEL_POS_FROM_CONTROLLER))
      model->setInputPosAt(cm.targetPos(), i);
    if ((options[i] & aris::plan::Plan::UPDATE_MODEL_VEL_FROM_CONTROLLER))
      model->setInputVelAt(cm.targetVel(), i);
  }
}
auto VirtualForceSensorTransfer::updateDataModel2Controller(
    const std::vector<std::uint64_t>& options,
    const aris::dynamic::ModelBase* model,
    aris::control::Controller* controller) -> void {
  for (std::size_t i = 0; i < controller->motorPool().size(); ++i) {
    auto& cm = controller->motorPool()[i];
    if ((options[i] & aris::plan::Plan::USE_TARGET_POS))
      cm.setTargetPos(model->inputPosAt(i));
    if ((options[i] & aris::plan::Plan::USE_TARGET_VEL))
      cm.setTargetVel(model->inputVelAt(i));
    if ((options[i] & aris::plan::Plan::USE_TARGET_TOQ))
      cm.setTargetToq(model->inputFceAt(i));
    if ((options[i] & aris::plan::Plan::USE_OFFSET_VEL))
      cm.setOffsetVel(model->inputVelAt(i));
    if ((options[i] & aris::plan::Plan::USE_OFFSET_TOQ))
      cm.setOffsetToq(model->inputFceAt(i));
  }
  for (std::size_t i = 0; i < controller->sensorPool().size(); ++i) {
    auto& sensor = controller->sensorPool()[i];
    try {
      auto& mfvs =
          dynamic_cast<controller::BufferedMotorForceVirtualSensor&>(sensor);
      mfvs.lockFreeUpdateData(
          controller::MotorForceData(model->inputFceAt(mfvs.motorIndex())));
    } catch (std::bad_cast& e) {
      std::cout << "Not a motor force virtual sensor, skip!" << std::endl;
      continue;
    }
  }
}
ARIS_REGISTRATION {
  aris::core::class_<VirtualForceSensorTransfer>("VirtualForceSensorTransfer")
      .inherit<aris::server::TransferModelController>();
}
}  // namespace sire::transfer