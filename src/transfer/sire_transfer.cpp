#include "sire/transfer/sire_transfer.hpp"
#include "sire/controller/controller_sensor.hpp"

namespace sire::transfer {
auto SireTransferModelController::updateDataController2Model(
    const std::vector<std::uint64_t>& options,
    const aris::control::Controller* controller,
    aris::dynamic::ModelBase* model) -> void {
  for (std::size_t i = 0; i < controller->motorPool().size(); ++i) {
    auto& cm = controller->motorPool()[i];
    if ((options[i] & aris::plan::Plan::UPDATE_MODEL_POS_FROM_CONTROLLER))
      model->setInputPosAt(cm.targetPos(), i);
    if ((options[i] & aris::plan::Plan::UPDATE_MODEL_VEL_FROM_CONTROLLER))
      model->setInputVelAt(cm.targetVel(), i);
  }
}
auto SireTransferModelController::updateDataModel2Controller(
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
      auto& mfvs = dynamic_cast<controller::MotorForceVirtualSensor&>(sensor);
      mfvs.updateData(std::make_unique<controller::MotorForceData>(
          model->inputFceAt(mfvs.motorIndex())));
    } catch (std::bad_cast& e) {
      std::cout << "Not a motor force virtual sensor, skip!" << std::endl;
      continue;
    }
  }
}
ARIS_REGISTRATION {
  aris::core::class_<SireTransferModelController>("SireTransferModelController")
      .inherit<aris::server::TransferModelController>();
}
}  // namespace sire::transfer