#include "sire/transfer/sire_transfer.hpp"
#include "sire/controller/controller_sensor.hpp"

namespace sire::transfer {
SireTransferModelController::SireTransferModelController() {
  part_pool_length_ = dynamic_cast<aris::dynamic::Model&>(
                          aris::server::ControlServer::instance().model())
                          .partPool()
                          .size();
  motion_pool_length_ = dynamic_cast<aris::dynamic::Model&>(
                          aris::server::ControlServer::instance().model())
                          .motionPool()
                          .size();
  general_motion_pool_length_ = dynamic_cast<aris::dynamic::Model&>(
                            aris::server::ControlServer::instance().model())
                            .generalMotionPool()
                            .size();
  dt_seconds_ = 0.001;
  motion_force_.resize(motion_pool_length_, 0.0);
  parts_ps_array_[0].resize(6 * part_pool_length_, 0.0);
  parts_ps_array_[1].resize(6 * part_pool_length_, 0.0);
  old_parts_ps_ = parts_ps_array_[0].data();
  now_parts_ps_ = parts_ps_array_[1].data();
  for (int i = 0; i < parts_as_array_.size(); ++i) {
    parts_as_array_[i].resize(6 * part_pool_length_, 0.0);
    parts_as_ptrs_[i] = parts_as_array_[i].data();
    parts_vs_array_[i].resize(6 * part_pool_length_, 0.0);
    parts_vs_ptrs_[i] = parts_vs_array_[i].data();
  }
};
auto SireTransferModelController::updateDataController2Model(
    const std::vector<std::uint64_t>& options,
    const aris::control::Controller* controller,
    aris::dynamic::ModelBase* model) -> void {
  // 读取（虚拟）力传感器数据作为关节力矩
  for (std::size_t i = 0; i < controller->sensorPool().size(); ++i) {
    try {
      auto& sensor =
          dynamic_cast<controller::MotorForceVirtualSensor&>(
          const_cast<aris::control::SensorBase&>(controller->sensorPool().at(i)));
      std::unique_ptr<aris::control::SensorData> ptr = sensor.copiedDataPtr();
      auto* force_data = dynamic_cast<controller::MotorForceData*>(ptr.get());
      motion_force_[sensor.motorIndex()] = force_data->force_;
    } catch (std::bad_cast& err) {
      continue;
    }
  }
  // 根据读取到的力做动力学正解
  aris::dynamic::Model* csModel =
      dynamic_cast<aris::dynamic::Model*>(model);
  csModel->setInputFce(motion_force_.data());
  if (csModel->forwardDynamics())
    std::cout << "forward dynamic failed" << std::endl;
  for (std::size_t i = 0; i < part_pool_length_; ++i) {
    auto& part = csModel->partPool()[i];
    std::swap(parts_as_ptrs_[0], parts_as_ptrs_[2]);
    std::swap(parts_as_ptrs_[0], parts_as_ptrs_[1]);
    part.getAs(parts_as_ptrs_[2] + static_cast<long>(6) * i);
  }
  integrateAs2Ps(parts_vs_ptrs_.data(), parts_as_ptrs_.data(), old_parts_ps_, now_parts_ps_);
  for (std::size_t i = 0; i < part_pool_length_; ++i) {
    aris::dynamic::Part& part = csModel->partPool().at(i);
    part.setVs(parts_vs_ptrs_[2]);
    aris::core::Matrix now_pm(4, 4);
    aris::dynamic::s_ps2pm(now_parts_ps_, now_pm.data());
    part.setPm(now_pm.data());
  }
  csModel->forwardKinematics();
  for (std::size_t i = 0; i < motion_pool_length_; ++i) {
    auto& motion = csModel->motionPool().at(i);
    motion.updA();
    motion.updV();
    motion.updP();
    // std::cout << motion.mp() << " ";
  }
  // std::cout << std::endl;
  for (std::size_t i = 0; i < general_motion_pool_length_; ++i) {
    auto& general_motion = csModel->generalMotionPool().at(i);
    general_motion.updA();
    general_motion.updV();
    general_motion.updP();
  }
  // for (std::size_t i = 0; i < controller->motorPool().size(); ++i) {
  //   auto& cm = controller->motorPool()[i];
  //   if ((options[i] & aris::plan::Plan::UPDATE_MODEL_POS_FROM_CONTROLLER))
  //     model->setInputPosAt(cm.targetPos(), i);
  //   if ((options[i] & aris::plan::Plan::UPDATE_MODEL_VEL_FROM_CONTROLLER))
  //     model->setInputVelAt(cm.targetVel(), i);
  // }
  std::swap(old_parts_ps_, now_parts_ps_);
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
      auto& mfvs =
          dynamic_cast<controller::MotorForceVirtualSensor&>(sensor);
      mfvs.updateData(std::make_unique<controller::MotorForceData>(
          model->inputFceAt(mfvs.motorIndex())));
    } catch (std::bad_cast& e) {
      std::cout << "Not a motor force virtual sensor, skip!" << std::endl;
      continue;
    }
  }
}
auto SireTransferModelController::integrateAs2Ps(double* vs_in[3],
                                                 double* as_in[3],
                                                 double* old_ps,
                                                 double* ps_out) -> void {
  std::swap(vs_in[0], vs_in[2]);
  std::swap(vs_in[0], vs_in[1]);
  for (int i = 0; i < part_pool_length_; ++i) {
    for (int j = 0; j < 6; ++j) {
      int temp = i * 6 + j;
      vs_in[2][temp] =
          vs_in[1][temp] +
          dt_seconds_ * (as_in[0][temp] + 4 * as_in[1][temp] + as_in[2][temp]) /
              6.0;
      ps_out[temp] =
          old_ps[temp] +
          dt_seconds_ * (vs_in[0][temp] + 4 * vs_in[1][temp] + vs_in[2][temp]) /
              6.0;
    }
  }
}

ARIS_REGISTRATION {
  aris::core::class_<SireTransferModelController>("SireTransferModelController")
      .inherit<aris::server::TransferModelController>();
}
}  // namespace sire::transfer