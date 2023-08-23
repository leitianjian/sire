#include "sire/transfer/part_pq_transfer.hpp"

namespace sire::transfer {
PartPQTransfer::PartPQTransfer() {
  part_pool_length_ = dynamic_cast<aris::dynamic::Model&>(
                          aris::server::ControlServer::instance().model())
                          .partPool()
                          .size();
  motion_pool_length_ = dynamic_cast<aris::dynamic::Model&>(
                            aris::server::ControlServer::instance().model())
                            .motionPool()
                            .size();
  general_motion_pool_length_ =
      dynamic_cast<aris::dynamic::Model&>(
          aris::server::ControlServer::instance().model())
          .generalMotionPool()
          .size();
  parts_pq_.resize(7 * part_pool_length_, 0.0);
  parts_pq_atomic_ptr_.store(nullptr);
};
auto PartPQTransfer::updateDataController2Model(
    const std::vector<std::uint64_t>& options,
    const aris::control::Controller* controller,
    aris::dynamic::ModelBase* model) -> void {
  for (sire::Size i = 0; i < controller->motorPool().size(); ++i) {
    auto& cm = controller->motorPool()[i];
    if ((options[i] & aris::plan::Plan::UPDATE_MODEL_POS_FROM_CONTROLLER))
      model->setInputPosAt(cm.targetPos(), i);
    if ((options[i] & aris::plan::Plan::UPDATE_MODEL_VEL_FROM_CONTROLLER))
      model->setInputVelAt(cm.targetVel(), i);
  }
}
auto PartPQTransfer::updateDataModel2Controller(
    const std::vector<std::uint64_t>&  options,
    const aris::dynamic::ModelBase* model,
    aris::control::Controller* controller) -> void {
  if (!parts_pq_atomic_ptr_.load()) {
    const aris::dynamic::Model* csModel =
        dynamic_cast<const aris::dynamic::Model*>(model);
    double* parts_pq_ptr = parts_pq_.data();
    for (sire::Size i = 0; i < part_pool_length_; ++i) {
      csModel->partPool().at(i).getPq(parts_pq_ptr + 7 * i);
    }
    parts_pq_atomic_ptr_.exchange(parts_pq_ptr);
  }

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
}

inline std::atomic<double*>& PartPQTransfer::getPartsPq() {
  return parts_pq_atomic_ptr_;
}

inline const std::atomic<double*>& PartPQTransfer::getPartsPq() const {
  return parts_pq_atomic_ptr_;
}

inline void PartPQTransfer::setPartsPq(double* ptr) {
  parts_pq_atomic_ptr_.store(ptr);
}

inline double* PartPQTransfer::exchange(double* ptr) {
  return parts_pq_atomic_ptr_.exchange(ptr);
}

ARIS_REGISTRATION {
  typedef sire::simulator::IntegratorBase& (PartPQTransfer::*IntegratorFunc)();
  aris::core::class_<PartPQTransfer>("PartPQTransfer")
      .inherit<aris::server::TransferModelController>();
}
}  // namespace sire::transfer