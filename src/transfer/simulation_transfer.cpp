#include "sire/transfer/simulation_transfer.hpp"

namespace sire::transfer {
SimulationTransfer::SimulationTransfer() : parts_vs_ptrs_(), parts_as_ptrs_() {
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
auto SimulationTransfer::updateDataController2Model(
    const std::vector<std::uint64_t>& options,
    const aris::control::Controller* controller,
    aris::dynamic::ModelBase* model) -> void {
  // 根据电机力输入model的force pool
  for (std::size_t i = 0; i < controller->motorPool().size(); ++i) {
    auto& cm = controller->motorPool()[i];
    dynamic_cast<aris::dynamic::SingleComponentForce&>(
        dynamic_cast<aris::dynamic::Model*>(model)->forcePool().at(i))
        .setFce(cm.targetToq());
  }
  // 根据读取到的力做动力学正解
  aris::dynamic::Model* csModel = dynamic_cast<aris::dynamic::Model*>(model);
  if (csModel->forwardDynamics())
    std::cout << "forward dynamic failed" << std::endl;
  // 读取正解做完的结果（As）
  std::swap(parts_as_ptrs_[0], parts_as_ptrs_[2]);
  std::swap(parts_as_ptrs_[0], parts_as_ptrs_[1]);
  for (std::size_t i = 0; i < part_pool_length_; ++i) {
    auto& part = csModel->partPool()[i];
    part.getAs(parts_as_ptrs_[2] + static_cast<long>(6) * i);
  }
  // As -> Vs -> Ps Runge-Kutta4
  integrateAs2Ps(parts_vs_ptrs_.data(), parts_as_ptrs_.data(), old_parts_ps_,
                 now_parts_ps_);
  // Ps -> Pm -> Parts
  for (std::size_t i = 0; i < part_pool_length_; ++i) {
    aris::dynamic::Part& part = csModel->partPool().at(i);
    part.setVs(parts_vs_ptrs_[2] + 6 * i);
    aris::core::Matrix now_pm(4, 4);
    aris::dynamic::s_ps2pm(now_parts_ps_ + 6 * i, now_pm.data());
    part.setPm(now_pm.data());
  }
  // 根据更新的杆件相关的信息更新Motion的值
  for (std::size_t i = 0; i < motion_pool_length_; ++i) {
    auto& motion = csModel->motionPool().at(i);
    motion.updA();
    motion.updV();
    motion.updP();
  }
  for (std::size_t i = 0; i < general_motion_pool_length_; ++i) {
    auto& general_motion = csModel->generalMotionPool().at(i);
    general_motion.updA();
    general_motion.updV();
    general_motion.updP();
  }
  // 调整与杆件相关的marker坐标与杆件位姿（最小二乘）
  csModel->forwardKinematics();
  std::swap(old_parts_ps_, now_parts_ps_);
}
auto SimulationTransfer::updateDataModel2Controller(
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
}
auto SimulationTransfer::integrateAs2Ps(double* vs_in[3], double* as_in[3],
                                        double* old_ps, double* ps_out)
    -> void {
  std::swap(vs_in[0], vs_in[2]);
  std::swap(vs_in[0], vs_in[1]);
  for (sire::Size i = 0; i < part_pool_length_; ++i) {
    double* as[3] = {as_in[0] + i * 6, as_in[1] + i * 6, as_in[2] + i * 6};
    integrator_->integrate(as, vs_in[1] + i * 6, vs_in[2] + i * 6);
    double* vs[3] = {vs_in[0] + i * 6, vs_in[1] + i * 6, vs_in[2] + i * 6};
    integrator_->integrate(vs, old_ps + i * 6, ps_out + i * 6);
  }
}

auto SimulationTransfer::resetIntegrator(simulator::IntegratorBase* integrator)
    -> void {
  integrator_.reset(integrator);
}
auto SimulationTransfer::integrator() -> simulator::IntegratorBase& {
  return *integrator_;
}
auto SimulationTransfer::integrator() const
    -> const simulator::IntegratorBase& {
  return const_cast<SimulationTransfer*>(this)->integrator();
}

ARIS_REGISTRATION {
  typedef sire::simulator::IntegratorBase& (
      SimulationTransfer::*IntegratorFunc)();
  aris::core::class_<SimulationTransfer>("SimulationTransfer")
      .inherit<aris::server::TransferModelController>()
      .prop("integrator", &SimulationTransfer::resetIntegrator,
            IntegratorFunc(&SimulationTransfer::integrator));
}
}  // namespace sire::transfer