#include "sire/integrator/explicit_euler_integrator.hpp"

#include <aris/core/reflection.hpp>
#include <aris/dynamic/model.hpp>

#include "sire/core/constants.hpp"
#include "sire/core/sire_assert.hpp"
#include "sire/integrator/integrator_base.hpp"

namespace sire::simulator {
ExplicitEulerIntegrator::ExplicitEulerIntegrator() : IntegratorBase(){};
auto ExplicitEulerIntegrator::doStep(double dt) -> bool {
  SIRE_ASSERT(model_ptr_ != nullptr);
  SIRE_ASSERT(dt > 0.0);
  if (model_ptr_->forwardDynamics()) {
    std::cout << "forward dynamic failed" << std::endl;
    return false;
  }
  // 对于每个Part，从as积分到vs之后积分到ps，并设置回去
  double as_buffer[6]{0}, vs_buffer[6]{0}, pm_buffer[16]{0}, ps_buffer[6]{0};

  for (sire::Size i = 0; i < part_pool_length_; ++i) {
    auto& part = model_ptr_->partPool()[i];
    part.getAs(as_buffer);
    part.getVs(vs_buffer);
    part.getPm(pm_buffer);
    aris::dynamic::s_pm2ps(pm_buffer, ps_buffer);
    //aris::dynamic::dsp(1, 6, as_buffer);
    for (sire::Size j = 0; j < kTwistSize; ++j) {
      vs_buffer[j] += dt * as_buffer[j];
      ps_buffer[j] += dt * vs_buffer[j];
    }
    aris::dynamic::s_ps2pm(ps_buffer, pm_buffer);
    part.setVs(vs_buffer);
    part.setPm(pm_buffer);
  }

  // 根据更新的杆件相关的信息更新Motion的值
  for (std::size_t i = 0; i < motion_pool_length_; ++i) {
    auto& motion = model_ptr_->motionPool().at(i);
    motion.updA();
    motion.updV();
    motion.updP();
  }
  for (std::size_t i = 0; i < general_motion_pool_length_; ++i) {
    auto& general_motion = model_ptr_->generalMotionPool().at(i);
    general_motion.updA();
    general_motion.updV();
    general_motion.updP();
  }
  // 调整与杆件相关的marker坐标与杆件位姿（最小二乘）
  model_ptr_->forwardKinematics();
  return true;
};
auto ExplicitEulerIntegrator::integrate(double** diff_data_in,
                                        double* old_result, double* result_out)
    -> bool {
  for (int i = 0; i < dataLength(); ++i) {
    result_out[i] = old_result[i] + stepSize() * diff_data_in[0][i];
  }
  return true;
};
ARIS_DEFINE_BIG_FOUR_CPP(ExplicitEulerIntegrator);

ARIS_REGISTRATION {
  aris::core::class_<ExplicitEulerIntegrator>("ExplicitEulerIntegrator")
      .inherit<IntegratorBase>();
}
}  // namespace sire::simulator