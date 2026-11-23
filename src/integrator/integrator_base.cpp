#include "sire/integrator/integrator_base.hpp"

#include <aris/core/reflection.hpp>
#include <aris/server/control_server.hpp>

#include "sire/core/constants.hpp"
#include "sire/core/sire_assert.hpp"
#include "sire/physics/physics_engine.hpp"

namespace sire::simulator {
struct IntegratorBase::Imp {
  double step_size_{0.001};
  sire::Size data_length_{0};
  physics::PhysicsEngine* engine_ptr_{nullptr};
};
auto IntegratorBase::init(physics::PhysicsEngine* engine) -> void {
  SIRE_ASSERT(engine != nullptr);
  imp_->engine_ptr_ = engine;
  model_ptr_ = engine->currentModel();
  part_pool_length_ = model_ptr_->partPool().size();
  motion_pool_length_ = model_ptr_->motionPool().size();
  general_motion_pool_length_ = model_ptr_->generalMotionPool().size();
  doInit();
};
auto IntegratorBase::doStep(double dt) -> bool {
  SIRE_ASSERT(model_ptr_ != nullptr);
  SIRE_ASSERT(dt > 0.0);
  // if (model_ptr_->forwardKinematics()) {
  //   std::cout << "forward kinematics failed" << std::endl;
  //   return false;
  // }

  // for (std::size_t i = 0; i < motion_pool_length_; ++i) {
  //   auto& motion = model_ptr_->motionPool().at(i);
  //   // std::cout << i << " " << motion.mp() << " " << motion.mv() << " "
  //   //           << motion.ma() << " ";
  //   std::cout << i << " " << motion.mp() << " " << motion.mv() << " "
  //             << motion.ma() << " " << std::endl;
  //   motion.updP();
  //   motion.updV();
  //   motion.updA();
  //   std::cout << i << " " << motion.mp() << " " << motion.mv() << " "
  //             << motion.ma() << " " << std::endl;
  //   // std::cout << motion.mp() << " " << motion.mv() << " " << motion.ma()
  //   //           << std::endl;
  // }
  // aris::dynamic::dsp(1, 6, model_ptr_->partPool()[4].as());
  // aris::dynamic::dsp(1, 6, model_ptr_->partPool()[4].vs());
  // aris::dynamic::dsp(1, 16, *model_ptr_->partPool()[4].pm());
  // for (std::size_t i = 0; i < motion_pool_length_; ++i) {
  //   auto& motion = model_ptr_->motionPool().at(i);

  //   // std::cout << motion.mp() << " " << motion.mv() << " " << motion.ma()
  //   //           << std::endl;
  // }
  // if (model_ptr_->forwardKinematicsVel()) {
  //   std::cout << "forward kinematics velocity failed" << std::endl;
  //   return false;
  // }
  // if (model_ptr_->forwardKinematicsAcc()) {
  //   std::cout << "forward kinematics Accel failed" << std::endl;
  //   return false;
  // }

  // aris::dynamic::dsp(1, 6, model_ptr_->partPool()[4].as());
  // aris::dynamic::dsp(1, 6, model_ptr_->partPool()[4].vs());
  // aris::dynamic::dsp(1, 16, *model_ptr_->partPool()[4].pm());
  // for (std::size_t i = 0; i < motion_pool_length_; ++i) {
  //   auto& motion = model_ptr_->motionPool().at(i);
  //   // std::cout << i << " " << motion.mp() << " " << motion.mv() << " "
  //   //           << motion.ma() << " ";
  //   // std::cout << i << " " << motion.mp() << " " << motion.mv() << " "
  //   //           << motion.ma() << " " << std::endl;
  //   motion.updP();
  //   motion.updV();
  //   motion.updA();
  //   // motion.setMf(0);
  //   // std::cout << i << " " << motion.mp() << " " << motion.mv() << " "
  //   //           << motion.ma() << " " << std::endl;
  //   // std::cout << motion.mp() << " " << motion.mv() << " " << motion.ma()
  //   //           << std::endl;
  // }
  // if (model_ptr_->inverseDynamics()) {
  //   std::cout << "inverse dynamics failed" << std::endl;
  //   return false;
  // }
  model_ptr_->solverPool()[1].kinPos();
  model_ptr_->solverPool()[1].kinVel();
  model_ptr_->solverPool()[2].dynAccAndFce();
  // for (std::size_t i = 0; i < motion_pool_length_; ++i) {
  //   auto& motion = model_ptr_->motionPool().at(i);
  //   // std::cout << *motion.f() << std::endl;
  //   std::cout << i << " " << motion.mp() << " " << motion.mv() << " "
  //             << motion.ma() << " " << motion.mf() << std::endl;
  // }
  // for (std::size_t i = 0; i < motion_pool_length_; ++i) {
  //   auto& motion = model_ptr_->motionPool().at(i);
  //   // std::cout << i << " " << motion.mp() << " " << motion.mv() << " "
  //   //           << motion.ma() << " ";
  //   motion.updA();
  //   // std::cout << motion.mp() << " " << motion.mv() << " " << motion.ma()
  //   //           << std::endl;
  // }
  return true;
};
auto IntegratorBase::step(double dt) -> bool { return doStep(dt); }
auto IntegratorBase::stepSize() const -> double { return imp_->step_size_; };
auto IntegratorBase::setStepSize(double step_size) -> void {
  imp_->step_size_ = step_size;
};
auto IntegratorBase::dataLength() const -> sire::Size {
  return imp_->data_length_;
};
auto IntegratorBase::setDataLength(sire::Size data_length) -> void {
  imp_->data_length_ = data_length;
};
IntegratorBase::IntegratorBase() : imp_(new Imp) {};
IntegratorBase::~IntegratorBase() = default;
ARIS_DEFINE_BIG_FOUR_CPP(IntegratorBase);

ARIS_REGISTRATION {
  aris::core::class_<IntegratorBase>("IntegratorBase")
      .prop("step_size", &IntegratorBase::setStepSize,
            &IntegratorBase::stepSize)
      .prop("data_length", &IntegratorBase::setDataLength,
            &IntegratorBase::dataLength);
}
}  // namespace sire::simulator