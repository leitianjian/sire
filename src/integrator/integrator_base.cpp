#include "sire/integrator/integrator_base.hpp"

#include <aris/core/reflection.hpp>
#include <aris/server/control_server.hpp>

#include "sire/core/constants.hpp"
#include "sire/core/sire_assert.hpp"
#include "sire/physics/physics_engine.hpp"

namespace sire::simulator {
struct IntegratorBase::Imp {
  double step_size_;
  sire::Size data_length_;
  physics::PhysicsEngine* engine_ptr_{nullptr};
};
auto IntegratorBase::init(physics::PhysicsEngine* engine) -> void {
  SIRE_ASSERT(engine != nullptr);
  imp_->engine_ptr_ = engine;
  model_ptr_ = dynamic_cast<aris::dynamic::Model*>(
      &aris::server::ControlServer::instance().model());
  part_pool_length_ = model_ptr_->partPool().size();
  motion_pool_length_ = model_ptr_->motionPool().size();
  general_motion_pool_length_ = model_ptr_->generalMotionPool().size();
  doInit();
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
IntegratorBase::IntegratorBase() : imp_(new Imp){};
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