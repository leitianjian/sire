#include "sire/simulator/simulator.hpp"

#include <aris/core/object.hpp>
#include <aris/core/reflection.hpp>
#include <aris/dynamic/model.hpp>
#include <aris/server/control_server.hpp>

#include "sire/core/constants.hpp"
#include "sire/core/module_base.hpp"
#include "sire/core/sire_assert.hpp"
#include "sire/middleware/sire_middleware.hpp"
#include "sire/physics/contact/contact_pair_manager.hpp"
#include "sire/sensor/sensor.hpp"
#include "sire/simulator/simple_event_manager.hpp"
#include "sire/simulator/simulator_modules.hpp"

namespace sire::simulator {
struct Simulator::Imp {
  aris::server::ControlServer* control_server_ptr_;
  middleware::SireMiddleware* middleware_ptr_;
  physics::PhysicsEngine* physics_engine_ptr_;
  simulator::SimulatorModules* simulator_modules_ptr_;
  IntegratorPool* integrator_pool_ptr_;
  SensorPool* sensor_pool_ptr_;

  SimpleEventManager event_manager_;

  // physics::contact::ContactPairManager contact_pair_manager_;

  // all time represent in seconds;
  double dt_;
  std::chrono::system_clock::time_point current_time_;
  std::chrono::system_clock::time_point start_time_;
  std::int64_t sim_count_;

  // std::unique_ptr<aris::dynamic::Model> prev_model_{
  //     std::make_unique<aris::dynamic::Model>()};

  // std::vector<aris::dynamic::Model*> model_pool_{2};
  aris::dynamic::Model* model_ptr_;
};

Simulator::Simulator() : imp_(new Imp) {}
Simulator::~Simulator() = default;
SIRE_DEFINE_MOVE_CTOR_CPP(Simulator);

// 初始化自己掌控的资源和获取挂在其他节点下的资源
// 需要cs.init()之后手动调用，不会被自动调用
auto Simulator::init(middleware::SireMiddleware* middleware) -> void {
  imp_->middleware_ptr_ = middleware;
  SIRE_ASSERT(imp_->middleware_ptr_ != nullptr);
  imp_->physics_engine_ptr_ = &imp_->middleware_ptr_->physicsEngine();
  imp_->simulator_modules_ptr_ = &imp_->middleware_ptr_->simulatorModules();
  imp_->integrator_pool_ptr_ = &imp_->simulator_modules_ptr_->integratorPool();
  imp_->sensor_pool_ptr_ = &imp_->simulator_modules_ptr_->sensorPool();

  // 初始化Simulator中的Model指针
  // ControlServer中有一个Model的资源，另一个用来备份的Model由Simulator管理
  imp_->control_server_ptr_ = &aris::server::ControlServer::instance();
  imp_->model_ptr_ =
      &dynamic_cast<aris::dynamic::Model&>(imp_->control_server_ptr_->model());

  //  aris::core::fromXmlString(*(imp_->prev_model_),
  //                            aris::core::toXmlString(*(imp_->model_ptr_)));
  //  imp_->prev_model_->init();

  //  imp_->model_pool_[0] = imp_->model_ptr_;
  //  imp_->model_pool_[1] = imp_->prev_model_.get();

  // 初始化Simulator中的资源
  // event manager;
  // contact pair manager;
  imp_->event_manager_.simulator_ptr_ = this;
  imp_->event_manager_.engine_ptr_ = imp_->physics_engine_ptr_;

  imp_->event_manager_.init();
  // imp_->contact_pair_manager_.init();

  // 正确设置model中的力
  imp_->physics_engine_ptr_->initPartContactForce2Model();
  imp_->model_ptr_->init();
}

auto Simulator::start() -> void { imp_->event_manager_.start(); }
inline auto Simulator::model() noexcept -> aris::dynamic::Model* {
  return imp_->model_ptr_;
}
auto Simulator::getModelState(
    const std::function<void(aris::server::ControlServer&, Simulator&,
                             std::any&)>& get_func,
    std::any& get_data) -> void {
  imp_->event_manager_.getModelState(get_func, get_data);
}
auto Simulator::backupModel() -> void {
  //  aris::core::fromXmlString(*(imp_->model_pool_[1]),
  //                            aris::core::toXmlString(*(imp_->model_pool_[0])));
}
auto Simulator::restoreModel() -> void {
  //  aris::core::fromJsonFile(*(imp_->model_pool_[0]),
  //                           aris::core::toXmlString(*(imp_->model_pool_[1])));
}
auto Simulator::integrateAs2Ps() -> void {}

auto Simulator::updateSysTime() -> void {
  imp_->current_time_ = std::chrono::system_clock::now();
}
auto Simulator::resetIntegratorPoolPtr(IntegratorPool* pool) -> void {
  imp_->integrator_pool_ptr_ = pool;
}
auto Simulator::integratorPoolPtr() const -> const IntegratorPool* {
  return imp_->integrator_pool_ptr_;
}
auto Simulator::resetSensorPoolPtr(SensorPool* pool) -> void {
  imp_->sensor_pool_ptr_ = pool;
}
auto Simulator::sensorPoolPtr() const -> const SensorPool* {
  return imp_->sensor_pool_ptr_;
}
auto Simulator::resetPhysicsEnginePtr(physics::PhysicsEngine* engine) -> void {
  imp_->physics_engine_ptr_ = engine;
}
auto Simulator::physicsEnginePtr() const -> const physics::PhysicsEngine* {
  return imp_->physics_engine_ptr_;
}
auto Simulator::resolveContact() -> void {
  // imp_->physics_engine_ptr_->handleContact();
}
auto Simulator::collisionDetection() -> void {
  imp_->physics_engine_ptr_->hasCollision();
}

auto Simulator::deltaT() -> double { return imp_->dt_; }

auto Simulator::setDeltaT(double delta_t_in) -> void {
  SIRE_ASSERT(delta_t_in >= 0);
  imp_->dt_ = delta_t_in;
}

ARIS_REGISTRATION {
  aris::core::class_<Simulator>("SireSimulator")
      .prop("dt", &Simulator::setDeltaT, &Simulator::deltaT);
}
}  // namespace sire::simulator