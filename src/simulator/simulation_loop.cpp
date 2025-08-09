#include "sire/simulator/simulation_loop.hpp"

#include <aris/core/object.hpp>
#include <aris/core/reflection.hpp>
#include <aris/dynamic/model.hpp>
#include <aris/server/control_server.hpp>

#include "sire/core/base_factory.hpp"
#include "sire/core/constants.hpp"
#include "sire/core/module_base.hpp"
#include "sire/core/prop_map.hpp"
#include "sire/core/sire_assert.hpp"
#include "sire/middleware/sire_middleware.hpp"
#include "sire/sensor/sensor.hpp"
#include "sire/simulator/controller.hpp"
#include "sire/simulator/event_manager.hpp"
#include "sire/simulator/joint_constraint_solver.hpp"
#include "sire/simulator/simulator_modules.hpp"

namespace sire::simulator {
using core::TriggerBase, core::EventBase, core::HandlerBase;
using std::map;
struct SimulationLoop::Imp {
  middleware::SireMiddleware* middleware_ptr_;
  physics::PhysicsEngine* physics_engine_ptr_;
  simulator::SimulatorModules* simulator_modules_ptr_;
  IntegratorPool* integrator_pool_ptr_;
  SensorPool* sensor_pool_ptr_;
  // std::vector<aris::dynamic::Model*> model_pool_{2};
  aris::dynamic::Model* model_ptr_;

  // 用来保存全局变量，使用xml配置，在trigger event handle中可以使用
  core::PropMap global_variable_pool_;
  std::unique_ptr<simulator::EventManager> event_manager_{
      new simulator::EventManager()};
  std::unique_ptr<Controller> ctrlPtr_{new Controller()};

  core::ContactPairManager contact_pair_manager_;

  core::Timer timer_;
  simulator::Recorder recorder_;
  double simDuration_{60};

  // all time represent in seconds;
  double dt_{0.001};
  double ctrlt_{0.001};
  std::chrono::system_clock::time_point current_time_;
  std::chrono::system_clock::time_point start_time_;
  std::int64_t sim_count_;

  std::atomic_bool is_simulation_running_{false};
  std::thread simulation_thread;

  std::atomic_bool is_data_fetch_running_{false};
  std::thread data_fetch_thread;
  std::atomic_bool can_get_data_{true};
  // std::unique_ptr<aris::dynamic::Model> prev_model_{
  //     std::make_unique<aris::dynamic::Model>()};

  // 打洞，读取数据 //
  std::atomic_bool if_get_data_{false}, if_get_data_ready_{false};
  const std::function<void(aris::server::ControlServer&, SimulationLoop&,
                           std::any&)>* get_data_func_{nullptr};
  std::any* get_data_{nullptr};
};

SimulationLoop::SimulationLoop() : imp_(new Imp) {}
SimulationLoop::~SimulationLoop() {
  imp_->is_data_fetch_running_.store(false);
  if (imp_->data_fetch_thread.joinable()) {
    imp_->data_fetch_thread.join();
  }
  imp_->is_simulation_running_.store(false);
  if (imp_->simulation_thread.joinable()) {
    imp_->simulation_thread.join();
  }
}
SIRE_DEFINE_MOVE_CTOR_CPP(SimulationLoop);

// 初始化自己掌控的资源和获取挂在其他节点下的资源
// 需要cs.init()之后手动调用，不会被自动调用
auto SimulationLoop::init(middleware::SireMiddleware* middleware) -> void {
  imp_->middleware_ptr_ = middleware;
  SIRE_ASSERT(imp_->middleware_ptr_ != nullptr);
  imp_->physics_engine_ptr_ = &imp_->middleware_ptr_->physicsEngine();
  imp_->simulator_modules_ptr_ = &imp_->middleware_ptr_->simulatorModules();
  imp_->integrator_pool_ptr_ = &imp_->simulator_modules_ptr_->integratorPool();
  imp_->sensor_pool_ptr_ = &imp_->simulator_modules_ptr_->sensorPool();

  // 初始化Simulator中的Model指针
  // ControlServer中有一个Model的资源，另一个用来备份的Model由Simulator管理
  imp_->model_ptr_ = &dynamic_cast<aris::dynamic::Model&>(
      aris::server::ControlServer::instance().model());

  //  aris::core::fromXmlString(*(imp_->prev_model_),
  //                            aris::core::toXmlString(*(imp_->model_ptr_)));
  //  imp_->prev_model_->init();

  //  imp_->model_pool_[0] = imp_->model_ptr_;
  //  imp_->model_pool_[1] = imp_->prev_model_.get();

  // 初始化Simulator中的资源
  // event manager;
  // contact pair manager;
  // imp_->event_manager_.simulator_ptr_ = this;
  // imp_->event_manager_.engine_ptr_ = imp_->physics_engine_ptr_;

  imp_->timer_.init();

  // create init trigger to event manager (must at id = 0)
  // std::unique_ptr<TriggerBase> init_trigger = createTriggerById(0);
  // imp_->event_manager_.addImmediateTrigger(std::move(init_trigger));
  std::unique_ptr<EventBase> init_event =
      this->eventManager().createEventById(0);
  imp_->event_manager_->addEvent(std::move(init_event));
  imp_->event_manager_->init(this);
  imp_->ctrlPtr_->init(this);

  imp_->model_ptr_->solverPool().add<solver::JointConstraintSolver>();

  // 正确设置model中的力
  imp_->physics_engine_ptr_->initPartContactForce2Model();
  std::cout << aris::core::toXmlString(*(imp_->model_ptr_)) << std::endl;
  imp_->model_ptr_->init();
  // double p[1]{0.916};
  // imp_->model_ptr_->motionPool()[2].setP(p);
  // imp_->model_ptr_->motionPool()[5].setP(p);
  // imp_->model_ptr_->motionPool()[8].setP(p);
  // imp_->model_ptr_->motionPool()[11].setP(p);
  // imp_->model_ptr_->forwardKinematics();

  // 开始允许获取数据
  imp_->is_data_fetch_running_.store(true);
  imp_->data_fetch_thread = std::thread([this]() {
    while (imp_->is_data_fetch_running_) {
      while (imp_->is_data_fetch_running_ &&
             !(imp_->if_get_data_.load() && imp_->can_get_data_.load()))
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      if (!imp_->is_data_fetch_running_) break;
      imp_->get_data_func_->operator()(aris::server::ControlServer::instance(),
                                       *this, *imp_->get_data_);
      imp_->if_get_data_ready_.store(true);  // 原子操作
      imp_->if_get_data_.store(false);
    }
  });
}
auto SimulationLoop::timer() -> core::Timer& { return imp_->timer_; }
auto SimulationLoop::recorder() -> simulator::Recorder& {
  return imp_->recorder_;
}
auto SimulationLoop::recordsContactCptInfo() -> void {
  imp_->physics_engine_ptr_->recordsContactCptInfo();
}
auto SimulationLoop::step(sire::Size frame_skip, bool pause_if_fast) -> void {
  for (sire::Size i = 0; i < frame_skip; ++i) {
    // Get header event pointer
    core::EventBase* header = imp_->event_manager_->eventListHeader();
    // New handler by event id
    std::unique_ptr<core::HandlerBase> handler =
        createHandlerByEventId(header->eventId());
    handler->init(this);
    imp_->can_get_data_.store(false);
    if (handler->handle(header)) {
      imp_->can_get_data_.store(true);
      // imp_->event_manager_->generateEvent();
      if (pause_if_fast) {
        imp_->timer_.pauseIfTooFast();
      }
      imp_->event_manager_->headerNextEvent(1);
      imp_->event_manager_->popEventListHeader();
    }
    // double p[6]{0}, v[6]{0}, a[6]{0};
    // model()->generalMotionPool().at(0).getP(p);
    // model()->generalMotionPool().at(0).getV(v);
    // model()->generalMotionPool().at(0).getA(a);
    // std::cout << "step: " << i << std::endl;
    // std::cout << "general_p: ";
    // aris::dynamic::dsp(1, 6, p);
    // aris::dynamic::dsp(1, 6, v);
    // aris::dynamic::dsp(1, 6, a);
    // std::cout << "action: "
    //           << dynamic_cast<aris::dynamic::SingleComponentForce&>(
    //                  model()->forcePool().at(0))
    //                  .fce()
    //           << ", "
    //           << dynamic_cast<aris::dynamic::SingleComponentForce&>(
    //                  model()->forcePool().at(1))
    //                  .fce()
    //           << std::endl;
  }
}
auto SimulationLoop::start() -> void {
  imp_->is_simulation_running_.store(true);
  imp_->simulation_thread = std::thread([this]() {
    while (imp_->is_simulation_running_ &&
           imp_->timer_.simTime() < imp_->simDuration_ &&
           !imp_->event_manager_->isEventListEmpty()) {
      step(1, true);
    }
    std::cout << imp_->recorder_.records.size() << std::endl;
  });
}
auto SimulationLoop::isTimeout() -> bool {
  if (imp_->timer_.simTime() >= imp_->simDuration_ - 1e-8)
    return true;
  else
    return false;
}
auto SimulationLoop::simTime() -> double { return imp_->timer_.simTime(); }
auto SimulationLoop::isEventListEmpty() -> bool {
  return imp_->event_manager_->isEventListEmpty();
}
auto SimulationLoop::isRunning() -> bool {
  return imp_->is_simulation_running_.load();
}
auto SimulationLoop::pause() -> void {
  imp_->is_simulation_running_.store(false);
  imp_->simulation_thread.join();
}
auto SimulationLoop::recordsToJson() -> nlohmann::json {
  nlohmann::json j;
  imp_->recorder_.to_json(j);
  return j;
}
auto SimulationLoop::createHandlerByEventId(sire::Size event_id)
    -> std::unique_ptr<HandlerBase> {
  return this->eventManager().createHandlerById(
      imp_->event_manager_->getHandlerIdByEventId(event_id));
}
auto SimulationLoop::model() noexcept -> aris::dynamic::Model* {
  return imp_->model_ptr_;
}
auto SimulationLoop::contactPairManager() noexcept
    -> core::ContactPairManager* {
  return &imp_->contact_pair_manager_;
}
auto SimulationLoop::getModelState(
    const std::function<void(aris::server::ControlServer&, SimulationLoop&,
                             std::any&)>& get_func,
    std::any& get_data) -> void {
  if (!imp_->is_data_fetch_running_)
    THROW_FILE_LINE("data fetch thread not start");
  imp_->get_data_func_ = &get_func;
  imp_->get_data_ = &get_data;

  imp_->if_get_data_ready_.store(false);
  imp_->if_get_data_.store(true);

  // spin waitting for get data
  while (!imp_->if_get_data_ready_.load())
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

  imp_->if_get_data_ready_.store(false);
}
// auto SimulationLoop::getModelState(
//     const std::function<void(aris::server::ControlServer&, SimulationLoop&,
//                              std::any&)>& get_func,
//     std::any& get_data) -> void {
//   imp_->event_manager_.getModelState(get_func, get_data);
// }
auto SimulationLoop::backupModel() -> void {
  //  aris::core::fromXmlString(*(imp_->model_pool_[1]),
  //                            aris::core::toXmlString(*(imp_->model_pool_[0])));
}
auto SimulationLoop::restoreModel() -> void {
  //  aris::core::fromJsonFile(*(imp_->model_pool_[0]),
  //                           aris::core::toXmlString(*(imp_->model_pool_[1])));
}
auto SimulationLoop::integrateAs2Ps() -> void {}

auto SimulationLoop::updateSysTime() -> void {
  imp_->current_time_ = std::chrono::system_clock::now();
}
auto SimulationLoop::resetIntegratorPoolPtr(IntegratorPool* pool) -> void {
  imp_->integrator_pool_ptr_ = pool;
}
auto SimulationLoop::integratorPoolPtr() const -> const IntegratorPool* {
  return imp_->integrator_pool_ptr_;
}
auto SimulationLoop::resetSensorPoolPtr(SensorPool* pool) -> void {
  imp_->sensor_pool_ptr_ = pool;
}
auto SimulationLoop::sensorPoolPtr() const -> const SensorPool* {
  return imp_->sensor_pool_ptr_;
}
auto SimulationLoop::resetPhysicsEnginePtr(physics::PhysicsEngine* engine)
    -> void {
  imp_->physics_engine_ptr_ = engine;
}
auto SimulationLoop::physicsEnginePtr() const -> const physics::PhysicsEngine* {
  return imp_->physics_engine_ptr_;
}
auto SimulationLoop::resolveContact() -> void {
  // imp_->physics_engine_ptr_->handleContact();
}
auto SimulationLoop::collisionDetection() -> void {
  imp_->physics_engine_ptr_->hasCollision();
}
auto SimulationLoop::resetEventManager(simulator::EventManager* manager)
    -> void {
  imp_->event_manager_.reset(manager);
}
auto SimulationLoop::eventManager() const -> const simulator::EventManager& {
  return *imp_->event_manager_;
}
auto SimulationLoop::resetController(simulator::Controller* ctrlPtr) -> void {
  SIRE_ASSERT(ctrlPtr != nullptr);
  imp_->ctrlPtr_.reset(ctrlPtr);
}
auto SimulationLoop::controller() const -> const simulator::Controller& {
  return *imp_->ctrlPtr_;
}
auto SimulationLoop::deltaT() -> double { return imp_->dt_; }
auto SimulationLoop::setDeltaT(double delta_t_in) -> void {
  SIRE_ASSERT(delta_t_in >= 0);
  imp_->dt_ = delta_t_in;
}
auto SimulationLoop::ctrlT() -> double { return imp_->ctrlt_; }
auto SimulationLoop::setCtrlT(double ctrlt) -> void {
  SIRE_ASSERT(ctrlt >= 0);
  imp_->ctrlt_ = ctrlt;
}
auto SimulationLoop::targetRealtimeRate() -> double {
  return imp_->timer_.targetRealtimeRate();
}
auto SimulationLoop::realtimeRate() -> double {
  return imp_->timer_.realtimeRate();
}
auto SimulationLoop::setRealtimeRate(double rate) -> void {
  imp_->timer_.setRealtimeRate(rate);
}
auto SimulationLoop::simDuration() -> double { return imp_->simDuration_; }
auto SimulationLoop::setSimDuration(double simDuration) -> void {
  imp_->simDuration_ = simDuration;
}
auto SimulationLoop::setGlobalVariablePool(core::PropMap& pool) -> void {
  imp_->global_variable_pool_ = pool;
}
auto SimulationLoop::getGlobalVariablePool() const -> const core::PropMap& {
  return imp_->global_variable_pool_;
}
auto SimulationLoop::reset() -> void {
  imp_->contact_pair_manager_.contactPairMap().clear();
  imp_->timer_.reset();
  imp_->recorder_.reset();
}

ARIS_REGISTRATION {
  auto setGlobalVariablePool = [](SimulationLoop* p,
                                  core::PropMap map) -> void {
    p->setGlobalVariablePool(map);
  };
  auto getGlobalVariablePool = [](SimulationLoop* p) -> core::PropMap {
    return p->getGlobalVariablePool();
  };
  typedef simulator::EventManager& (SimulationLoop::*EventManagerFunc)();
  typedef simulator::Controller& (SimulationLoop::*ControllerFunc)();

  aris::core::class_<SimulationLoop>("SimulationLoop")
      .prop("dt", &SimulationLoop::setDeltaT, &SimulationLoop::deltaT)
      .prop("ctrlt", &SimulationLoop::setCtrlT, &SimulationLoop::ctrlT)
      .prop("realtime_rate", &SimulationLoop::setRealtimeRate,
            &SimulationLoop::targetRealtimeRate)
      .prop("sim_duration", &SimulationLoop::setSimDuration,
            &SimulationLoop::simDuration)
      .prop("global_variable_pool", &setGlobalVariablePool,
            &getGlobalVariablePool)
      .prop("event_manager", &SimulationLoop::resetEventManager,
            EventManagerFunc(&SimulationLoop::eventManager))
      .prop("controller", &SimulationLoop::resetController,
            ControllerFunc(&SimulationLoop::controller));
}
}  // namespace sire::simulator