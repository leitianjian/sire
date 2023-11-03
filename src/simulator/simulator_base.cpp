#include "sire/simulator/simulator_base.hpp"

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
#include "sire/simulator/simple_event_manager.hpp"
#include "sire/simulator/simulator_modules.hpp"

namespace sire::simulator {
using core::TriggerBase, core::EventBase, core::HandlerBase;
using std::map;
struct SimulatorBase::Imp {
  aris::server::ControlServer* control_server_ptr_;
  middleware::SireMiddleware* middleware_ptr_;
  physics::PhysicsEngine* physics_engine_ptr_;
  simulator::SimulatorModules* simulator_modules_ptr_;
  IntegratorPool* integrator_pool_ptr_;
  SensorPool* sensor_pool_ptr_;

  // 用来保存全局变量，使用xml配置，在trigger event handle中可以使用
  core::PropMap global_variable_pool_;
  // Event相关
  // BaseFactory<TriggerBase>* trigger_factory_;
  core::EventBaseFactory* event_factory_;
  core::HandlerBaseFactory* handler_factory_;
  map<sire::Size, std::string> trigger_pool_;
  map<sire::Size, std::string> event_pool_;
  map<sire::Size, std::string> handler_pool_;
  map<sire::Size, std::unique_ptr<TriggerBase> (*)()> trigger_creator_;
  map<sire::Size, std::unique_ptr<EventBase> (*)()> event_creator_;
  map<sire::Size, std::unique_ptr<HandlerBase> (*)()> handler_creator_;
  std::unique_ptr<core::EventManager> event_manager_;

  core::ContactPairManager contact_pair_manager_;

  core::Timer timer_;

  // all time represent in seconds;
  double dt_;
  std::chrono::system_clock::time_point current_time_;
  std::chrono::system_clock::time_point start_time_;
  std::int64_t sim_count_;

  std::atomic_bool is_simulation_running_{false};
  std::thread simulation_thread;
  // std::unique_ptr<aris::dynamic::Model> prev_model_{
  //     std::make_unique<aris::dynamic::Model>()};

  // 打洞，读取数据 //
  std::atomic_bool if_get_data_{false}, if_get_data_ready_{false};
  const std::function<void(aris::server::ControlServer&, SimulatorBase&,
                           std::any&)>* get_data_func_{nullptr};
  std::any* get_data_{nullptr};

  // std::vector<aris::dynamic::Model*> model_pool_{2};
  aris::dynamic::Model* model_ptr_;
  Imp()
      : event_factory_(&core::EventBaseFactory::instance()),
        handler_factory_(&core::HandlerBaseFactory::instance()) {}
};

SimulatorBase::SimulatorBase() : imp_(new Imp) {}
SimulatorBase::~SimulatorBase() = default;
SIRE_DEFINE_MOVE_CTOR_CPP(SimulatorBase);

// 初始化自己掌控的资源和获取挂在其他节点下的资源
// 需要cs.init()之后手动调用，不会被自动调用
auto SimulatorBase::init(middleware::SireMiddleware* middleware) -> void {
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
  // imp_->event_manager_.simulator_ptr_ = this;
  // imp_->event_manager_.engine_ptr_ = imp_->physics_engine_ptr_;

  imp_->timer_.init();
  // imp_->trigger_pool_ = imp_->trigger_factory_->idNamePair();
  // imp_->trigger_creator_ = imp_->trigger_factory_->map();
  imp_->event_pool_ = imp_->event_factory_->idNamePair();
  imp_->event_creator_ = imp_->event_factory_->map();
  imp_->handler_pool_ = imp_->handler_factory_->idNamePair();
  imp_->handler_creator_ = imp_->handler_factory_->map();

  // create init trigger to event manager (must at id = 0)
  // std::unique_ptr<TriggerBase> init_trigger = createTriggerById(0);
  // imp_->event_manager_.addImmediateTrigger(std::move(init_trigger));
  std::unique_ptr<EventBase> init_event = createEventById(0);
  imp_->event_manager_->addEvent(std::move(init_event));
  imp_->event_manager_->init();

  // 正确设置model中的力
  imp_->physics_engine_ptr_->initPartContactForce2Model();
  imp_->model_ptr_->init();
}
auto SimulatorBase::timer() -> core::Timer& { return imp_->timer_; }
auto SimulatorBase::start() -> void {
  imp_->is_simulation_running_.store(true);
  imp_->simulation_thread = std::thread([this]() {
    while (imp_->is_simulation_running_ &&
           imp_->event_manager_->isEventListEmpty()) {
      // Get header event pointer
      core::EventBase* header = imp_->event_manager_->eventListHeader();
      // New handler by event id
      std::unique_ptr<core::HandlerBase> handler =
          createHandlerByEventId(header->eventId());
      handler->init(this);
      if (handler->handle(header)) {
        if (imp_->if_get_data_.exchange(false)) {
          imp_->get_data_func_->operator()(
              aris::server::ControlServer::instance(), *this, *imp_->get_data_);
          imp_->if_get_data_ready_.store(true);  // 原子操作
        }

        imp_->timer_.pauseIfTooFast();
        imp_->event_manager_->headerNextEvent(1);
        imp_->event_manager_->popEventListHeader();
      }
    }
  });
}
auto SimulatorBase::createTriggerById(sire::Size trigger_id)
    -> std::unique_ptr<TriggerBase> {
  std::unique_ptr<TriggerBase> new_trigger =
      imp_->trigger_creator_[trigger_id]();
  new_trigger->setTriggerId(trigger_id);
  new_trigger->setTriggerType(imp_->trigger_pool_[trigger_id]);
  return new_trigger;
}
auto SimulatorBase::createEventById(sire::Size event_id)
    -> std::unique_ptr<EventBase> {
  std::unique_ptr<EventBase> new_event = imp_->event_creator_[event_id]();
  new_event->setEventId(event_id);
  new_event->setEventType(imp_->event_pool_[event_id]);
  return std::move(new_event);
}
auto SimulatorBase::createHandlerById(sire::Size handler_id)
    -> std::unique_ptr<HandlerBase> {
  std::unique_ptr<HandlerBase> new_handler =
      imp_->handler_creator_[handler_id]();
  new_handler->setHandlerId(handler_id);
  new_handler->setHandlerType(imp_->handler_pool_[handler_id]);
  return new_handler;
}
auto SimulatorBase::createEventByTriggerId(sire::Size trigger_id)
    -> std::unique_ptr<EventBase> {
  return createEventById(
      imp_->event_manager_->getEventIdByTriggerId(trigger_id));
}
auto SimulatorBase::createHandlerByEventId(sire::Size event_id)
    -> std::unique_ptr<HandlerBase> {
  return createHandlerById(
      imp_->event_manager_->getHandlerIdByEventId(event_id));
}
inline auto SimulatorBase::model() noexcept -> aris::dynamic::Model* {
  return imp_->model_ptr_;
}
inline auto SimulatorBase::contactPairManager() noexcept
    -> core::ContactPairManager* {
  return &imp_->contact_pair_manager_;
}
auto SimulatorBase::getModelState(
    const std::function<void(aris::server::ControlServer&, SimulatorBase&,
                             std::any&)>& get_func,
    std::any& get_data) -> void {
  if (!imp_->is_simulation_running_) THROW_FILE_LINE("simulator not start");
  imp_->get_data_func_ = &get_func;
  imp_->get_data_ = &get_data;

  imp_->if_get_data_ready_.store(false);
  imp_->if_get_data_.store(true);

  while (!imp_->if_get_data_ready_.load())
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

  imp_->if_get_data_ready_.store(false);
}
// auto SimulatorBase::getModelState(
//     const std::function<void(aris::server::ControlServer&, SimulatorBase&,
//                              std::any&)>& get_func,
//     std::any& get_data) -> void {
//   imp_->event_manager_.getModelState(get_func, get_data);
// }
auto SimulatorBase::backupModel() -> void {
  //  aris::core::fromXmlString(*(imp_->model_pool_[1]),
  //                            aris::core::toXmlString(*(imp_->model_pool_[0])));
}
auto SimulatorBase::restoreModel() -> void {
  //  aris::core::fromJsonFile(*(imp_->model_pool_[0]),
  //                           aris::core::toXmlString(*(imp_->model_pool_[1])));
}
auto SimulatorBase::integrateAs2Ps() -> void {}

auto SimulatorBase::updateSysTime() -> void {
  imp_->current_time_ = std::chrono::system_clock::now();
}
auto SimulatorBase::resetIntegratorPoolPtr(IntegratorPool* pool) -> void {
  imp_->integrator_pool_ptr_ = pool;
}
auto SimulatorBase::integratorPoolPtr() const -> const IntegratorPool* {
  return imp_->integrator_pool_ptr_;
}
auto SimulatorBase::resetSensorPoolPtr(SensorPool* pool) -> void {
  imp_->sensor_pool_ptr_ = pool;
}
auto SimulatorBase::sensorPoolPtr() const -> const SensorPool* {
  return imp_->sensor_pool_ptr_;
}
auto SimulatorBase::resetPhysicsEnginePtr(physics::PhysicsEngine* engine)
    -> void {
  imp_->physics_engine_ptr_ = engine;
}
auto SimulatorBase::physicsEnginePtr() const -> const physics::PhysicsEngine* {
  return imp_->physics_engine_ptr_;
}
auto SimulatorBase::resolveContact() -> void {
  // imp_->physics_engine_ptr_->handleContact();
}
auto SimulatorBase::collisionDetection() -> void {
  imp_->physics_engine_ptr_->hasCollision();
}
auto SimulatorBase::resetEventManager(core::EventManager* manager) -> void {
  imp_->event_manager_.reset(manager);
}
auto SimulatorBase::eventManager() const -> const core::EventManager& {
  return *imp_->event_manager_;
}
auto SimulatorBase::deltaT() -> double { return imp_->dt_; }
auto SimulatorBase::setDeltaT(double delta_t_in) -> void {
  SIRE_ASSERT(delta_t_in >= 0);
  imp_->dt_ = delta_t_in;
}
auto SimulatorBase::realtimeRate() -> double {
  return imp_->timer_.realtimeRate();
}
auto SimulatorBase::setRealtimeRate(double rate) -> void {
  SIRE_ASSERT(rate >= 0);
  imp_->timer_.setRealtimeRate(rate);
}
auto SimulatorBase::setGlobalVariablePool(core::PropMap& pool) -> void {
  imp_->global_variable_pool_ = pool;
}
auto SimulatorBase::getGlobalVariablePool() const -> const core::PropMap& {
  return imp_->global_variable_pool_;
}

ARIS_REGISTRATION {
  auto setGlobalVariablePool = [](SimulatorBase* p, core::PropMap map) -> void {
    p->setGlobalVariablePool(map);
  };
  auto getGlobalVariablePool = [](SimulatorBase* p) -> core::PropMap {
    return p->getGlobalVariablePool();
  };
  typedef core::EventManager& (SimulatorBase::*EventManagerFunc)();

  aris::core::class_<SimulatorBase>("SimulatorBase")
      .prop("dt", &SimulatorBase::setDeltaT, &SimulatorBase::deltaT)
      .prop("realtime_rate", &SimulatorBase::setRealtimeRate,
            &SimulatorBase::realtimeRate)
      .prop("global_variable_pool", &setGlobalVariablePool,
            &getGlobalVariablePool)
      .prop("event_manager", &SimulatorBase::resetEventManager,
            EventManagerFunc(&SimulatorBase::eventManager));
}
}  // namespace sire::simulator