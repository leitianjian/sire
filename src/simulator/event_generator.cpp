#include "sire/simulator/event_generator.hpp"

#include "log/easyloggingConfig.hpp"

#include <aris/core/object.hpp>
#include <aris/core/reflection.hpp>
#include <aris/dynamic/model.hpp>
#include <aris/server/control_server.hpp>

#include "sire/core/base_factory.hpp"
#include "sire/core/constants.hpp"
#include "sire/core/module_base.hpp"
#include "sire/core/prop_map.hpp"
#include "sire/core/sire_assert.hpp"
#include "sire/sensor/sensor.hpp"
#include "sire/simulator/simulator_modules.hpp"

namespace sire::simulator {
using core::TriggerBase, core::EventBase, core::HandlerBase;
using std::map;
struct EventGenerator::Imp {
  physics::PhysicsEngine* physics_engine_ptr_;
  simulator::SimulatorModules* simulator_modules_ptr_;
  IntegratorPool* integrator_pool_ptr_;
  SensorPool* sensor_pool_ptr_;
  SimulationLoop* simulationLoopPtr_;

  // 用来保存全局变量，使用xml配置，在trigger event handle中可以使用
  core::PropMap global_variable_pool_;
  // Event相关
  // BaseFactory<TriggerBase>* trigger_factory_;
  core::EventBaseFactory* event_factory_;
  core::HandlerBaseFactory* handler_factory_;
  map<sire::Size, std::string> trigger_pool_;
  map<sire::Size, std::string> event_pool_;
  map<sire::Size, std::string> handler_pool_;
  map<sire::Size, std::unique_ptr<EventBase> (*)()> event_creator_;
  map<sire::Size, std::unique_ptr<HandlerBase> (*)()> handler_creator_;
  std::unique_ptr<simulator::EventManager> event_manager_;

  core::ContactPairManager contact_pair_manager_;

  core::Timer timer_;
  simulator::Recorder recorder_;
  double simDuration_{60};

  // all time represent in seconds;
  double dt_;
  double ctrlt_;
  std::chrono::system_clock::time_point current_time_;
  std::chrono::system_clock::time_point start_time_;
  std::int64_t sim_count_;

  aris::dynamic::Model* model_ptr_;
  Imp()
      : event_factory_(&core::EventBaseFactory::instance()),
        handler_factory_(&core::HandlerBaseFactory::instance()) {}
};

EventGenerator::EventGenerator() : imp_(new Imp) {}
EventGenerator::~EventGenerator() = default;
SIRE_DEFINE_MOVE_CTOR_CPP(EventGenerator);

// 初始化自己掌控的资源和获取挂在其他节点下的资源
// 需要cs.init()之后手动调用，不会被自动调用
auto EventGenerator::init() -> void {
  // imp_->middleware_ptr_ = middleware;
  // SIRE_ASSERT(imp_->middleware_ptr_ != nullptr);
  // imp_->physics_engine_ptr_ = &imp_->middleware_ptr_->physicsEngine();
  // imp_->simulator_modules_ptr_ = &imp_->middleware_ptr_->simulatorModules();
  imp_->integrator_pool_ptr_ = &imp_->simulator_modules_ptr_->integratorPool();
  imp_->sensor_pool_ptr_ = &imp_->simulator_modules_ptr_->sensorPool();

  // 初始化Simulator中的Model指针
  // ControlServer中有一个Model的资源，另一个用来备份的Model由Simulator管理
  imp_->model_ptr_ = &dynamic_cast<aris::dynamic::Model&>(
      aris::server::ControlServer::instance().model());

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

  // 正确设置model中的力
  imp_->physics_engine_ptr_->initPartContactForce2Model();
  imp_->model_ptr_->init();
}
auto EventGenerator::timer() -> core::Timer& { return imp_->timer_; }

auto EventGenerator::generateEvents() -> std::unique_ptr<core::EventBase> {
  double prevCtrlTime = 0;
  double prevSimTime = 0;  // ctrl time 也算prevSimTime
  double nextSuggestTime = 0;
  double dt = 0;
  double ctrlTime = 0;
  double nextCtrlTime = prevCtrlTime + ctrlTime;
  double nextSimTime = prevSimTime + nextSuggestTime;
  std::unique_ptr<core::EventBase> step_event =
      simulationLoopPtr()->eventManager().createEventById(1);
  if (nextSimTime < nextCtrlTime) {
    step_event->eventProp().addProp("dt", nextSimTime);
    simulationLoopPtr()->eventManager().addEvent(std::move(step_event));
    return step_event;
  } else {
    step_event->eventProp().addProp("dt", nextCtrlTime);
    simulationLoopPtr()->eventManager().addEvent(std::move(step_event));
    return step_event;
  }
}

auto EventGenerator::createEventById(sire::Size event_id)
    -> std::unique_ptr<EventBase> {
  std::unique_ptr<EventBase> new_event = imp_->event_creator_[event_id]();
  new_event->setEventId(event_id);
  new_event->setEventType(imp_->event_pool_[event_id]);
  return std::move(new_event);
}
auto EventGenerator::createHandlerById(sire::Size handler_id)
    -> std::unique_ptr<HandlerBase> {
  std::unique_ptr<HandlerBase> new_handler =
      imp_->handler_creator_[handler_id]();
  new_handler->setHandlerId(handler_id);
  new_handler->setHandlerType(imp_->handler_pool_[handler_id]);
  return new_handler;
}
auto EventGenerator::createHandlerByEventId(sire::Size event_id)
    -> std::unique_ptr<HandlerBase> {
  return createHandlerById(
      imp_->event_manager_->getHandlerIdByEventId(event_id));
}
auto EventGenerator::contactPairManager() noexcept
    -> core::ContactPairManager* {
  return &imp_->contact_pair_manager_;
}

auto EventGenerator::integrateAs2Ps() -> void {}

auto EventGenerator::updateSysTime() -> void {
  imp_->current_time_ = std::chrono::system_clock::now();
}
auto EventGenerator::resetSimulationLoopPtr(SimulationLoop* loop) -> void {
  imp_->simulationLoopPtr_ = loop;
}
auto EventGenerator::simulationLoopPtr() const -> const SimulationLoop* {
  return imp_->simulationLoopPtr_;
}
auto EventGenerator::resetIntegratorPoolPtr(IntegratorPool* pool) -> void {
  imp_->integrator_pool_ptr_ = pool;
}
auto EventGenerator::integratorPoolPtr() const -> const IntegratorPool* {
  return imp_->integrator_pool_ptr_;
}
auto EventGenerator::resetSensorPoolPtr(SensorPool* pool) -> void {
  imp_->sensor_pool_ptr_ = pool;
}
auto EventGenerator::sensorPoolPtr() const -> const SensorPool* {
  return imp_->sensor_pool_ptr_;
}
auto EventGenerator::resetPhysicsEnginePtr(physics::PhysicsEngine* engine)
    -> void {
  imp_->physics_engine_ptr_ = engine;
}
auto EventGenerator::physicsEnginePtr() const -> const physics::PhysicsEngine* {
  return imp_->physics_engine_ptr_;
}
auto EventGenerator::collisionDetection() -> void {
  imp_->physics_engine_ptr_->hasCollision();
}
auto EventGenerator::resetEventManager(simulator::EventManager* manager) -> void {
  imp_->event_manager_.reset(manager);
}
auto EventGenerator::eventManager() const -> const simulator::EventManager& {
  return *imp_->event_manager_;
}
auto EventGenerator::targetRealtimeRate() -> double {
  return imp_->timer_.targetRealtimeRate();
}
auto EventGenerator::realtimeRate() -> double {
  return imp_->timer_.realtimeRate();
}
auto EventGenerator::setRealtimeRate(double rate) -> void {
  imp_->timer_.setRealtimeRate(rate);
}
auto EventGenerator::simDuration() -> double { return imp_->simDuration_; }
auto EventGenerator::setSimDuration(double simDuration) -> void {
  imp_->simDuration_ = simDuration;
}
auto EventGenerator::setGlobalVariablePool(core::PropMap& pool) -> void {
  imp_->global_variable_pool_ = pool;
}
auto EventGenerator::getGlobalVariablePool() const -> const core::PropMap& {
  return imp_->global_variable_pool_;
}
auto EventGenerator::reset() -> void {
  imp_->contact_pair_manager_.contactPairMap().clear();
  imp_->timer_.reset();
  imp_->recorder_.reset();
}

ARIS_REGISTRATION {
  auto setGlobalVariablePool = [](EventGenerator* p,
                                  core::PropMap map) -> void {
    p->setGlobalVariablePool(map);
  };
  auto getGlobalVariablePool = [](EventGenerator* p) -> core::PropMap {
    return p->getGlobalVariablePool();
  };
  typedef simulator::EventManager& (EventGenerator::*EventGeneratorFunc)();

  aris::core::class_<EventGenerator>("EventGenerator")
      .prop("realtime_rate", &EventGenerator::setRealtimeRate,
            &EventGenerator::realtimeRate)
      .prop("sim_duration", &EventGenerator::setSimDuration,
            &EventGenerator::simDuration)
      .prop("global_variable_pool", &setGlobalVariablePool,
            &getGlobalVariablePool)
      .prop("event_manager", &EventGenerator::resetEventManager,
            EventGeneratorFunc(&EventGenerator::eventManager));
}
}  // namespace sire::simulator