#include "sire/simulator/simulation_loop.hpp"

#include <typeinfo>

#include <aris/core/object.hpp>
#include <aris/core/reflection.hpp>
#include <aris/dynamic/model.hpp>
#include <aris/server/control_server.hpp>

#include "sire/core/base_factory.hpp"
#include "sire/core/constants.hpp"
#include "sire/core/module_base.hpp"
#include "sire/core/profiler.hpp"
#include "sire/core/prop_map.hpp"
#include "sire/core/sire_assert.hpp"
#include "sire/middleware/sire_middleware.hpp"
#include "sire/sensor/sensor.hpp"
#include "sire/simulator/controller.hpp"
#include "sire/simulator/event_manager.hpp"
#include "sire/simulator/joint_constraint_solver.hpp"
#include "sire/simulator/simulator_modules.hpp"

#include "log/easyloggingConfig.hpp"

namespace sire::simulator {
auto ModelData::initFromModel(const aris::dynamic::Model& model) -> void {
  partSize = model.partPool().size();
  motionSize = model.motionPool().size();
  fcePoolSize = model.forcePool().size();
  partPqVec.resize(partSize);
  partVsVec.resize(partSize);
}
auto ModelData::update(aris::dynamic::Model& model) -> void {
  for (size_t i = 0; i < partSize; ++i) {
    const auto& part = model.partPool()[i];
    part.getPq(partPqVec[i].data());
    part.getVs(partVsVec[i].data());
  }
  for (size_t i = 0; i < fcePoolSize; ++i) {
    auto& fce = model.forcePool()[i];
    if (typeid(fce) == typeid(aris::dynamic::GeneralForce)) {
      generalFceIdx.push_back(i);
      std::array<double, 6> arr;
      std::copy_n(dynamic_cast<aris::dynamic::GeneralForce&>(fce).fce(), 6,
                  arr.begin());
      generalFceVec.push_back(std::move(arr));  // move 可省略，array 是值语义
    } else if (typeid(fce) == typeid(aris::dynamic::SingleComponentForce)) {
      singleCompFceIdx.push_back(i);
      singleCompFceVec.push_back(
          dynamic_cast<aris::dynamic::SingleComponentForce&>(fce).fce());
    }
  }
}
auto ModelData::resetModel(aris::dynamic::Model& model) -> void {
  for (sire::Size i = 0; i < partSize; ++i) {
    model.partPool()[i].setPq(partPqVec[i].data());
    model.partPool()[i].setVs(partVsVec[i].data());
    std::fill_n(const_cast<double*>(model.partPool()[i].as()), 6, 0);
  }
  // TODO: 重要：一定要加这个，要不然会导致初始状态积分存在不一致的情况
  // 单独的Mp不行，Mp,Mv之后效果比较好，好像Mp Mv Ma之后效果才最好，很奇怪。
  for (sire::Size i = 0; i < motionSize; ++i) {
    model.motionPool()[i].setMp(0);
    model.motionPool()[i].setMv(0);
    model.motionPool()[i].setMa(0);
  }
  // double cf[6]{0};
  // for (sire::Size i = 0; i < model.jointPool().size(); ++ i) {
  //   model.jointPool()[i].setCf(cf);
  // }
  sire::Size gfSize{generalFceIdx.size()}, scfSize{singleCompFceIdx.size()};
  for (sire::Size i = 0; i < gfSize; ++i) {
    auto& gf = dynamic_cast<aris::dynamic::GeneralForce&>(
        model.forcePool()[generalFceIdx[i]]);
    gf.setFce(generalFceVec[i].data());
  }
  for (sire::Size i = 0; i < scfSize; ++i) {
    dynamic_cast<aris::dynamic::SingleComponentForce&>(
        model.forcePool()[singleCompFceIdx[i]])
        .setFce(singleCompFceVec[i]);
  }
  for (sire::Size i = gfSize + scfSize; i < model.forcePool().size(); ++i) {
    if (i < gfSize + scfSize + motionSize) {
      dynamic_cast<aris::dynamic::SingleComponentForce&>(model.forcePool()[i])
          .setFce(0);
    } else {
      auto& gf =
          dynamic_cast<aris::dynamic::GeneralForce&>(model.forcePool()[i]);
      std::fill_n(const_cast<double*>(gf.fce()), 6, 0);
    }
  }
}
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
  ModelData init_model_data_;

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
  bool is_init_ctrl_{false};
  bool is_ctrl_flag_{false};

  double prevCtrlTime_{0};
  double prevIntTime_{0};
  double nextSuggestTime_{-1};

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

  imp_->init_model_data_.initFromModel(*imp_->model_ptr_);
  imp_->init_model_data_.update(*imp_->model_ptr_);
  // 正确设置model中的力
  imp_->physics_engine_ptr_->initPartContactForce2Model();
  // std::cout << aris::core::toXmlString(*(imp_->model_ptr_)) << std::endl;
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
auto SimulationLoop::init(aris::dynamic::Model* m, physics::PhysicsEngine* e,
                          simulator::SimulatorModules* sm) -> void {
  imp_->physics_engine_ptr_ = e;
  imp_->integrator_pool_ptr_ = &sm->integratorPool();

  // 初始化Simulator中的Model指针
  // ControlServer中有一个Model的资源，另一个用来备份的Model由Simulator管理
  imp_->model_ptr_ = m;
  imp_->timer_.init();

  imp_->event_manager_->init(this);
  imp_->ctrlPtr_->init(this);
  imp_->is_ctrl_flag_ = imp_->is_init_ctrl_;

  // 很重要，与积分器用的.back().kinPos()相关，去掉这个就要用fk，不用back
  imp_->model_ptr_->solverPool().add<solver::JointConstraintSolver>();
  imp_->init_model_data_.initFromModel(*imp_->model_ptr_);
  imp_->init_model_data_.update(*imp_->model_ptr_);
  // 正确设置model中的力
}
auto SimulationLoop::timer() -> core::Timer& { return imp_->timer_; }
auto SimulationLoop::recorder() -> simulator::Recorder& {
  return imp_->recorder_;
}
auto SimulationLoop::recordsContactCptInfo() -> nlohmann::json {
  return imp_->physics_engine_ptr_->recordsContactCptInfo();
}
auto SimulationLoop::headerIsCtrl() -> bool {
  return imp_->event_manager_->eventListHeader()->eventId() == 2;
}
auto SimulationLoop::integrate() -> bool {
  // Get header event pointer
  core::EventBase* header = imp_->event_manager_->eventListHeader();
  // New handler by event id
  std::unique_ptr<core::HandlerBase> handler =
      createHandlerByEventId(header->eventId());
  handler->init(this);
  imp_->can_get_data_.store(false);
  handler->integrate(header);
  imp_->can_get_data_.store(true);
  return header->eventId() == 2;
  // imp_->can_get_data_.store(false);
  // if (handler->handle(header)) {
  //   imp_->can_get_data_.store(true);
  //   // imp_->event_manager_->generateEvent();
  //   // if (pause_if_fast) {
  //   //   imp_->timer_.pauseIfTooFast();
  //   // }
  //   imp_->event_manager_->headerNextEvent(1);
  //   imp_->event_manager_->popEventListHeader();
  // }
}
auto SimulationLoop::handleContact() -> void {
  // Get header event pointer
  core::EventBase* header = imp_->event_manager_->eventListHeader();
  // New handler by event id
  std::unique_ptr<core::HandlerBase> handler =
      createHandlerByEventId(header->eventId());
  handler->init(this);
  // imp_->can_get_data_.store(false);
  if (handler->handle(header)) {
    // imp_->can_get_data_.store(true);
    // imp_->event_manager_->generateEvent();
    // if (pause_if_fast) {
    //   imp_->timer_.pauseIfTooFast();
    // }
    imp_->event_manager_->headerNextEvent(1);
    imp_->event_manager_->popEventListHeader();
  }
}
auto SimulationLoop::step(sire::Size frame_skip, bool pause_if_fast) -> void {
  for (sire::Size i = 0; i < frame_skip; ++i) {
    // SIRE_PROFILE_FRAME_NAMED("sim/frame");
    SIRE_PROFILE_FRAME();
    // Get header event pointer
    core::EventBase* header = imp_->event_manager_->eventListHeader();
    // New handler by event id
    std::unique_ptr<core::HandlerBase> handler =
        createHandlerByEventId(header->eventId());
    handler->init(this);
    imp_->can_get_data_.store(false);
    {
      SIRE_PROFILE_SCOPE("sim/integrate");
      handler->integrate(header);
    }
    imp_->can_get_data_.store(true);
    bool handled = false;
    {
      SIRE_PROFILE_SCOPE("sim/handle");
      handled = handler->handle(header);
    }
    if (handled) {
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

// ═══════════════════════════════════════════════════════════════════════
//  MuJoCo-style control-timed API — for RL and other fixed-interval
//  controllers that need to integrate through variable-size sub-steps.
// ═══════════════════════════════════════════════════════════════════════
auto SimulationLoop::applyActuators() -> void {
  auto* engine = imp_->physics_engine_ptr_;
  SIRE_ASSERT(engine != nullptr);
  engine->fwdActuators();
}

auto SimulationLoop::stepPhysics() -> double {
  auto* engine = imp_->physics_engine_ptr_;
  SIRE_ASSERT(engine != nullptr);
  engine->resetPartContactForce();

  engine->updateGeometryLocationFromModel();
  // 1. Collision detection
  std::vector<physics::common::PenetrationAsPointPair> pairs;
  {
    SIRE_PROFILE_SCOPE("sim/collisionDetection");
    engine->cptPointPairPenetration(pairs);
  }
  double nextCtrlSimSuggestDt = imp_->event_manager_->cptNextCtrlSimSuggestDt();
  std::vector<physics::common::PointPairContactInfo> contact_info;
  {
    SIRE_PROFILE_SCOPE("sim/contactSolving");
    // process_penetration_depth_and_maintain_impact_set6(this, pairs);
    // TODO(ltj): 关节的控制力怎么进来，控制要怎么写
    engine->integrateByContactInfo(nextCtrlSimSuggestDt, pairs, contact_info);
  }

  SIRE_PROFILE_FRAME();
  return nextCtrlSimSuggestDt;
}

auto SimulationLoop::stepSimple() -> double {
  applyActuators();
  return stepPhysics();
}

auto SimulationLoop::advanceToSimTime(double targetTime)
    -> std::pair<double, sire::Size> {
  double t0 = imp_->timer_.simTime();
  sire::Size sub_steps = 0;

  while (imp_->timer_.simTime() < targetTime - 1e-12) {
    double remaining = targetTime - imp_->timer_.simTime();
    // Clamp the nominal suggestion so we don't shoot past target.
    double original_dt = imp_->dt_;
    double original_ctrl = imp_->ctrlt_;

    double dt_actual = stepPhysics();
    ++sub_steps;

    // Safety: if dt_actual is zero (shouldn't happen), break to avoid
    // infinite loop.
    if (dt_actual <= 1e-14) {
      break;
    }
  }

  return {imp_->timer_.simTime() - t0, sub_steps};
}

// ═══════════════════════════════════════════════════════════════════════
//  stepPhysicsSimple — solver computes forces, loop handles integration
// ═══════════════════════════════════════════════════════════════════════
auto SimulationLoop::stepPhysicsSimple() -> double {
  using namespace physics;
  auto* engine = imp_->physics_engine_ptr_;
  SIRE_ASSERT(engine != nullptr);

  engine->resetPartContactForce();
  engine->updateGeometryLocationFromModel();

  std::vector<common::PenetrationAsPointPair> pairs;
  {
    SIRE_PROFILE_SCOPE("sim/collisionDetection");
    engine->cptPointPairPenetration(pairs);
  }

  double dt_actual;

  if (pairs.empty()) {
    dt_actual = imp_->ctrlt_;
    imp_->integrator_pool_ptr_->at(0).updPs(dt_actual);
  } else {
    // process_penetration_depth_and_maintain_impact_set6(this, pairs);

    std::vector<std::array<double, 16>> T_C_vec;
    std::vector<common::PointPairContactInfo> contact_info;
    double suggest_dt = imp_->ctrlt_;
    double nextCtrlSimSuggestDt =
        imp_->event_manager_->cptNextCtrlSimSuggestDt();
    {
      SIRE_PROFILE_SCOPE("sim/contactSolving");
      engine->integrateByContactInfo(nextCtrlSimSuggestDt, pairs, contact_info);
    }
    imp_->integrator_pool_ptr_->at(0).updPs(dt_actual);
    imp_->recorder_.recordContactInfo(contact_info, 1);
    imp_->recorder_.recordPenetrationPairs(pairs);
  }

  double t = imp_->timer_.updateSimTime(dt_actual);
  imp_->recorder_.addRecord(t);
  imp_->model_ptr_->setTime(t);
  return dt_actual;
}

auto SimulationLoop::cptNextCtrlSimSuggestDt() -> double {
  double nextCtrlSimSuggestDt{-1};
  double nextCtrlTime = imp_->prevCtrlTime_ + imp_->ctrlt_;
  double nextSimTime = imp_->prevIntTime_ + imp_->dt_;
  DLOG(DEBUG) << "next ctrl time " << nextCtrlTime << " next sim time "
              << nextSimTime;
  if (nextCtrlTime < nextSimTime ||
      aris::dynamic::s_is_equal(nextCtrlTime, nextSimTime, 1e-6)) {
    nextCtrlSimSuggestDt =
        nextCtrlTime - imp_->timer_.simTime();
  } else {
    nextCtrlSimSuggestDt =
        nextSimTime - imp_->timer_.simTime();
  }
  return nextCtrlSimSuggestDt;
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
auto SimulationLoop::isInitCtrl() -> bool { return imp_->is_init_ctrl_; }
auto SimulationLoop::setIsInitCtrl(bool isInitCtrl) -> void {
  imp_->is_init_ctrl_ = isInitCtrl;
}
auto SimulationLoop::isCtrlFlag() -> bool { return imp_->is_ctrl_flag_; }
auto SimulationLoop::setIsCtrlFlag(bool isCtrlFlag) -> void {
  imp_->is_ctrl_flag_ = isCtrlFlag;
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
  imp_->contact_pair_manager_.clear();
  imp_->event_manager_->reset();
  imp_->init_model_data_.resetModel(*imp_->model_ptr_);
  imp_->is_ctrl_flag_ = imp_->is_init_ctrl_;
  imp_->timer_.reset();
  imp_->recorder_.reset();
  imp_->prevCtrlTime_ = 0;
  imp_->prevIntTime_ = 0;
  imp_->nextSuggestTime_ = -1;
}
auto SimulationLoop::resetRL() -> void {
  imp_->contact_pair_manager_.clear();
  imp_->event_manager_->reset();
  imp_->is_ctrl_flag_ = imp_->is_init_ctrl_;
  imp_->timer_.reset();
  imp_->prevCtrlTime_ = 0;
  imp_->prevIntTime_ = 0;
  imp_->nextSuggestTime_ = -1;
}
auto SimulationLoop::resetRecorder() -> void {
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
      .prop("isInitCtrl", &SimulationLoop::setIsInitCtrl,
            &SimulationLoop::isInitCtrl)
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