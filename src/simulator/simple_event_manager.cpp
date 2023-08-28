#include "sire/simulator/simple_event_manager.hpp"

#include <atomic>
#include <list>
#include <thread>

#include <aris/dynamic/screw.hpp>

#include "sire/core/sire_assert.hpp"
#include "sire/core/sorted_pair.hpp"
#include "sire/physics/contact/contact_pair_manager.hpp"
#include "sire/simulator/simulator.hpp"

namespace sire::simulator {
using core::SortedPair;
using physics::PhysicsEngine;
using physics::common::PenetrationAsPointPair;
using physics::contact::ContactPairManager;
using physics::contact::ContactPairValue;
struct SimpleEventManager::Imp {
  std::list<Event> event_list_;
  std::list<Event>::iterator header_;
  std::list<Event>::iterator next_;

  double current_time_{0.0};
  double next_time_{0.0};
  std::chrono::time_point<std::chrono::high_resolution_clock> begin_time_,
      target_time_;

  std::atomic_bool is_event_manager_running_{false};
  std::thread event_thread;

  // 打洞，读取数据 //
  std::atomic_bool if_get_data_{false}, if_get_data_ready_{false};
  const std::function<void(aris::server::ControlServer&, Simulator&,
                           std::any&)>* get_data_func_{nullptr};
  std::any* get_data_{nullptr};
};
SimpleEventManager::SimpleEventManager()
    : imp_(new Imp),
      dt_(0.001),
      simulator_ptr_(nullptr),
      engine_ptr_(nullptr) {}
SimpleEventManager::~SimpleEventManager() = default;
SIRE_DEFINE_MOVE_CTOR_CPP(SimpleEventManager);

auto SimpleEventManager::init() -> void {
  dt_ = simulator_ptr_->deltaT();
  imp_->header_ = imp_->event_list_.begin();
  imp_->next_ = imp_->event_list_.end();

  imp_->current_time_ = 0.0;
  imp_->next_time_ = 0;
  Event start;
  start.functionalities_.insert(EventFeature::START);
  start.time_ = 0;
  imp_->event_list_.push_back(start);
  imp_->header_ = imp_->event_list_.begin();
}
auto SimpleEventManager::start() -> void {
  imp_->is_event_manager_running_.store(true);
  imp_->event_thread = std::thread([this]() {
    imp_->begin_time_ = imp_->target_time_ =
        std::chrono::high_resolution_clock::now();
    while (imp_->is_event_manager_running_ &&
           imp_->header_ != imp_->event_list_.end()) {
      //
      Event& header = *imp_->header_;
      imp_->current_time_ = header.time_;
      resolveEvent(header);

      if (imp_->if_get_data_.exchange(false)) {
        imp_->get_data_func_->operator()(
            aris::server::ControlServer::instance(), *simulator_ptr_,
            *imp_->get_data_);
        imp_->if_get_data_ready_.store(true);  // 原子操作
      }

      // 重置是否继续循环下一个 next 的 flag
      // 设置这一轮循环的正确的 next，用以处理下面插入的情况
      imp_->next_ = std::next(imp_->header_, 1);
      Event& next = *imp_->next_;
      imp_->next_time_ = next.time_;
      // Step前Model备份
      simulator_ptr_->backupModel();
      step(imp_->next_time_ - imp_->current_time_);
      std::this_thread::sleep_until(
          imp_->begin_time_ +
          10 * std::chrono::duration<double>(imp_->next_time_));
      engine_ptr_->updateGeometryLocationFromModel();
      // 使用碰撞检测结果求解接触问题
      engine_ptr_->handleContact();
      preprocessEvent(next);
      imp_->header_ = imp_->next_;
      imp_->current_time_ = imp_->next_time_;
    }
  });
}

auto SimpleEventManager::getModelState(
    const std::function<void(aris::server::ControlServer&, Simulator&,
                             std::any&)>& get_func,
    std::any& get_data) -> void {
  if (!imp_->is_event_manager_running_) THROW_FILE_LINE("simulator not start");
  imp_->get_data_func_ = &get_func;
  imp_->get_data_ = &get_data;

  imp_->if_get_data_ready_.store(false);
  imp_->if_get_data_.store(true);

  while (!imp_->if_get_data_ready_.load())
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

  imp_->if_get_data_ready_.store(false);
}

auto SimpleEventManager::step(double dt) -> bool {
  return simulator_ptr_->integratorPoolPtr()->at(0).step(dt);
}

auto SimpleEventManager::resolveEvent(Event& e) -> void {
  for (auto it = e.functionalities_.begin(); it != e.functionalities_.end();
       ++it) {
    switch (*it) {
      case EventFeature::START:
        resolveStartEvent(e);
        break;
      case EventFeature::CONTACT_START:
        resolveContactStartEvent(e);
        break;
      case EventFeature::CONTACT_END:
        resolveContactEndEvent(e);
        break;
      case EventFeature::NORMAL_STEP:
        resolveNormalStepEvent(e);
        break;
    }
  }
}

auto SimpleEventManager::preprocessEvent(Event& e) -> bool {
  for (auto it = e.functionalities_.begin(); it != e.functionalities_.end();
       ++it) {
    switch (*it) {
      case EventFeature::CONTACT_START:
        preprocessContactStartEvent(e);
        break;
      case EventFeature::CONTACT_END:
        preprocessContactEndEvent(e);
        break;
      case EventFeature::NORMAL_STEP:
        preprocessNormalStepEvent(e);
        break;
      case EventFeature::END:
        preprocessEndEvent(e);
        break;
    }
  }
  // TODO(leitianjian): need to judge whether continou next;
  return false;
}
auto SimpleEventManager::resolveStartEvent(Event& start_event) -> void {
  // 事件管理器加入下一步长事件
  addNextStepEventAt(imp_->current_time_ + dt_, imp_->header_);
  // 需要主动调用一次碰撞检测，作为整个系统的开始
  engine_ptr_->updateGeometryLocationFromModel();

  std::vector<physics::common::PenetrationAsPointPair> penetrationPairs;
  engine_ptr_->cptPointPairPenetration(penetrationPairs);

  if (penetrationPairs.size() != 0) {
    start_event.functionalities_.insert(EventFeature::CONTACT_START);
    for (auto& penetration : penetrationPairs) {
      core::SortedPair s(penetration.id_A, penetration.id_B);
      // 直接将检测到的碰撞插入contact_start，认为他们都是碰撞开始
      // 此时是仿真开始，没有时刻比现在更早了，
      // 在resolveContactStart中直接对当前进行受力分析与微分方程求解
      start_event.contact_start_pair_.insert({s, {penetration}});
    }
  }
}
auto SimpleEventManager::resolveContactStartEvent(Event& e) -> void {
  // 根据碰撞检测结果进行约束求解，设置约束力并设置约束力大小
}
auto SimpleEventManager::resolveContactEndEvent(Event& e) -> void {}
auto SimpleEventManager::resolveNormalStepEvent(Event& e) -> void {
  addNextStepEventAt(imp_->current_time_ + dt_, imp_->header_, false);
}

auto SimpleEventManager::preprocessContactStartEvent(Event& e) -> void {
  std::vector<physics::common::PenetrationAsPointPair> pairs;
  engine_ptr_->collisionDetection().computePointPairPenetration(pairs);
  // 确定event记录的contact_start_pair是否被记录检测到碰撞
  for (auto& contact_start : e.contact_start_pair_) {
  }

  for (auto& collided_pair : pairs) {
    SortedPair pair(collided_pair.id_A, collided_pair.id_B);
  }

  // 验证接触开始失败
  // if (!engine_ptr_->compareWithRecord()) {
  //   // 删掉对应点对
  //   e->contact_start_pair_.erase();
  //   if (e->contact_start_pair_.size() == 0) {
  //     e->functionalities_.erase(EventFeature::CONTACT_START);
  //   }
  // }
}
auto SimpleEventManager::preprocessContactEndEvent(Event& e) -> void {}
auto SimpleEventManager::preprocessNormalStepEvent(Event& e) -> void {}
auto SimpleEventManager::preprocessEndEvent(Event& e) -> void {}

auto SimpleEventManager::insertEventAt(Event&& e,
                                       std::list<Event>::const_iterator it,
                                       bool reverse) -> void {
  // 这个push的到需要的位置
  std::list<Event>::const_reverse_iterator it_r{it};
  if (reverse) {
    for (; it_r != imp_->event_list_.rend(); ++it_r) {
      if (it->time_ < e.time_) {
        imp_->event_list_.insert(it_r.base(), std::forward<Event>(e));
        break;
      }
    }
    if (it_r == imp_->event_list_.rend()) {
      imp_->event_list_.insert(it_r.base(), std::forward<Event>(e));
    }
  } else {
    for (; it != imp_->event_list_.end(); ++it) {
      if (it->time_ > e.time_) {
        imp_->event_list_.insert(it, std::forward<Event>(e));
        break;
      }
    }
    if (it == imp_->event_list_.end()) {
      imp_->event_list_.insert(it, std::forward<Event>(e));
    }
  }
}
// 事件时间
// 插入位置的搜索起点
// 插入位置的搜索方向
auto SimpleEventManager::addNextStepEventAt(double time,
                                            std::list<Event>::const_iterator it,
                                            bool reverse) -> void {
  Event step;
  step.functionalities_.insert(EventFeature::NORMAL_STEP);
  step.time_ = time;
  // 这个push的到需要的位置
  insertEventAt(std::move(step), it, reverse);
}
auto SimpleEventManager::addPrevContactStartEventAt(
    double time, CollisionResultMap&& collision_result,
    std::list<Event>::const_iterator it, bool reverse) -> void {
  Event contact_start;
  contact_start.functionalities_.insert(EventFeature::CONTACT_START);
  contact_start.time_ = time;
  contact_start.contact_start_pair_.swap(collision_result);
  // 这个push的到需要的位置
  insertEventAt(std::move(contact_start), it, reverse);
}
}  // namespace sire::simulator