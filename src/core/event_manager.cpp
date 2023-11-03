#include "sire/core/event_manager.hpp"

#include <chrono>
#include <list>
#include <map>
#include <string>
#include <type_traits>
#include <unordered_set>
#include <vector>

#include <aris/core/basic_type.hpp>
#include <aris/core/object.hpp>
#include <aris/core/reflection.hpp>

#include "sire/core/constants.hpp"
#include "sire/core/event_base.hpp"
#include "sire/core/handler_base.hpp"
#include "sire/core/sire_assert.hpp"

namespace sire::core {
using namespace std;
struct EventManager::Imp {
  aris::core::PointerArray<EventHandlerIdPair> trigger_event_pair_pool_;
  aris::core::PointerArray<EventHandlerIdPair> event_handler_pair_pool_;
  map<sire::Size, sire::Size> trigger_event_map_;
  map<sire::Size, sire::Size> event_handler_map_;
  // std::list<std::unique_ptr<TriggerBase>> immediate_trigger_list_;
  // // 不会直接影响event 和
  // handler，需要定时加入once_trigger，当作单次触发器处理
  // std::list<std::unique_ptr<TriggerBase>> conditional_trigger_list_;
  std::list<std::unique_ptr<EventBase>> event_list_;
  std::list<std::unique_ptr<EventBase>>::iterator header_;
};
EventManager::EventManager() : imp_(make_unique<Imp>()) {}
EventManager::~EventManager() = default;
SIRE_DEFINE_MOVE_CTOR_CPP(EventManager);
auto EventManager::init() -> void {
  imp_->header_ = imp_->event_list_.begin();
}
auto EventManager::resetEventHandlerPairPool(
    aris::core::PointerArray<EventHandlerIdPair>* pool) -> void {
  for (auto& id_pair : *pool) {
    imp_->event_handler_map_[id_pair.getEventId()] = id_pair.getHandlerId();
  }
}
auto EventManager::eventHandlerPairPool()
    -> aris::core::PointerArray<EventHandlerIdPair>& {
  imp_->event_handler_pair_pool_.clear();
  for (auto& [key, value] : imp_->event_handler_map_) {
    imp_->event_handler_pair_pool_.push_back(
        std::make_unique<EventHandlerIdPair>(key, value).release());
  }
  return imp_->event_handler_pair_pool_;
}
auto EventManager::getEventIdByTriggerId(sire::Size trigger_id) -> sire::Size {
  return imp_->trigger_event_map_[trigger_id];
}
auto EventManager::getHandlerIdByEventId(sire::Size event_id) -> sire::Size {
  return imp_->event_handler_map_[event_id];
}
auto EventManager::addEvent(std::unique_ptr<EventBase> e) -> void {
  imp_->event_list_.push_back(std::move(e));
}
auto EventManager::isEventListEmpty() -> bool {
  return imp_->header_ != imp_->event_list_.end();
}
auto EventManager::eventListHeader() -> EventBase* {
  return imp_->header_->get();
}
auto EventManager::popEventListHeader() -> void {
  imp_->event_list_.pop_front();
}
auto EventManager::headerNextEvent(sire::Size n) -> void {
  imp_->header_ = std::next(imp_->header_, n);
}
auto EventManager::addImmediateTrigger(
    std::unique_ptr<core::TriggerBase> trigger) -> void {
  // imp_->immediate_trigger_list_.push_back(trigger);
}
auto EventManager::executeAllImmediateTrigger() -> void {
  // for (auto&& trigger : imp_->immediate_trigger_list_) {
  //   trigger->trigger()
  // }
}
ARIS_REGISTRATION {
  aris::core::class_<EventHandlerIdPair>("EventHandlerIdPair")
      .prop("event_id", &EventHandlerIdPair::setEventId,
            &EventHandlerIdPair::getEventId)
      .prop("handler_id", &EventHandlerIdPair::setHandlerId,
            &EventHandlerIdPair::getHandlerId);

  using EventHandlerIdPairPool = aris::core::PointerArray<EventHandlerIdPair>;
  aris::core::class_<EventHandlerIdPairPool>("EventHandlerPairPool")
      .asRefArray();
  typedef EventHandlerIdPairPool& (EventManager::*EventHandlerPairPoolFunc)();

  aris::core::class_<EventManager>("EventManager")
      .prop("event_handler_pair_pool", &EventManager::resetEventHandlerPairPool,
            EventHandlerPairPoolFunc(&EventManager::eventHandlerPairPool));
}
}  // namespace sire::core