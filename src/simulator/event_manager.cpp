#include "sire/simulator/event_manager.hpp"

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

#include "sire/core/base_factory.hpp"
#include "sire/core/constants.hpp"
#include "sire/core/event_base.hpp"
#include "sire/core/handler_base.hpp"
#include "sire/core/sire_assert.hpp"
#include "sire/simulator/simulation_loop.hpp"

#include "log/easyloggingConfig.hpp"

namespace sire::simulator {
struct EventManager::Imp {
  SimulationLoop* simulationLoopPtr_;
  // prevCtrlTime <= prevIntTime_
  double prevCtrlTime_{-1};
  double prevIntTime_{-1};
  double nextSuggestTime_{-1};
  sire::Size nextEventId_{1};

  aris::core::PointerArray<EventHandlerIdPair> trigger_event_pair_pool_;
  aris::core::PointerArray<EventHandlerIdPair> event_handler_pair_pool_;
  std::map<sire::Size, sire::Size> trigger_event_map_;
  std::map<sire::Size, sire::Size> event_handler_map_;
  // std::list<std::unique_ptr<TriggerBase>> immediate_trigger_list_;
  // // 不会直接影响event 和
  // handler，需要定时加入once_trigger，当作单次触发器处理
  // std::list<std::unique_ptr<TriggerBase>> conditional_trigger_list_;
  // Event相关
  core::EventBaseFactory* event_factory_;
  core::HandlerBaseFactory* handler_factory_;
  std::map<sire::Size, std::string> event_pool_;
  std::map<sire::Size, std::string> handler_pool_;
  std::map<sire::Size, std::unique_ptr<core::EventBase> (*)()> event_creator_;
  std::map<sire::Size, std::unique_ptr<core::HandlerBase> (*)()>
      handler_creator_;
  std::list<std::unique_ptr<core::EventBase>> event_list_;
  std::list<std::unique_ptr<core::EventBase>>::iterator header_;
  Imp()
      : event_factory_(&core::EventBaseFactory::instance()),
        handler_factory_(&core::HandlerBaseFactory::instance()) {}
};
EventManager::EventManager() : imp_(std::make_unique<Imp>()) {
  // imp_->trigger_pool_ = imp_->trigger_factory_->idNamePair();
  // imp_->trigger_creator_ = imp_->trigger_factory_->map();
  imp_->event_pool_ = imp_->event_factory_->idNamePair();
  imp_->event_creator_ = imp_->event_factory_->map();
  imp_->handler_pool_ = imp_->handler_factory_->idNamePair();
  imp_->handler_creator_ = imp_->handler_factory_->map();
}
EventManager::~EventManager() = default;
SIRE_DEFINE_MOVE_CTOR_CPP(EventManager);
auto EventManager::init(simulator::SimulationLoop* simulationLoopPtr) -> void {
  imp_->header_ = imp_->event_list_.begin();
  SIRE_ASSERT(simulationLoopPtr != nullptr);
  imp_->simulationLoopPtr_ = simulationLoopPtr;
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
auto EventManager::addEvent(std::unique_ptr<core::EventBase> e) -> void {
  imp_->event_list_.push_back(std::move(e));
}
auto EventManager::createEventById(sire::Size event_id)
    -> std::unique_ptr<core::EventBase> {
  std::unique_ptr<core::EventBase> new_event = imp_->event_creator_[event_id]();
  new_event->setEventId(event_id);
  new_event->setEventType(imp_->event_pool_[event_id]);
  return std::move(new_event);
}
auto EventManager::createHandlerById(sire::Size handler_id)
    -> std::unique_ptr<core::HandlerBase> {
  std::unique_ptr<core::HandlerBase> new_handler =
      imp_->handler_creator_[handler_id]();
  new_handler->setHandlerId(handler_id);
  new_handler->setHandlerType(imp_->handler_pool_[handler_id]);
  return new_handler;
}
auto EventManager::updateCtrlSimTime(core::EventId eventId, double currentTime)
    -> void {
  if (eventId == 0) {
    imp_->prevCtrlTime_ = 0;
    imp_->prevIntTime_ = 0;
    imp_->nextSuggestTime_ = -1;
  } else if (eventId == 1) {
    imp_->prevIntTime_ = currentTime;
    imp_->nextSuggestTime_ = -1;
  } else if (eventId == 2) {
    imp_->prevCtrlTime_ = currentTime;
    imp_->prevIntTime_ = currentTime;
    imp_->nextSuggestTime_ = -1;
  }
}
auto EventManager::cptNextCtrlSimSuggestDt() -> double {
  double nextCtrlSimSuggestDt{-1};
  double simDt = imp_->simulationLoopPtr_->deltaT();
  double ctrlDt = imp_->simulationLoopPtr_->ctrlT();
  SIRE_ASSERT(simDt > 0);
  SIRE_ASSERT(ctrlDt > 0);
  double nextCtrlTime = imp_->prevCtrlTime_ + ctrlDt;
  double nextSimTime = imp_->prevIntTime_ + simDt;
  std::cout << "next ctrl time " << nextCtrlTime << " next sim time "
            << nextSimTime << std::endl;
  if (nextCtrlTime < nextSimTime ||
      aris::dynamic::s_is_equal(nextCtrlTime, nextSimTime, 1e-8)) {
    nextCtrlSimSuggestDt =
        nextCtrlTime - imp_->simulationLoopPtr_->timer().simTime();
    imp_->nextEventId_ = 2;
  } else {
    nextCtrlSimSuggestDt =
        nextSimTime - imp_->simulationLoopPtr_->timer().simTime();
    imp_->nextEventId_ = 1;
  }
  return nextCtrlSimSuggestDt;
}
auto EventManager::generateEvent() -> void {
  double nextDt{-1};
  double simDt = imp_->simulationLoopPtr_->deltaT();
  double ctrlDt = imp_->simulationLoopPtr_->ctrlT();
  SIRE_ASSERT(simDt > 0);
  SIRE_ASSERT(ctrlDt > 0);
  std::unique_ptr<core::EventBase> nextEvent;
  if (imp_->nextSuggestTime_ <= 0) {
    double nextCtrlTime = imp_->prevCtrlTime_ + ctrlDt;
    double nextSimTime = imp_->prevIntTime_ + simDt;
    if (nextCtrlTime < nextSimTime) {
      nextDt = nextCtrlTime - imp_->simulationLoopPtr_->timer().simTime();
      nextEvent = this->createEventById(2);
    } else {
      nextDt = nextSimTime - imp_->simulationLoopPtr_->timer().simTime();
      nextEvent = this->createEventById(1);
    }
  } else {
    nextDt = imp_->nextSuggestTime_;
    nextEvent = this->createEventById(1);
  }
  DLOG_IF(imp_->nextSuggestTime_ > 0, DEBUG)
      << "dt: " << " suggestDt: " << imp_->nextSuggestTime_;
  nextEvent->eventProp().addProp("dt", nextDt);
  this->addEvent(std::move(nextEvent));
}
auto EventManager::nextEventId() -> sire::Size { return imp_->nextEventId_; }
auto EventManager::setNextEventId(sire::Size nextEventId) -> void {
  imp_->nextEventId_ = nextEventId;
}
auto EventManager::isEventListEmpty() -> bool {
  return imp_->header_ != imp_->event_list_.end();
}
auto EventManager::eventListHeader() -> core::EventBase* {
  return imp_->header_->get();
}
auto EventManager::popEventListHeader() -> void {
  imp_->event_list_.pop_front();
}
auto EventManager::headerNextEvent(sire::Size n) -> void {
  imp_->header_ = std::next(imp_->header_, n);
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
}  // namespace sire::simulator