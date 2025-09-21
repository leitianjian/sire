#ifndef SIRE_EVENT_MANAGER_HPP_
#define SIRE_EVENT_MANAGER_HPP_
#include <memory>

#include <sire_lib_export.h>

#include <aris/core/object.hpp>

#include "sire/core/handler_base.hpp"
#include "sire/core/sire_decl_def_macro.hpp"
#include "sire/core/sorted_pair.hpp"
namespace sire::simulator {
// class SimulationLoop;
class SIRE_API EventHandlerIdPair {
 public:
  EventHandlerIdPair() = default;
  EventHandlerIdPair(core::EventId name1, core::HandlerId name2)
      : name1_(name1), name2_(name2) {}
  auto setEventId(core::EventId name1) -> void { name1_ = name1; }
  auto setHandlerId(core::HandlerId name2) -> void { name2_ = name2; }
  auto getEventId() const -> core::EventId { return name1_; }
  auto getHandlerId() const -> core::HandlerId { return name2_; }

 private:
  core::EventId name1_;
  core::HandlerId name2_;
};
class SIRE_API EventManager {
 public:
  auto init(simulator::SimulationLoop* simulationLoopPtr) -> void;
  auto resetEventHandlerPairPool(
      aris::core::PointerArray<EventHandlerIdPair>* pool) -> void;
  auto eventHandlerPairPool() -> aris::core::PointerArray<EventHandlerIdPair>&;
  // Event 相关
  auto createEventById(sire::Size event_id) -> std::unique_ptr<core::EventBase>;
  auto createHandlerById(sire::Size handler_id)
      -> std::unique_ptr<core::HandlerBase>;
  auto addEvent(std::unique_ptr<core::EventBase> e) -> void;
  auto updateCtrlSimTime(core::EventId eventId, double currentTime) -> void;
  auto generateEvent() -> void;
  auto cptNextCtrlSimSuggestDt() -> double;
  auto nextEventId() -> sire::Size;
  auto setNextEventId(sire::Size nextEventId) -> void;
  auto isEventListEmpty() -> bool;
  auto eventListHeader() -> core::EventBase*;
  auto popEventListHeader() -> void;
  auto getHandlerIdByEventId(sire::Size event_id) -> sire::Size;
  auto getEventIdByTriggerId(sire::Size trigger_id) -> sire::Size;
  auto headerNextEvent(sire::Size n) -> void;
  auto addEventHandlerRule(sire::core::EventId name1,
                           sire::core::HandlerId name2) -> void;
  auto eventHandlerMap() -> std::map<sire::Size, sire::Size>&;
  //   auto addImmediateTrigger(std::unique_ptr<core::TriggerBase> trigger) ->
  //   void; auto addPeriodicTrigger(std::unique_ptr<core::TriggerBase> trigger)
  //   -> void {};
  auto executeAllImmediateTrigger() -> void;

  EventManager();
  virtual ~EventManager();
  SIRE_DECLARE_MOVE_CTOR(EventManager);

 private:
  struct Imp;
  std::unique_ptr<Imp> imp_;
};
}  // namespace sire::simulator
#endif