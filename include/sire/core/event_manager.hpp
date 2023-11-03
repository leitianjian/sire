#ifndef SIRE_CORE_EVENT_MANAGER_HPP_
#define SIRE_CORE_EVENT_MANAGER_HPP_
#include <memory>

#include <sire_lib_export.h>

#include <aris/core/object.hpp>

#include "sire/core/handler_base.hpp"
#include "sire/core/sire_decl_def_macro.hpp"
#include "sire/core/sorted_pair.hpp"
#include "sire/core/trigger_base.hpp"
namespace sire::core {
class EventHandlerIdPair {
 public:
  EventHandlerIdPair() = default;
  EventHandlerIdPair(EventId name1, HandlerId name2)
      : name1_(name1), name2_(name2) {}
  auto setEventId(EventId name1) -> void { name1_ = name1; }
  auto setHandlerId(HandlerId name2) -> void { name2_ = name2; }
  auto getEventId() const -> EventId { return name1_; }
  auto getHandlerId() const -> HandlerId { return name2_; }

 private:
  EventId name1_;
  HandlerId name2_;
};
class SIRE_API EventManager {
 public:
  auto init() -> void;
  auto resetEventHandlerPairPool(
      aris::core::PointerArray<EventHandlerIdPair>* pool) -> void;
  auto eventHandlerPairPool() -> aris::core::PointerArray<EventHandlerIdPair>&;
  auto addEvent(std::unique_ptr<EventBase> e) -> void;
  auto isEventListEmpty() -> bool;
  auto eventListHeader() -> EventBase*;
  auto popEventListHeader() -> void;
  auto getHandlerIdByEventId(sire::Size event_id) -> sire::Size;
  auto getEventIdByTriggerId(sire::Size trigger_id) -> sire::Size;
  auto headerNextEvent(sire::Size n) -> void;
  auto addImmediateTrigger(std::unique_ptr<core::TriggerBase> trigger) -> void;
  auto addPeriodicTrigger(std::unique_ptr<core::TriggerBase> trigger) -> void{};
  auto executeAllImmediateTrigger() -> void;

  EventManager();
  virtual ~EventManager();
  SIRE_DECLARE_MOVE_CTOR(EventManager);

 private:
  struct Imp;
  std::unique_ptr<Imp> imp_;
};
}  // namespace sire::core
#endif