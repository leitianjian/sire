#ifndef SIRE_HANDLER_BASE_HPP_
#define SIRE_HANDLER_BASE_HPP_
#include <sire_lib_export.h>

#include <aris/core/object.hpp>

#include "sire/core/constants.hpp"
#include "sire/core/event_base.hpp"
namespace sire::simulator {
class SimulatorBase;
}
namespace sire::core {
class EventManager;
using HandlerId = sire::Size;
class SIRE_API HandlerBase {
 public:
  explicit HandlerBase() : handler_type_(), handler_id_(-1){};
  virtual ~HandlerBase() = default;
  ARIS_DEFINE_BIG_FOUR(HandlerBase);

  virtual auto init(simulator::SimulatorBase*) -> void = 0;
  virtual auto handle(EventBase*) -> bool = 0;
  auto handlerType() const -> std::string { return handler_type_; }
  auto setHandlerType(const std::string& handler_type) -> void {
    handler_type_ = handler_type;
  }
  auto targetEventId() const -> EventId { return target_event_id_; }
  auto setTargetEventId(EventId target_event_id) -> void {
    target_event_id_ = target_event_id;
  }
  auto handlerId() const -> HandlerId { return handler_id_; }
  auto setHandlerId(HandlerId handler_id) -> void { handler_id_ = handler_id; }

 private:
  std::string handler_type_;
  EventId target_event_id_;
  HandlerId handler_id_;
};
}  // namespace sire::core
#endif