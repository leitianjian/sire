#ifndef SIRE_EVENT_BASE_HPP_
#define SIRE_EVENT_BASE_HPP_
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <sire_lib_export.h>

#include <aris/core/object.hpp>

#include "sire/core/constants.hpp"
#include "sire/core/prop_map.hpp"
#include "sire/core/sire_assert.hpp"
#include "sire/core/sire_decl_def_macro.hpp"
namespace sire::core {
using EventId = sire::Size;
class SIRE_API EventBase {
 public:
  EventBase() : event_type_(), event_id_(-1){};
  virtual ~EventBase() = default;
  ARIS_DEFINE_BIG_FOUR(EventBase);

  virtual auto init() -> void = 0;
  auto eventType() const -> std::string { return event_type_; }
  auto setEventType(const std::string& event_type) -> void {
    event_type_ = event_type;
  }
  auto eventId() const -> EventId { return event_id_; }
  auto setEventId(EventId event_id) -> void { event_id_ = event_id; }
  auto eventProp() const -> const core::PropMap& { return event_prop_; }
  auto eventProp() -> core::PropMap& { return event_prop_; }

 private:
  std::string event_type_;
  EventId event_id_;
  core::PropMap event_prop_;
};
}  // namespace sire::core
#endif