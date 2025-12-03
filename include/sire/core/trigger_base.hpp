#ifndef SIRE_TRIGGER_BASE_HPP_
#define SIRE_TRIGGER_BASE_HPP_
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <sire_lib_export.h>

#include <aris/core/object.hpp>

#include "sire/core/base_factory.hpp"
#include "sire/core/constants.hpp"
#include "sire/core/sire_assert.hpp"
#include "sire/core/sire_decl_def_macro.hpp"
namespace sire::simulator {
class SimulationLoop;
}
namespace sire::core {
class EventManager;
class SIRE_API TriggerBase {
 public:
  explicit TriggerBase() : trigger_type_(), trigger_id_(-1){};
  virtual ~TriggerBase() = default;
  ARIS_DEFINE_BIG_FOUR(TriggerBase);

  virtual auto trigger(simulator::SimulationLoop*) -> void = 0;
  auto triggerType() const -> std::string { return trigger_type_; }
  auto setTriggerType(const std::string& trigger_type) -> void {
    trigger_type_ = trigger_type;
  }
  auto targetEventId() const -> EventId { return target_event_id_; }
  auto setTargetEventId(EventId event_id) -> void {
    target_event_id_ = event_id;
  }
  auto triggerId() const -> sire::Size { return trigger_id_; }
  auto setTriggerId(sire::Size trigger_id) -> void { trigger_id_ = trigger_id; }

 private:
  std::string trigger_type_;
  EventId target_event_id_;
  sire::Size trigger_id_;
};

template <typename T>
class SIRE_API TriggerRegister final {
  TriggerRegister() = default;
  ~TriggerRegister() = default;
  static auto registration(const std::string& s, sire::Size id) -> void {
  //  BaseFactory<TriggerBase>::instance().registration(s, id,
  //                                                    &createTriggerFunc<T>);
  }

 private:
  static auto createTriggerFunc() -> std::unique_ptr<TriggerBase> {
    return std::make_unique<T>();
  }
};
}  // namespace sire::core
#endif