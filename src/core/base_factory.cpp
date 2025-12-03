#include "sire/core/base_factory.hpp"

namespace sire::core {
auto EventBaseFactory::instance() noexcept -> EventBaseFactory& {
  static EventBaseFactory factory;
  return factory;
}
auto EventBaseFactory::generateAndVerifyNewId(EventId id) -> EventId {
  if (auto search = map_.find(id); search != map_.end()) {
    static EventId id_generator = 0;
    for (auto search = map_.find(id_generator); search != map_.end();) {
      ++id_generator;
    }
    return id_generator;
  } else {
    return id;
  }
}
auto HandlerBaseFactory::instance() noexcept -> HandlerBaseFactory& {
  static HandlerBaseFactory factory;
  return factory;
}
auto HandlerBaseFactory::generateAndVerifyNewId(HandlerId id) -> HandlerId {
  if (auto search = map_.find(id); search != map_.end()) {
    static HandlerId id_generator = 0;
    for (auto search = map_.find(id_generator); search != map_.end();) {
      ++id_generator;
    }
    return id_generator;
  } else {
    return id;
  }
}
}  // namespace sire::core