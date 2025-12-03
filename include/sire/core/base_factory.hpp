#ifndef SIRE_BASE_FACTORY_HPP_
#define SIRE_BASE_FACTORY_HPP_
#include <map>
#include <memory>
#include <string>

#include <sire_lib_export.h>

#include "sire/core/constants.hpp"
#include "sire/core/event_base.hpp"
#include "sire/core/handler_base.hpp"
// template class or function should not dll_export
// client usage will cause link error
namespace sire::core {
using EventId = sire::Size;
class SIRE_API EventBaseFactory {
  typedef std::map<EventId, std::unique_ptr<EventBase> (*)()> Map;

 public:
  auto registration(const std::string& name, EventId id,
                    std::unique_ptr<EventBase> (*createFunc)()) {
    EventId new_id = generateAndVerifyNewId(id);
    map_.insert(std::make_pair(new_id, createFunc));
    id_name_pair_.insert({new_id, name});
  }
  auto generateAndVerifyNewId(EventId id) -> EventId;
  auto map() -> Map& { return map_; };
  auto idNamePair() -> std::map<EventId, std::string>& {
    return id_name_pair_;
  };
  auto clear() -> void {
    map_.clear();
    id_name_pair_.clear();
  }
  static auto instance() noexcept -> EventBaseFactory&;

 private:
  EventBaseFactory() = default;
  virtual ~EventBaseFactory() = default;
  EventBaseFactory(const EventBaseFactory&) = delete;
  EventBaseFactory& operator=(const EventBaseFactory&) = delete;
  Map map_;
  std::map<EventId, std::string> id_name_pair_;
};
template <typename T>
auto createEventFunc() -> std::unique_ptr<EventBase> {
  return std::make_unique<T>();
}
template <typename T>
class EventRegister final {
 public:
  EventRegister(const std::string& s, EventId id){
    EventBaseFactory::instance().registration(s, id, &createEventFunc<T>);
  };
  ~EventRegister() = default;
  static auto registration(const std::string& s, EventId id) -> void {
    EventBaseFactory::instance().registration(s, id, &createEventFunc<T>);
  }
  static auto clear() -> void { EventBaseFactory::instance().clear(); }
};
using HandlerId = sire::Size;
class SIRE_API HandlerBaseFactory {
  typedef std::map<HandlerId, std::unique_ptr<HandlerBase> (*)()> Map;

 public:
  auto registration(const std::string& name, HandlerId id,
                    std::unique_ptr<HandlerBase> (*createFunc)()) {
    HandlerId new_id = generateAndVerifyNewId(id);
    map_.insert(std::make_pair(new_id, createFunc));
    id_name_pair_.insert({new_id, name});
  }
  auto generateAndVerifyNewId(HandlerId id) -> HandlerId;
  auto map() -> Map& { return map_; };
  auto idNamePair() -> std::map<HandlerId, std::string>& {
    return id_name_pair_;
  };
  auto clear() -> void {
    map_.clear();
    id_name_pair_.clear();
  }
  static auto instance() noexcept -> HandlerBaseFactory&;

 private:
  HandlerBaseFactory() = default;
  virtual ~HandlerBaseFactory() = default;
  HandlerBaseFactory(const HandlerBaseFactory&) = delete;
  HandlerBaseFactory& operator=(const HandlerBaseFactory&) = delete;
  Map map_;
  std::map<HandlerId, std::string> id_name_pair_;
};

template <typename T>
auto createHandlerFunc() -> std::unique_ptr<HandlerBase> {
  return std::make_unique<T>();
}

template <typename T>
class HandlerRegister final {
 public:
 public:
  HandlerRegister() = default;
  ~HandlerRegister() = default;
  static auto registration(const std::string& s, HandlerId id) -> void {
    HandlerBaseFactory::instance().registration(s, id, &createHandlerFunc<T>);
  }
  static auto clear() -> void { HandlerBaseFactory::instance().clear(); }
};

// template <typename T>
// class SIRE_API BaseFactory {
//   typedef std::map<sire::Size, std::unique_ptr<T> (*)()> Map;
//
//  public:
//   auto registration(const std::string& name, sire::Size id,
//                     std::unique_ptr<T> (*createFunc)()) {
//     sire::Size new_id = generateAndVerifyNewId(id);
//     map_.insert(std::make_pair(new_id, createFunc));
//     id_name_pair_.insert({new_id, name});
//   }
//   auto generateAndVerifyNewId(sire::Size id) -> sire::Size {
//     if (auto search = map_.find(id); search != map_.end()) {
//       static sire::Size id_generator = 0;
//       for (auto search = map_->find(id_generator); search != map_.end();) {
//         ++id_generator;
//       }
//       return id_generator;
//     } else {
//       return id;
//     }
//   }
//   auto createObject(const std::string& s) -> std::unique_ptr<T>;
//   auto map() -> Map& { return map_; };
//   auto idNamePair() -> std::map<sire::Size, std::string>& {
//     return id_name_pair_;
//   };
//   static auto instance() noexcept -> BaseFactory<T>&;
//
//  private:
//   BaseFactory() = default;
//   virtual ~BaseFactory() = default;
//   BaseFactory(const BaseFactory&) = delete;
//   BaseFactory& operator=(const BaseFactory&) = delete;
//   Map map_;
//   std::map<sire::Size, std::string> id_name_pair_;
// };

}  // namespace sire::core
#endif