#ifndef SIRE_CORE_UNORDERED_MAP_H_
#define SIRE_CORE_PROPERTY_MAP_H_

#include <string>
#include <unordered_map>

#include <sire_lib_export.h>

#include <aris/core/object.hpp>

namespace sire::core {
template <class ValueType>
class SIRE_API PropertyMap {
  using Map = std::unordered_map<std::string, ValueType>;

 public:
  auto empty() -> bool { return map_.empty(); }
  auto begin() -> Map::iterator { return map_.begin(); }

 private:
  Map map_;
};
}  // namespace sire::core

#endif