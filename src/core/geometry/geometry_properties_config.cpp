#ifndef SIRE_GEOMETRY_PROPERTIES_CONFIG_HPP_
#define SIRE_GEOMETRY_PROPERTIES_CONFIG_HPP_

#include <any>
#include <atomic>
#include <set>
#include <string>
#include <unordered_map>

#include <sire_lib_export.h>

#include "sire/core/module_base.hpp"

namespace sire::geometry {
using namespace std;
class GeometryPropertiesConfig : public  {
  // Property的值类型是any，并且名字和类型会在xml中被注册以使用
  using Group = std::unordered_map<std::string, std::any>;

 public:
  virtual ~GeometryProperties() = default;

  bool hasGroup(const std::string& group_name) const {
    return groups_.count(group_name) > 0;
  }

  int numGroups() const { return static_cast<int>(groups_.size()); }

  const Group& getGroupByName(const std::string& group_name) const;

  std::set<std::string> GetGroupNames() const;

  /** Adds the named property (`group_name`, `name`) with the given `value`.
Adds the group if it doesn't already exist.

@param group_name   The group name.
@param name         The name of the property -- must be unique in the group.
@param value        The value to assign to the property.
@throws std::exception if the property already exists.
@tparam ValueType   The type of data to store with the attribute -- must be
                   copy constructible or cloneable (see Value).  */
  template <typename ValueType>
  void AddProperty(const std::string& group_name, const std::string& name,
                   const ValueType& value) {
    AddPropertyAbstract(group_name, name, std::make_any(value));
  }

 private:
  // The collection of property groups.
  std::unordered_map<std::string, Group> groups_;
};
}  // namespace sire::geometry
#endif