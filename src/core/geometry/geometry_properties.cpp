#include "sire/core/geometry/geometry_properties.hpp"

#include <stdexcept>
#include <string>

#include "sire/core/sire_assert.hpp"

namespace sire::geometry {
const GeometryProperties::Group& GeometryProperties::getGroupByName(
    const string& group_name) const {
  const auto iter = groups_.find(group_name);
  if (iter != groups_.end()) {
    return iter->second;
  }
  throw std::logic_error(
      std::string("GetPropertiesInGroup(): Can't retrieve properties for a "
                  "group that doesn't exist: '" +
                  group_name + "'"));
}

std::set<string> GeometryProperties::getGroupNames() const {
  std::set<string> group_names;
  for (const auto& pair : groups_) {
    group_names.insert(pair.first);
  }
  return group_names;
}
void GeometryProperties::addProperty(const string& group_name,
                                     const string& name, double value) {
  writeProperty(
      group_name, name, value, [&group_name, &name](const Group& group) {
        const auto value_iter = group.find(name);
        if (value_iter != group.end()) {
          throw std::logic_error(
              std::string("AddProperty(): Trying to add property ('" +
                          group_name + "', ' + " + name +
                          "'); a property with that name already exists"));
        }
      });
}

void GeometryProperties::updateProperty(const string& group_name,
                                        const string& name, double value) {
  writeProperty(group_name, name, value,
                [&group_name, &name, &value](const Group& group) {
                  const auto value_iter = group.find(name);
                });
}
bool GeometryProperties::hasProperty(const string& group_name,
                                     const string& name) const {
  return getPropertyMaybe(group_name, name, false) != nullptr;
}
double GeometryProperties::getProperty(const string& group_name,
                                       const string& name) const {
  const double* abstract = getPropertyMaybe(group_name, name, true);
  if (!abstract) {
    throw std::logic_error(
        std::string("GetProperty(): There is no property ('" + group_name +
                    "', '" + name + "')"));
  }
  return *abstract;
}
void GeometryProperties::writeProperty(
    const string& group_name, const string& name, double value,
    const std::function<void(const Group&)>& throw_if_invalid) {
  auto iter = groups_.find(group_name);
  if (iter == groups_.end()) {
    auto result = groups_.insert({group_name, Group{}});
    SIRE_DEMAND(result.second);
    iter = result.first;
  }

  Group& group = iter->second;
  throw_if_invalid(group);

  group[name] = value;
}
const double* GeometryProperties::getPropertyMaybe(
    const string& group_name, const string& name,
    bool throw_for_bad_group) const {
  const auto iter = groups_.find(group_name);
  if (iter == groups_.end()) {
    if (throw_for_bad_group) {
      throw std::logic_error(
          std::string("GetProperty(): Trying to read property ('" + group_name +
                      "', '" + name + "'), but the group does not exist."));
    } else {
      return nullptr;
    }
  }
  const Group& group = iter->second;
  const auto value_iter = group.find(name);
  if (value_iter != group.end()) {
    return &value_iter->second;
  } else {
    return nullptr;
  }
}
}  // namespace sire::geometry
