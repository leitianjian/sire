#ifndef SIRE_GEOMETRY_PROPERTIES_HPP_
#define SIRE_GEOMETRY_PROPERTIES_HPP_

#include <any>
#include <atomic>
#include <functional>
#include <set>
#include <string>
#include <unordered_map>

#include <sire_lib_export.h>

namespace sire::geometry {
using namespace std;
class GeometryProperties {

  // TODO(leitianjian):
  // Property的值类型是任意类型，并且名字和类型会在xml中被注册以使用
  using Group = std::unordered_map<std::string, double>;

  virtual ~GeometryProperties() = default;

  bool hasGroup(const std::string& group_name) const {
    return groups_.count(group_name) > 0;
  }

  int numGroups() const { return static_cast<int>(groups_.size()); }

  const Group& getGroupByName(const std::string& group_name) const;

  std::set<std::string> getGroupNames() const;

  /** Adds the named property (`group_name`, `name`) with the given `value`.
   Adds the group if it doesn't already exist.

   @param group_name   The group name.
   @param name         The name of the property -- must be unique in the group.
   @param value        The value to assign to the property.
   @throws std::exception if the property already exists.
   */
  void addProperty(const std::string& group_name, const std::string& name,
                   double value);

  /** Updates the named property (`group_name`, `name`) with the given `value`.
   If the property doesn't already exist, it is equivalent to calling
   `AddProperty`. If the property does exist, its value (which must have the
   same type as `value`) will be replaced.

   @param group_name   The group name.
   @param name         The name of the property -- must be unique in the group.
   @param value        The value to assign to the property.
   @throws std::exception if the property exists with a different type.
   @tparam ValueType   The type of data to store with the attribute -- must be
                   copy constructible or cloneable (see Value).  */
  void updateProperty(const std::string& group_name, const std::string& name,
                      double value);

  /** Reports if the property (`group_name`, `name`) exists in the group.

@param group_name  The name of the group to which the tested property should
                  belong.
@param name        The name of the property under question.
@returns true iff the group exists and a property with the given `name`
             exists in that group.  */
  bool hasProperty(const std::string& group_name,
                   const std::string& name) const;

  /** Retrieves the typed value for the property (`group_name`, `name`) from
   this set of properties.

   @param group_name  The name of the group to which the property belongs.
   @param name        The name of the desired property.
   @throws std::exception if a) the group name is invalid,
                          b) the property name is invalid, or
                          c) the property type is not that specified.
   @tparam ValueType  The expected type of the desired property.
   @returns const ValueType& of stored value.
            If ValueType is Eigen::Vector4d, the return type will be a copy
            translated from Rgba.
   */
  double getProperty(const std::string& group_name,
                     const std::string& name) const;

  /** Retrieves the typed value for the property (`group_name`, `name`) from the
    set of properties (if it exists), otherwise returns the given default value.
    The given `default_value` is returned only if the property is missing. If
    the property exists and is of a _different_ type, an exception will be
    thrown. If it is of the expected type, the stored value will be returned.

    Generally, it is unnecessary to explicitly declare the `ValueType` of the
    property value; it will be inferred from the provided default value.
    Sometimes it is convenient to provide the default value in a form that can
    be implicitly converted to the final type. In that case, it is necessary
    to explicitly declare the desired `ValueType` so the compiler does not
    infer the wrong type, e.g.:

    ```
    // Note the _integer_ value as default value.
    const double my_value = properties.GetPropertyOrDefault<double>("g", "p",
    2);
    ```

    @param group_name     The name of the group to which the property belongs.
    @param name           The name of the desired property.
    @param default_value  The alternate value to return if the property cannot
                          be acquired.
    @throws std::exception if a property of the given name exists but is not
                        of `ValueType`.  */
  double GetPropertyOrDefault(const std::string& group_name,
                              const std::string& name,
                              double default_value) const {
    const double* value = getPropertyMaybe(group_name, name, false);
    if (!value) {
      return default_value;
    } else {
      // This incurs the cost of copying a stored value.
      return *value;
    }
  }

 private:
  // Conditionally writes the property (group_name, name) with the given value.
  // The caller provides a test function that should throw if assigning `value`
  // to the specified property would fail. The function takes the `Group`
  // associated with `group_name`.
  void writeProperty(const std::string& group_name, const std::string& name,
                     double value,
                     const std::function<void(const Group&)>& throw_if_invalid);
  // Return value or nullptr if it does not exist.
  // If `throw_for_bad_group` is true, an error will be thrown if `group_name`
  // does not exist.
  const double* getPropertyMaybe(const std::string& group_name,
                                 const std::string& name,
                                 bool throw_for_bad_group) const;

  // The collection of property groups.
  std::unordered_map<std::string, Group> groups_;
};
}  // namespace sire::geometry
#endif