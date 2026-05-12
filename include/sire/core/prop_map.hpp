#ifndef SIRE_PROP_MAP_HPP_
#define SIRE_PROP_MAP_HPP_
#include <string_view>

#include <sire_lib_export.h>

#include <aris/core/object.hpp>
namespace sire::core {
class SIRE_API PropMap {
 public:
  auto addProp(const std::string& name, double value) -> void;
  auto rmProp(const std::string& name) -> bool;
  auto updProp(const std::string& name, double new_value) -> void;
  auto clear() -> void;
  auto contains(const std::string& name) const -> bool;
  auto getPropValue(const std::string& name) const -> double;
  auto getPropValueOrDefault(const std::string& name,
                             double default_value) const -> double;
  auto toString() const -> std::string;
  auto fromString(std::string_view str) -> bool;
  auto swap(PropMap& other) noexcept -> PropMap&;
  auto compare(const PropMap& other) const noexcept -> bool;
  PropMap();
  PropMap(std::string s);
  virtual ~PropMap();
  ARIS_DECLARE_BIG_FOUR(PropMap);

 private:
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;
};
inline bool operator==(const PropMap& lhs, const PropMap& rhs) {
  return lhs.compare(rhs);
}
}  // namespace sire::core
namespace std {
template <>
inline auto swap<sire::core::PropMap>(sire::core::PropMap& a,
                                      sire::core::PropMap& b) noexcept -> void {
  a.swap(b);
}
}  // namespace std
#endif  // !SIRE_PROP_MAP_HPP_
