#ifndef SIRE_MATERIAL_PAIR_PROP_HPP_
#define SIRE_MATERIAL_PAIR_PROP_HPP_
#include <algorithm>
#include <string>
#include <utility>

#include <sire_lib_export.h>

#include "sire/core/prop_map.hpp"
#include "sire/core/sorted_pair.hpp"
namespace sire::core {
class SIRE_API MaterialPairProp {
 public:
  auto setFirstName(const std::string& first_name) -> void {
    first_material_name = first_name;
    material_name_pair_.set(first_material_name, second_material_name);
  }
  auto setSecondName(const std::string& second_name) -> void {
    second_material_name = second_name;
    material_name_pair_.set(first_material_name, second_material_name);
  }
  auto getFirstName() const -> const std::string& {
    return first_material_name;
  }
  auto getSecondName() const -> const std::string& {
    return second_material_name;
  }
  auto sortedPair() const -> const SortedPair<std::string>& {
    return material_name_pair_;
  }
  auto setMaterialProp(core::PropMap& map) -> void { material_prop_ = map; }
  auto getMaterialProp() const -> const PropMap& { return material_prop_; }
  auto getMaterialProp() -> core::PropMap& {
    return const_cast<core::PropMap&>(
        static_cast<const MaterialPairProp&>(*this).getMaterialProp());
  }
  MaterialPairProp(const core::SortedPair<std::string>& p,
                   const core::PropMap& m)
      : first_material_name(p.first()),
        second_material_name(p.second()),
        material_name_pair_(p),
        material_prop_(m) {}
  MaterialPairProp() = default;

 private:
  std::string first_material_name;
  std::string second_material_name;
  SortedPair<std::string> material_name_pair_;
  PropMap material_prop_;
};
}  // namespace sire::core
namespace std {
template <>
struct hash<sire::core::MaterialPairProp> {
  std::size_t operator()(const sire::core::MaterialPairProp& m) const {
    using std::hash;
    return hash<sire::core::SortedPair<std::string>>()(m.sortedPair());
  }
};
}  // namespace std
#endif