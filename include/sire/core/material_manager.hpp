#ifndef SIRE_MATERIAL_MANAGER_HPP_
#define SIRE_MATERIAL_MANAGER_HPP_
#include <sire_lib_export.h>

#include <aris/core/object.hpp>

#include "sire/core/material_pair_prop.hpp"
#include "sire/core/prop_map.hpp"
#include "sire/core/sire_decl_def_macro.hpp"
#include "sire/core/sorted_pair.hpp"
namespace sire::core {
class MaterialManager {
 public:
  MaterialManager();
  virtual ~MaterialManager();
  SIRE_DECLARE_MOVE_CTOR(MaterialManager);
  auto addProp(const core::SortedPair<std::string>& p, const core::PropMap& m)
      -> void;
  auto rmProp(const core::SortedPair<std::string>& p) -> bool;
  auto updProp(const core::SortedPair<std::string>& p, const core::PropMap& m)
      -> void;
  auto clear() -> void;
  auto getPropMapOrDefault(const core::SortedPair<std::string>& p) const
      -> const core::PropMap&;
  auto defaultProp() const -> const core::PropMap&;
  auto defaultProp() -> core::PropMap& {
    return const_cast<core::PropMap&>(
        static_cast<const MaterialManager&>(*this).defaultProp());
  }
  auto setDefaultProp(core::PropMap& map) -> void;
  auto resetMaterialPairPropPool(
      aris::core::PointerArray<core::MaterialPairProp>* pool) -> void;
  auto materialPairPropPool()
      -> aris::core::PointerArray<core::MaterialPairProp>&;

 private:
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;
};
}  // namespace sire::core
#endif