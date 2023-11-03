#include "sire/core/material_manager.hpp"

#include <string>
#include <unordered_map>

#include <aris/core/object.hpp>
#include <aris/core/reflection.hpp>

namespace sire::core {
using std::string;
using MaterialPair = SortedPair<string>;
struct MaterialManager::Imp {
  aris::core::PointerArray<core::MaterialPairProp> material_prop_pool_{};
  core::PropMap default_prop_{"{k:1.4e8, d:1000, cr:1.0}"};
  std::unordered_map<MaterialPair, core::PropMap> material_pair_prop_;
};
MaterialManager::MaterialManager() : imp_(new Imp){};
MaterialManager::~MaterialManager() = default;
SIRE_DEFINE_MOVE_CTOR_CPP(MaterialManager);
auto MaterialManager::addProp(const MaterialPair& p, const PropMap& m) -> void {
  imp_->material_pair_prop_[p] = m;
}
auto MaterialManager::rmProp(const MaterialPair& p) -> bool {
  if (auto search = imp_->material_pair_prop_.find(p);
      search != imp_->material_pair_prop_.end()) {
    imp_->material_pair_prop_.erase(search);
    return true;
  } else {
    return false;
  }
}
auto MaterialManager::updProp(const core::SortedPair<std::string>& p,
                              const core::PropMap& m) -> void {
  imp_->material_pair_prop_[p] = m;
}
auto MaterialManager::clear() -> void { imp_->material_pair_prop_.clear(); }
auto MaterialManager::getPropMapOrDefault(
    const core::SortedPair<std::string>& p) const -> const PropMap& {
  if (auto search = imp_->material_pair_prop_.find(p);
      search != imp_->material_pair_prop_.end()) {
    return search->second;
  } else {
    return imp_->default_prop_;
  }
}
auto MaterialManager::defaultProp() const -> const core::PropMap& {
  return imp_->default_prop_;
}

auto MaterialManager::setDefaultProp(core::PropMap& map) -> void {
  imp_->default_prop_ = map;
}

auto MaterialManager::resetMaterialPairPropPool(
    aris::core::PointerArray<core::MaterialPairProp>* pool) -> void {
  for (auto& material_prop : *pool) {
    imp_->material_pair_prop_[material_prop.sortedPair()] =
        material_prop.getMaterialProp();
  }
}

auto MaterialManager::materialPairPropPool()
    -> aris::core::PointerArray<core::MaterialPairProp>& {
  imp_->material_prop_pool_.clear();
  for (auto& [key, value] : imp_->material_pair_prop_) {
    imp_->material_prop_pool_.push_back(
        std::make_unique<core::MaterialPairProp>(key, value).release());
  }
  return imp_->material_prop_pool_;
}

ARIS_REGISTRATION {
  using MaterialPairPropPool = aris::core::PointerArray<core::MaterialPairProp>;

  aris::core::class_<MaterialPairPropPool>("MaterialPairPropPool").asRefArray();

  typedef MaterialPairPropPool& (MaterialManager::*MaterialPairPropPoolFunc)();

  auto setDefaultProp = [](MaterialManager* m, core::PropMap prop) -> void {
    m->setDefaultProp(prop);
  };
  auto getDefaultProp = [](MaterialManager* m) -> core::PropMap {
    return m->defaultProp();
  };
  aris::core::class_<MaterialManager>("MaterialManager")
      .prop("default_prop", &setDefaultProp, &getDefaultProp)
      .prop("material_pair_prop_pool",
            &MaterialManager::resetMaterialPairPropPool,
            MaterialPairPropPoolFunc(&MaterialManager::materialPairPropPool));
  ;
}
}  // namespace sire::core