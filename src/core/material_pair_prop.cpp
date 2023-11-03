#include "sire/core/material_pair_prop.hpp"

#include <aris/core/reflection.hpp>
namespace sire::core {

ARIS_REGISTRATION {
  auto setMaterialProp = [](MaterialPairProp* p, core::PropMap map) -> void {
    p->setMaterialProp(map);
  };
  auto getMaterialProp = [](MaterialPairProp* p) -> core::PropMap {
    return p->getMaterialProp();
  };
  aris::core::class_<MaterialPairProp>("MaterialPairProp")
      .prop("first_name", &MaterialPairProp::setFirstName,
            &MaterialPairProp::getFirstName)
      .prop("second_name", &MaterialPairProp::setSecondName,
            &MaterialPairProp::getSecondName)
      .prop("prop", &setMaterialProp, &getMaterialProp);
}
}  // namespace sire::core