#ifndef SIRE_DISPLAY3D_INIT_COMMAND_H_
#define SIRE_DISPLAY3D_INIT_COMMAND_H_

#include <sire_lib_export.h>

#include <aris.hpp>

namespace sire::plan {
class SIRE_API Display3dInit
    : public aris::core::CloneObject<Display3dInit, aris::plan::Plan> {
 public:
  auto virtual prepareNrt() -> void override;
  auto virtual collectNrt() -> void override;
  auto virtual test() -> void;
  explicit Display3dInit(const std::string& name = "Display3dInit_plan");
};
}  // namespace sire::plan
#endif