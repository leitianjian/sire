#ifndef SIRE_DISPLAY3D_INIT_COMMAND_H_
#define SIRE_DISPLAY3D_INIT_COMMAND_H_

#include <aris.hpp>

namespace sire::plan {
class Display3dInit
    : public aris::core::CloneObject<Display3dInit, aris::plan::Plan> {
 public:
  auto virtual prepareNrt() -> void override;
  auto virtual collectNrt() -> void override;
  explicit Display3dInit(const std::string& name = "Display3dInit_plan");
};
}  // namespace sire::server
#endif