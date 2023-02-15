#ifndef SIRE_CAM_BACKEND_COMMAND_H_
#define SIRE_CAM_BACKEND_COMMAND_H_

#include <aris.hpp>

namespace sire::cam_backend {
class CptCollisionMap
    : public aris::core::CloneObject<CptCollisionMap, aris::plan::Plan> {
 public:
  auto virtual prepareNrt() -> void override;
  auto virtual collectNrt() -> void override;
  explicit CptCollisionMap(const std::string& name = "cpt_collision_map_plan");
};
}  // namespace sire::cam_backend
#endif