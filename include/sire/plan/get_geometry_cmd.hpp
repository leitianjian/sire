#ifndef SIRE_GET_GEOMETRY_COMMAND_H_
#define SIRE_GET_GEOMETRY_COMMAND_H_

#include <aris.hpp>

namespace sire::plan {
class GetGeometry
    : public aris::core::CloneObject<GetGeometry, aris::plan::Plan> {
 public:
  auto virtual prepareNrt() -> void override;
  auto virtual collectNrt() -> void override;
  explicit GetGeometry(const std::string& name = "GetGeometry_plan");
};
}  // namespace sire::server
#endif