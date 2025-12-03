#ifndef SIRE_GET_COMMAND_H_
#define SIRE_GET_COMMAND_H_

#include <aris.hpp>

namespace sire::plan {
class Get : public aris::core::CloneObject<Get, aris::plan::Plan> {
 public:
  auto virtual prepareNrt() -> void override;
  auto virtual collectNrt() -> void override;
  explicit Get(const std::string& name = "Get_plan");
};
}  // namespace sire::server
#endif