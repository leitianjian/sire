#ifndef COMMAND_H_
#define COMMAND_H_

#include <aris.hpp>

namespace sire::server {
class Get : public aris::core::CloneObject<Get, aris::plan::Plan> {
 public:
  auto virtual prepareNrt() -> void;
  auto virtual collectNrt() -> void;
  explicit Get(const std::string& name = "Get_plan");
};
}
#endif