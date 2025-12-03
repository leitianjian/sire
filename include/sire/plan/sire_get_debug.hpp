#ifndef SIRE_GET_DEBUG_COMMAND_H_
#define SIRE_GET_DEBUG_COMMAND_H_

#include <aris.hpp>

namespace sire::plan {
class SireGetDebug
    : public aris::core::CloneObject<SireGetDebug, aris::plan::Plan> {
 public:
  auto virtual prepareNrt() -> void override;
  auto virtual collectNrt() -> void override;
  explicit SireGetDebug(const std::string& name = "sire_get_debug");
};
}  // namespace sire::server
#endif