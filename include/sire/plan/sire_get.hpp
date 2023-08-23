#ifndef SIRE_GET_COMMAND_H_
#define SIRE_GET_COMMAND_H_

#include <aris.hpp>

namespace sire::plan {
class SireGet : public aris::core::CloneObject<SireGet, aris::plan::Plan> {
 public:
  auto virtual prepareNrt() -> void override;
  auto virtual collectNrt() -> void override;
  explicit SireGet(const std::string& name = "sire_get");
};
}  // namespace sire::server
#endif