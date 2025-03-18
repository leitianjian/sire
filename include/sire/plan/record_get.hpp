#ifndef SIRE_RECORD_GET_COMMAND_H_
#define SIRE_RECORD_GET_COMMAND_H_

#include <aris.hpp>

namespace sire::plan {
class RecordGet
    : public aris::core::CloneObject<RecordGet, aris::plan::Plan> {
 public:
  auto virtual prepareNrt() -> void override;
  // auto virtual collectNrt() -> void override;
  explicit RecordGet(const std::string& name = "record_get");
};
}  // namespace sire::server
#endif