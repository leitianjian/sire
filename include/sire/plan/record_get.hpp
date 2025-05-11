#ifndef SIRE_RECORD_GET_COMMAND_H_
#define SIRE_RECORD_GET_COMMAND_H_

#include <sire_lib_export.h>

#include <aris.hpp>

namespace sire::plan {
class SIRE_API RecordGet
    : public aris::core::CloneObject<RecordGet, aris::plan::Plan> {
 public:
  auto virtual prepareNrt() -> void override;
  // auto virtual collectNrt() -> void override;
  explicit RecordGet(const std::string& name = "record_get");
};
}  // namespace sire::plan
#endif