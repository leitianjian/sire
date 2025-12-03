#ifndef SIRE_SIM_RESET_COMMAND_H_
#define SIRE_SIM_RESET_COMMAND_H_

#include <sire_lib_export.h>

#include <aris.hpp>

namespace sire::plan {
class SIRE_API SimReset
    : public aris::core::CloneObject<SimReset, aris::plan::Plan> {
 public:
  auto virtual prepareNrt() -> void override;
  virtual ~SimReset();
  explicit SimReset(const std::string& name = "SimReset_plan");
  ARIS_DECLARE_BIG_FOUR(SimReset);

 private:
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;
};
}  // namespace sire::plan
#endif