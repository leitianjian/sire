#ifndef SIRE_SIM_PLAY_COMMAND_H_
#define SIRE_SIM_PLAY_COMMAND_H_

#include <sire_lib_export.h>

#include <aris.hpp>

namespace sire::plan {
class SIRE_API SimPlay
    : public aris::core::CloneObject<SimPlay, aris::plan::Plan> {
 public:
  auto virtual prepareNrt() -> void override;
  virtual ~SimPlay();
  explicit SimPlay(const std::string& name = "SimReset");
  ARIS_DECLARE_BIG_FOUR(SimPlay);

 private:
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;
};
}  // namespace sire::plan
#endif