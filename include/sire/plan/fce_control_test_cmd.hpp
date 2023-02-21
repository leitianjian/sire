#ifndef SIRE_FORCE_CONTROL_TEST_COMMAND_H_
#define SIRE_FORCE_CONTROL_TEST_COMMAND_H_

#include <aris.hpp>

namespace sire::plan {
class ForceControlTest
    : public aris::core::CloneObject<ForceControlTest, aris::plan::Plan> {
 public:
  auto virtual prepareNrt() -> void override;
  auto virtual collectNrt() -> void override;
  auto virtual executeRT() -> int override;
  virtual ~ForceControlTest() = default;
  explicit ForceControlTest(const std::string& name = "ForceControlTest_plan");
  ARIS_DECLARE_BIG_FOUR(ForceControlTest);

 private:
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;
};
}  // namespace sire::plan
#endif