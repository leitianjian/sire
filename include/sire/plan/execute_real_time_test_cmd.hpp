#ifndef SIRE_EXECUTE_REAL_TIME_TEST_COMMAND_H_
#define SIRE_EXECUTE_REAL_TIME_TEST_COMMAND_H_

#include <aris.hpp>

namespace sire::plan {
class ExecuteRTTest
    : public aris::core::CloneObject<ExecuteRTTest, aris::plan::Plan> {
 public:
  auto virtual prepareNrt() -> void override;
  auto virtual collectNrt() -> void override;
  auto virtual executeRT() -> int override;
  virtual ~ExecuteRTTest() = default;
  explicit ExecuteRTTest(const std::string& name = "ExecuteRTTest_plan");
  ARIS_DECLARE_BIG_FOUR(ExecuteRTTest);

 private:
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;
};
}  // namespace sire::plan
#endif