#include "sire/plan/fce_control_test_cmd.hpp"

#include "sire/controller/controller_sensor.hpp"
#include "sire/ext/json.hpp"
#include "sire/server/interface.hpp"

namespace sire::plan {
struct ForceControlTest::Imp {};
auto ForceControlTest::prepareNrt() -> void {
  for (auto& option : motorOptions())
    option |= aris::plan::Plan::USE_TARGET_TOQ;
}
auto ForceControlTest::executeRT() -> int {
  std::vector<double> force_data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  model()->setInputFce(force_data.data());
  return count() > 5000 ? 0 : 1;
}
auto ForceControlTest::collectNrt() -> void {}
ForceControlTest::ForceControlTest(const std::string& name) : imp_(new Imp) {
  aris::core::fromXmlString(command(),
                            "<Command name=\"FCTest\">"
                            "</Command>");
}
ARIS_DEFINE_BIG_FOUR_CPP(ForceControlTest);

ARIS_REGISTRATION {
  aris::core::class_<ForceControlTest>("SireForceControlTest")
      .inherit<aris::plan::Plan>();
}
}  // namespace sire::plan