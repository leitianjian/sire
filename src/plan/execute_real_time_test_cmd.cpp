#include "sire/plan/execute_real_time_test_cmd.hpp"

#include "sire/core/constants.hpp"
#include "sire/ext/json.hpp"
#include "sire/server/interface.hpp"

namespace sire::plan {
struct ExecuteRTTest::Imp {
  sire::Size count_{5000};
};
auto ExecuteRTTest::prepareNrt() -> void {
  for (const auto& cmd_param : cmdParams()) {
    if (cmd_param.first == "count") {
      int temp = int32Param(cmd_param.first);
      imp_->count_ = temp;
    }
  }
}
auto ExecuteRTTest::executeRT() -> int {
  return count() > int64_t(imp_->count_) ? 0 : 1;
}
auto ExecuteRTTest::collectNrt() -> void {}
ExecuteRTTest::ExecuteRTTest(const std::string& name) : imp_(new Imp) {
  aris::core::fromXmlString(
      command(),
      "<Command name=\"executeRT_test\">"
      "	<GroupParam>"
      "		<Param name=\"count\" abbreviation=\"c\" default=\"5000\"/>"
      "	</GroupParam>"
      "</Command>");
}
ARIS_DEFINE_BIG_FOUR_CPP(ExecuteRTTest);

ARIS_REGISTRATION {
  aris::core::class_<ExecuteRTTest>("ExecuteRTTest")
      .inherit<aris::plan::Plan>();
}
}  // namespace sire::plan