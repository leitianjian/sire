#include "sire/plan/record_get.hpp"

#include <array>
#include <iostream>
#include <tuple>
#include <vector>

#include "sire/core/constants.hpp"
#include "sire/ext/json.hpp"
// #include "sire/ext/tinyxml2.h"
#include "sire/middleware/sire_middleware.hpp"
#include "sire/server/interface.hpp"
#include "sire/simulator/recorder.hpp"

namespace sire::plan {
struct GetParam {
  std::vector<std::vector<double>> part_pq;
  std::vector<std::vector<double>> part_vs;
  std::vector<double> motors_a;
  std::vector<double> motors_f;
  std::vector<double> motors_v;
  std::vector<double> motors_p;
  int state_code;
  bool is_cs_started;
  std::string currentPlan;
  int currentPlanId;
};

auto RecordGet::prepareNrt() -> void {
  option() |=
      NOT_PRINT_CMD_INFO | NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;
  for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
  auto& cs = *controlServer();
  auto& middleware = dynamic_cast<middleware::SireMiddleware&>(cs.middleWare());
  auto& simulator = middleware.simulationLoop();
  auto& recorder = simulator.recorder();
  nlohmann::json output_json;
  recorder.to_json(output_json);

  // 将 JSON 数据写入文件
  if (std::ofstream outfile("record.json"); outfile.is_open()) {
    outfile << output_json.dump(4) << std::endl;
  } else {
    std::cerr << "Fail to open file" << std::endl;
  }

  std::vector<std::pair<std::string, std::any>> out_param;
  ret() = out_param;
}

// auto RecordGet::collectNrt() -> void {}

RecordGet::RecordGet(const std::string& name) {
  aris::core::fromXmlString(command(),
                            "<Command name=\"record_get\">"
                            "</Command>");
}

ARIS_REGISTRATION {
  aris::core::class_<RecordGet>("RecordGet").inherit<aris::plan::Plan>();
}
}  // namespace sire::plan