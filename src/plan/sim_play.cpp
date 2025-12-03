#include "sire/plan/sim_play.hpp"

#include <random>

#include "sire/core/constants.hpp"
#include "sire/ext/json.hpp"
#include "sire/middleware/sire_middleware.hpp"
#include "sire/server/interface.hpp"

namespace sire::plan {
struct GetParam {
  std::vector<std::vector<double>> part_pq;  // 怎么获得 body_pq?
  std::vector<std::vector<double>> part_vs;  // 怎么获得 body_pq?
  std::vector<double> motors_v;
  std::vector<double> motors_p;
  std::vector<double> motors_a;
  std::vector<double> motors_f;
  std::vector<std::vector<double>> general_motion_p;
  std::vector<std::vector<double>> general_motion_v;
  int state_code;
  bool is_cs_started;
  std::string currentPlan;
  int currentPlanId;
};
struct SimPlay::Imp {
  uint32_t frame_skip_{25};
};
auto SimPlay::prepareNrt() -> void {
  option() |=
      NOT_PRINT_CMD_INFO | NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;
  for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
  auto& cs = *controlServer();
  auto& middleware = dynamic_cast<middleware::SireMiddleware&>(cs.middleWare());
  auto& simulator = middleware.simulationLoop();
  if (!simulator.isRunning()) {
    simulator.start();
  }
  return;
}
SimPlay::~SimPlay() = default;
SimPlay::SimPlay(const std::string& name) : imp_(new Imp) {
  aris::core::fromXmlString(command(),
                            "<Command name=\"sim_play\">"
                            "</Command>");
}
ARIS_DEFINE_BIG_FOUR_CPP(SimPlay);

ARIS_REGISTRATION {
  aris::core::class_<SimPlay>("SimPlay").inherit<aris::plan::Plan>();
}
}  // namespace sire::plan