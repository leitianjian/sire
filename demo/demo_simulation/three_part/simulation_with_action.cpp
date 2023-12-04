#include "./simulation_with_action.hpp"

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
struct SimulationWithAction::Imp {
  sire::Size frame_skip_{25};
};
auto SimulationWithAction::prepareNrt() -> void {
  option() |=
      NOT_PRINT_CMD_INFO | NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;
  for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
  GetParam par;
  par.part_pq.resize(model()->partPool().size());
  par.part_vs.resize(model()->partPool().size());
  par.motors_a.resize(model()->motionPool().size(), 0);
  par.motors_v.resize(model()->motionPool().size());
  par.motors_p.resize(model()->motionPool().size());
  par.motors_f.resize(model()->motionPool().size());
  par.general_motion_p.resize(model()->generalMotionPool().size());
  par.general_motion_v.resize(model()->generalMotionPool().size());
  auto& cs = *controlServer();
  auto& middleware = dynamic_cast<middleware::SireMiddleware&>(cs.middleWare());
  auto& simulator = middleware.simulatorBase();
  for (const auto& cmd_param : cmdParams()) {
    if (cmd_param.first == "action") {
      par.motors_f.clear();
      par.motors_f.resize(model()->motionPool().size(), 0.0);
      auto temp = matrixParam(cmd_param.first);
      if (temp.size() == 1)
        std::fill(par.motors_f.begin(), par.motors_f.end(), temp.toDouble());
      else if (temp.size() == model()->motionPool().size())
        std::copy(temp.begin(), temp.end(), par.motors_f.begin());
      else
        THROW_FILE_LINE("");

    } else if (cmd_param.first == "frame_skip") {
      sire::Size temp = uint64Param(cmd_param.first);
      imp_->frame_skip_ = temp;
    }
  }
  for (sire::Size i = 0; i < model()->motionPool().size(); ++i) {
    dynamic_cast<aris::dynamic::SingleComponentForce&>(
        model()->forcePool().at(i))
        .setFce(par.motors_f[i]);
  }
  simulator.step(imp_->frame_skip_);

  for (std::size_t i = 0; i < model()->generalMotionPool().size(); ++i) {
    auto& general_motion = model()->generalMotionPool().at(i);
    // general_motion.updP();
    std::array<double, 6> temp_p{0.0}, temp_v{0.0};
    general_motion.getP(temp_p.data());
    general_motion.getV(temp_v.data());
    par.general_motion_p[i].assign(temp_p.begin(), temp_p.end());
    par.general_motion_v[i].assign(temp_v.begin(), temp_v.end());
  }
  auto& inter =
      dynamic_cast<server::ProgramWebInterface&>(cs.interfacePool().at(0));
  std::vector<std::pair<std::string, std::any>> out_param;
  out_param.push_back(std::make_pair<std::string, std::any>(
      "general_motion_p", nlohmann::json(par.general_motion_p)));
  out_param.push_back(std::make_pair<std::string, std::any>(
      "general_motion_v", nlohmann::json(par.general_motion_v)));
  ret() = out_param;
  return;
}
SimulationWithAction::SimulationWithAction(const std::string& name)
    : imp_(new Imp) {
  aris::core::fromXmlString(
      command(),
      "<Command name=\"sim_act\">"
      "	<GroupParam>"
      "  <Param name=\"action\" abbreviation=\"a\" default=\"{0,0}\"/>"
      "	 <Param name=\"frame_skip\" abbreviation=\"s\" default=\"25\"/>"
      "	</GroupParam>"
      "</Command>");
}
ARIS_DEFINE_BIG_FOUR_CPP(SimulationWithAction);

ARIS_REGISTRATION {
  aris::core::class_<SimulationWithAction>("SimulationWithAction")
      .inherit<aris::plan::Plan>();
}
}  // namespace sire::plan