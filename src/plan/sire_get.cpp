#include "sire/plan/sire_get.hpp"

#include <tuple>

#include "sire/core/constants.hpp"
#include "sire/ext/json.hpp"
#include "sire/middleware/sire_middleware.hpp"
#include "sire/server/interface.hpp"

namespace sire::plan {
struct GetParam {
  std::vector<std::vector<double>> part_pq;  // 怎么获得 body_pq?
  std::vector<std::vector<double>> part_vs;  // 怎么获得 body_pq?
  std::vector<double> motors_a;
  std::vector<double> motors_f;
  std::vector<double> motors_v;
  std::vector<double> motors_p;
  int state_code;
  bool is_cs_started;
  std::string currentPlan;
  int currentPlanId;
};

// TODO(ltj)解决一下高帧率情况下出现的getRtData方法崩溃的问题，本质不能加锁，需要用无锁数据结构把东西发出来
auto SireGet::prepareNrt() -> void {
  option() |=
      NOT_PRINT_CMD_INFO | NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;
  for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
  GetParam par;
  auto& cs = *controlServer();
  auto& middleware = dynamic_cast<middleware::SireMiddleware&>(cs.middleWare());
  auto& simulator = middleware.simulatorBase();

  auto part_pq_only = cmdParams().find("part_pq");
  // 如果只需要part_pq
  if (part_pq_only != cmdParams().end()) {
    par.part_pq.resize(model()->partPool().size());
  } else {
    par.part_pq.resize(model()->partPool().size());
    par.part_vs.resize(model()->partPool().size());
    par.motors_a.resize(model()->motionPool().size(), 0);
    par.motors_v.resize(model()->motionPool().size());
    par.motors_p.resize(model()->motionPool().size());
    par.motors_f.resize(model()->motionPool().size());
  }

  std::any param = par;
  if (part_pq_only != cmdParams().end()) {
    simulator.getModelState(
        [](aris::server::ControlServer& cs, simulator::SimulatorBase& s,
           std::any& data) -> void {
          auto& get_param = std::any_cast<GetParam&>(data);
          auto m = dynamic_cast<aris::dynamic::Model*>(&cs.model());
          std::vector<double> temp_pq(7, 0.0);
          for (sire::Size i(0); i < m->partPool().size(); ++i) {
            m->partPool().at(i).getPq(temp_pq.data());
            get_param.part_pq[i].assign(temp_pq.begin(), temp_pq.end());
          }
        },
        param);
  } else {
    simulator.getModelState(
        [](aris::server::ControlServer& cs, simulator::SimulatorBase&,
           std::any& data) -> void {
          auto& get_param = std::any_cast<GetParam&>(data);
          auto m = dynamic_cast<aris::dynamic::Model*>(&cs.model());
          std::array<double, 7> temp_pq{0.0};
          std::array<double, 6> temp_vs{0.0};
          for (sire::Size i(0); i < m->partPool().size(); ++i) {
            // par.tool->getPq(*par.wobj, std::any_cast<GetParam
            // &>(data).part_pq.data() + i * 7);
            m->partPool().at(i).getPq(temp_pq.data());
            m->partPool().at(i).getVs(temp_vs.data());
            get_param.part_pq[i].assign(temp_pq.begin(), temp_pq.end());
            get_param.part_vs[i].assign(temp_vs.begin(), temp_vs.end());
          }
          for (sire::Size i(0); i < m->motionPool().size(); ++i) {
            get_param.motors_a[i] = m->motionPool()[i].ma();
            get_param.motors_f[i] = m->motionPool()[i].mf();
            get_param.motors_v[i] = m->motionPool()[i].mv();
            get_param.motors_p[i] = m->motionPool()[i].mp();
          }
        },
        param);
  }

  auto out_data = std::any_cast<GetParam&>(param);
  auto& inter =
      dynamic_cast<server::ProgramWebInterface&>(cs.interfacePool().at(0));
  std::vector<std::pair<std::string, std::any>> out_param;
  if (part_pq_only != cmdParams().end()) {
    out_param.push_back(std::make_pair<std::string, std::any>(
        "part_pq", nlohmann::json(out_data.part_pq)));

  } else {
    out_data.is_cs_started = cs.running();
    out_param.push_back(std::make_pair<std::string, std::any>(
        "part_pq", nlohmann::json(out_data.part_pq)));
    out_param.push_back(std::make_pair<std::string, std::any>(
        "part_vs", nlohmann::json(out_data.part_vs)));
    out_param.push_back(std::make_pair<std::string, std::vector<double>>(
        "motors_acc", nlohmann::json(out_data.motors_a)));
    out_param.push_back(std::make_pair<std::string, std::vector<double>>(
        "motors_force", nlohmann::json(out_data.motors_f)));
    out_param.push_back(std::make_pair<std::string, std::vector<double>>(
        "motors_vel", nlohmann::json(out_data.motors_v)));
    out_param.push_back(std::make_pair<std::string, std::vector<double>>(
        "motors_pos", nlohmann::json(out_data.motors_p)));
    out_param.push_back(std::make_pair<std::string, std::any>(
        "state_code", out_data.state_code));
    out_param.push_back(std::make_pair<std::string, std::any>(
        "cs_is_started", out_data.is_cs_started));
    out_param.push_back(std::make_pair<std::string, std::any>(
        "current_plan", out_data.currentPlan));
    out_param.push_back(std::make_pair<std::string, std::any>(
        "current_plan_id", out_data.currentPlanId));
    out_param.push_back(std::make_pair(std::string("cs_err_code"),
                                       std::make_any<int>(cs.errorCode())));
    out_param.push_back(std::make_pair(
        std::string("cs_err_msg"), std::make_any<std::string>(cs.errorMsg())));
    // export const getProErrMsg = state => state.server.pro_err_msg;
    out_param.push_back(std::make_pair<std::string, std::any>(
        "pro_err_msg", inter.lastError()));
    // export const getProErrCode = state => state.server.pro_err_code;
    out_param.push_back(std::make_pair<std::string, std::any>(
        "pro_err_code", inter.lastErrorCode()));
    // export const getProErrMsg = state = > state.server.pro_err_msg;
    out_param.push_back(std::make_pair<std::string, std::any>(
        "pro_err_line", inter.lastErrorLine()));
    // export const getProgramLine = state => state.server.line;
    out_param.push_back(std::make_pair<std::string, std::any>(
        "line", std::get<1>(inter.currentFileLine())));
    // export const getProgramFile = state => state.server.file;
    out_param.push_back(std::make_pair<std::string, std::any>(
        "file", std::get<0>(inter.currentFileLine())));
  }
  ret() = out_param;
  return;
}

auto SireGet::collectNrt() -> void {}

SireGet::SireGet(const std::string& name) {
  aris::core::fromXmlString(command(),
                            "<Command name=\"get\">"
                            "	<GroupParam>"
                            "		<UniqueParam default=\"all\">"
                            "		  <Param name=\"all\"/>"
                            "		  <Param name=\"part_pq\"/>"
                            "		</UniqueParam>"
                            "	</GroupParam>"
                            "</Command>");
}

ARIS_REGISTRATION {
  aris::core::class_<SireGet>("SireGet").inherit<aris::plan::Plan>();
}
}  // namespace sire::plan