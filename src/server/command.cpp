#include "sire/server/command.hpp"

#include "sire/controller/controller_sensor.hpp"
#include "sire/ext/json.hpp"
#include "sire/server/interface.hpp"

#define CHECK_PARAM_STRING                           \
  "		<UniqueParam default=\"check_all\">"            \
  "			<Param name=\"check_all\"/>"                   \
  "			<Param name=\"check_none\"/>"                  \
  "			<GroupParam>"                                  \
  "				<UniqueParam default=\"check_enable\">"       \
  "					<Param name=\"check_enable\"/>"              \
  "					<Param name=\"not_check_enable\"/>"          \
  "				</UniqueParam>"                               \
  "				<UniqueParam default=\"check_pos\">"          \
  "					<Param name=\"check_pos\"/>"                 \
  "					<Param name=\"not_check_pos\"/>"             \
  "					<GroupParam>"                                \
  "						<UniqueParam "                              \
  "default=\"check_pos_max\">"                       \
  "							<Param "                                   \
  "name=\"check_pos_max\"/>"                         \
  "							<Param "                                   \
  "name=\"not_check_pos_max\"/>"                     \
  "						</UniqueParam>"                             \
  "						<UniqueParam "                              \
  "default=\"check_pos_min\">"                       \
  "							<Param "                                   \
  "name=\"check_pos_min\"/>"                         \
  "							<Param "                                   \
  "name=\"not_check_pos_min\"/>"                     \
  "						</UniqueParam>"                             \
  "						<UniqueParam "                              \
  "default=\"check_pos_continuous\">"                \
  "							<Param "                                   \
  "name=\"check_pos_continuous\"/>"                  \
  "							<Param "                                   \
  "name=\"not_check_pos_continuous\"/>"              \
  "						</UniqueParam>"                             \
  "						<UniqueParam "                              \
  "default=\"check_pos_continuous_second_order\">"   \
  "							<Param "                                   \
  "name=\"check_pos_continuous_second_order\"/>"     \
  "							<Param "                                   \
  "name=\"not_check_pos_continuous_second_order\"/>" \
  "						</UniqueParam>"                             \
  "						<UniqueParam "                              \
  "default=\"check_pos_following_error\">"           \
  "							<Param "                                   \
  "name=\"check_pos_following_error\"/>"             \
  "							<Param "                                   \
  "name=\"not_check_pos_following_error\"/>"         \
  "						</UniqueParam>"                             \
  "					</GroupParam>"                               \
  "				</UniqueParam>"                               \
  "				<UniqueParam default=\"check_vel\">"          \
  "					<Param name=\"check_vel\"/>"                 \
  "					<Param name=\"not_check_vel\"/>"             \
  "					<GroupParam>"                                \
  "						<UniqueParam "                              \
  "default=\"check_vel_max\">"                       \
  "							<Param "                                   \
  "name=\"check_vel_max\"/>"                         \
  "							<Param "                                   \
  "name=\"not_check_vel_max\"/>"                     \
  "						</UniqueParam>"                             \
  "						<UniqueParam "                              \
  "default=\"check_vel_min\">"                       \
  "							<Param "                                   \
  "name=\"check_vel_min\"/>"                         \
  "							<Param "                                   \
  "name=\"not_check_vel_min\"/>"                     \
  "						</UniqueParam>"                             \
  "						<UniqueParam "                              \
  "default=\"check_vel_continuous\">"                \
  "							<Param "                                   \
  "name=\"check_vel_continuous\"/>"                  \
  "							<Param "                                   \
  "name=\"not_check_vel_continuous\"/>"              \
  "						</UniqueParam>"                             \
  "						<UniqueParam "                              \
  "default=\"check_vel_following_error\">"           \
  "							<Param "                                   \
  "name=\"check_vel_following_error\"/>"             \
  "							<Param "                                   \
  "name=\"not_check_vel_following_error\"/>"         \
  "						</UniqueParam>"                             \
  "					</GroupParam>"                               \
  "				</UniqueParam>"                               \
  "			</GroupParam>"                                 \
  "		</UniqueParam>"

namespace sire::server {
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

  // end_pq, end_pe, motion_pos, motion_vel, motion_acc, motion_toq, ai,
  // forcedata, forceoffset;
  /*std::vector<bool> digtal_in, digtal_out;
  std::int32_t state_code;
  aris::control::EthercatController::SlaveLinkState sls[7];
  aris::control::EthercatController::MasterLinkState mls{};
  std::vector<int> motion_state;
  std::string currentplan;
  int vel_percent;
  aris::dynamic::Marker* tool, * wobj;
  bool is_dragging, teachingmode;*/
};

auto get_state_code(aris::server::ControlServer& cs, ProgramWebInterface& inter)
    -> int {
  bool isEnable = true;
  auto& motor_pool = cs.controller().motorPool();
  for (int i = 0; i < motor_pool.size(); ++i) {
    if ((motor_pool[i].statusWord() & 0x6f) == 0x27) isEnable = false;
  }
  if (isEnable) {
    return 100;
  } else {
    if (inter.isAutoMode()) {
      if (inter.isAutoRunning()) return 400;
      if (inter.isAutoPaused()) return 410;
      if (inter.isAutoStopped()) return 420;
      return 300;
    } else {
      return 200;
    }
  }
  return 0;
}

auto Get::prepareNrt() -> void {
  option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_PRINT_CMD_INFO;
  // option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;
  for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
  GetParam par;
  par.part_pq.resize(model()->partPool().size());
  par.part_vs.resize(model()->partPool().size());
  par.motors_a.resize(model()->motionPool().size());
  par.motors_v.resize(model()->motionPool().size());
  par.motors_p.resize(model()->motionPool().size());
  par.motors_f.resize(model()->motionPool().size());
  std::any param = par;
  auto& cs = *controlServer();
  if (cs.running()) {
    cs.getRtData(
        [](aris::server::ControlServer& cs, const aris::plan::Plan* target,
           std::any& data) -> void {
          auto& get_param = std::any_cast<GetParam&>(data);
          auto m = dynamic_cast<aris::dynamic::Model*>(&cs.model());
          std::vector<double> temp_pq(7, 0.0);
          std::vector<double> temp_vs(6, 0.0);
          for (aris::Size i(0); i < m->partPool().size(); ++i) {
            // par.tool->getPq(*par.wobj, std::any_cast<GetParam
            // &>(data).part_pq.data() + i * 7);
            m->partPool().at(i).getPq(temp_pq.data());
            m->partPool().at(i).getVs(temp_vs.data());
            get_param.part_pq[i].assign(temp_pq.begin(), temp_pq.end());
            get_param.part_vs[i].assign(temp_vs.begin(), temp_vs.end());
          }
          for (aris::Size i(0); i < m->motionPool().size(); ++i) {
            get_param.motors_a[i] = m->motionPool()[i].ma();
            get_param.motors_f[i] = m->motionPool()[i].mf();
            get_param.motors_v[i] = m->motionPool()[i].mv();
            get_param.motors_p[i] = m->motionPool()[i].mp();
          }
          if (target == nullptr) {
            get_param.currentPlan = "none";
            get_param.currentPlanId = -1;
          } else {
            get_param.currentPlan = target->command().name();
            get_param.currentPlanId =
                const_cast<aris::plan::Plan*>(target)->cmdId();
          }
        },
        param);
  }
  auto out_data = std::any_cast<GetParam&>(param);

  auto& inter = dynamic_cast<ProgramWebInterface&>(cs.interfacePool().at(0));

  std::vector<std::pair<std::string, std::any>> out_param;
  out_data.state_code = get_state_code(cs, inter);
  out_data.is_cs_started = cs.running();

  // out_param.push_back(std::make_pair<std::string, std::any>("part_pq",
  // std::make_any<nlohmann::json>(std::move(js))));
  // out_param.push_back(std::make_pair<std::string, std::any>("part_pq",
  // js.dump(2)));
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
  out_param.push_back(
      std::make_pair<std::string, std::any>("state_code", out_data.state_code));
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
  out_param.push_back(
      std::make_pair<std::string, std::any>("pro_err_msg", inter.lastError()));
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

  ret() = out_param;
}

auto Get::collectNrt() -> void {}

Get::Get(const std::string& name) {
  aris::core::fromXmlString(
      command(),
      "<Command name=\"get\">"
      "	<GroupParam>"
      "		<Param name=\"tool\" default=\"tool0\"/>"
      "		<Param name=\"wobj\" default=\"wobj0\"/>"
      "	</GroupParam>"
      "</Command>");
}

struct ForceDataConfig {};
struct ForceDataContainer {
  std::vector<std::vector<std::unique_ptr<aris::control::SensorData>>>
      data_sensor_force_;
  ForceDataContainer(aris::Size sensor_size = 6)
      : data_sensor_force_(sensor_size) {}
};
struct DataProperty {
  std::string name;
  std::string description;
  std::string unit;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(DataProperty, name, description);
};
// steam of SensorData
struct StreamStructure {
  std::string name;
  std::string description;
  std::vector<DataProperty> properties;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(StreamStructure, name, description,
                                 properties);
};
struct ForceDataRetStructure {
  std::string name;
  std::string description;
  std::vector<StreamStructure> data_streams;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(ForceDataRetStructure, name, description,
                                 data_streams);
};
auto GetForceSensorData::prepareNrt() -> void {
  option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_PRINT_CMD_INFO;
  for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
  bool show_info = true;
  bool show_config = false;
  for (const auto& cmd_param : cmdParams()) {
    if (cmd_param.first == "info") {
      show_info = true;
      show_config = false;
    }
    if (cmd_param.first == "config") {
      show_info = false;
      show_config = true;
    }
  }
  auto& sensor_pool = controller()->sensorPool();
  aris::Size sensor_pool_size = sensor_pool.size();
  aris::Size num_sensor_data_field = 1;
  if (show_info) {
    ForceDataContainer param;
    param.data_sensor_force_.resize(sensor_pool_size);
    std::vector<std::vector<double>> result_force(sensor_pool_size);
    for (int i = 0; i < sensor_pool_size; ++i) {
      try {
        aris::Size data_count = 0;
        auto& virtual_force_sensor =
            dynamic_cast<controller::BufferedMotorForceVirtualSensor&>(
                sensor_pool.at(i));
        param.data_sensor_force_[i].resize(virtual_force_sensor.bufferSize());
        virtual_force_sensor.retrieveBufferData(param.data_sensor_force_[i],
                                                data_count);
        result_force[i].resize(data_count);
        for (int j = 0; j < data_count; ++j) {
          result_force[i][j] = static_cast<controller::MotorForceData*>(
                                   param.data_sensor_force_[i][j].release())
                                   ->force_;
        }
      } catch (std::bad_cast& err) {
        std::cout << "sensor at " << i << " is not a virtual force sensor"
                  << std::endl;
        continue;
      }
    }
    std::vector<std::pair<std::string, std::any>> out_param;
    out_param.push_back(std::make_pair<std::string, std::any>(
        "motors_force", nlohmann::json(result_force)));
    ret() = out_param;
    return;
  }
  if (show_config) {
    auto& retStructure = ForceDataRetStructure();
    retStructure.name = "Force data structure";
    retStructure.description = "motor virtual force sensor data streams";
    retStructure.data_streams.resize(sensor_pool_size);
    for (int i = 0; i < sensor_pool_size; ++i) {
      auto& data_stream = StreamStructure();
      data_stream.description = "stream of sensor data";
      data_stream.name = "stream";
      data_stream.properties.resize(num_sensor_data_field);
      for (int j = 0; j < num_sensor_data_field; ++j) {
        auto& dataProperty = DataProperty();
        dataProperty.name = "force";
        dataProperty.description = "force magnitude";
        data_stream.properties[j] = dataProperty;
      }
      retStructure.data_streams[i] = data_stream;
    }
    std::vector<std::pair<std::string, std::any>> out_param;
    out_param.push_back(std::make_pair<std::string, std::any>(
        "motors_force_stream_config", nlohmann::json(retStructure)));
    ret() = out_param;
    return;
  }
}

auto GetForceSensorData::collectNrt() -> void {}

GetForceSensorData::GetForceSensorData(const std::string& name) {
  aris::core::fromXmlString(command(),
                            "<Command name=\"get_force\">"
                            "  <UniqueParam default=\"info\">"
                            "    <Param name=\"info\" abbreviation=\"i\"/>"
                            "    <Param name=\"config\" abbreviation=\"c\"/>"
                            "  </UniqueParam>"
                            "</Command>");
}

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

auto set_check_option(
    const std::map<std::string_view, std::string_view>& cmd_params,
    aris::plan::Plan& plan) -> void {
  for (const auto& cmd_param : cmd_params) {
    if (cmd_param.first == "check_all") {
      for (auto& option : plan.motorOptions())
        option &= ~(aris::plan::Plan::NOT_CHECK_POS_MIN |
                    aris::plan::Plan::NOT_CHECK_POS_MAX |
                    aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS |
                    aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
                    aris::plan::Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
                    aris::plan::Plan::NOT_CHECK_VEL_MIN |
                    aris::plan::Plan::NOT_CHECK_VEL_MAX |
                    aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS |
                    aris::plan::Plan::NOT_CHECK_VEL_FOLLOWING_ERROR);
    } else if (cmd_param.first == "check_none") {
      for (auto& option : plan.motorOptions())
        option |= aris::plan::Plan::NOT_CHECK_POS_MIN |
                  aris::plan::Plan::NOT_CHECK_POS_MAX |
                  aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS |
                  aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
                  aris::plan::Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
                  aris::plan::Plan::NOT_CHECK_VEL_MIN |
                  aris::plan::Plan::NOT_CHECK_VEL_MAX |
                  aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS |
                  aris::plan::Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
    } else if (cmd_param.first == "check_enable") {
      for (auto& option : plan.motorOptions())
        option &= ~(aris::plan::Plan::NOT_CHECK_ENABLE);
    } else if (cmd_param.first == "not_check_enable") {
      for (auto& option : plan.motorOptions())
        option |= aris::plan::Plan::NOT_CHECK_ENABLE;
    } else if (cmd_param.first == "check_pos") {
      for (auto& option : plan.motorOptions())
        option &= ~(aris::plan::Plan::NOT_CHECK_POS_MIN |
                    aris::plan::Plan::NOT_CHECK_POS_MAX |
                    aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS |
                    aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
                    aris::plan::Plan::NOT_CHECK_POS_FOLLOWING_ERROR);
    } else if (cmd_param.first == "not_check_pos") {
      for (auto& option : plan.motorOptions())
        option |= aris::plan::Plan::NOT_CHECK_POS_MIN |
                  aris::plan::Plan::NOT_CHECK_POS_MAX |
                  aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS |
                  aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
                  aris::plan::Plan::NOT_CHECK_POS_FOLLOWING_ERROR;
    } else if (cmd_param.first == "check_vel") {
      for (auto& option : plan.motorOptions())
        option &= ~(aris::plan::Plan::NOT_CHECK_VEL_MIN |
                    aris::plan::Plan::NOT_CHECK_VEL_MAX |
                    aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS |
                    aris::plan::Plan::NOT_CHECK_VEL_FOLLOWING_ERROR);
    } else if (cmd_param.first == "not_check_vel") {
      for (auto& option : plan.motorOptions())
        option |= aris::plan::Plan::NOT_CHECK_VEL_MIN |
                  aris::plan::Plan::NOT_CHECK_VEL_MAX |
                  aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS |
                  aris::plan::Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
    } else if (cmd_param.first == "check_pos_min") {
      for (auto& option : plan.motorOptions())
        option &= ~aris::plan::Plan::NOT_CHECK_POS_MIN;
    } else if (cmd_param.first == "not_check_pos_min") {
      for (auto& option : plan.motorOptions())
        option |= aris::plan::Plan::NOT_CHECK_POS_MIN;
    } else if (cmd_param.first == "check_pos_max") {
      for (auto& option : plan.motorOptions())
        option &= ~aris::plan::Plan::NOT_CHECK_POS_MAX;
    } else if (cmd_param.first == "not_check_pos_max") {
      for (auto& option : plan.motorOptions())
        option |= aris::plan::Plan::NOT_CHECK_POS_MAX;
    } else if (cmd_param.first == "check_pos_continuous") {
      for (auto& option : plan.motorOptions())
        option &= ~aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS;
    } else if (cmd_param.first == "not_check_pos_continuous") {
      for (auto& option : plan.motorOptions())
        option |= aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS;
    } else if (cmd_param.first == "check_pos_continuous_second_order") {
      for (auto& option : plan.motorOptions())
        option &= ~aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
    } else if (cmd_param.first == "not_check_pos_continuous_second_order") {
      for (auto& option : plan.motorOptions())
        option |= aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
    } else if (cmd_param.first == "check_pos_following_error") {
      for (auto& option : plan.motorOptions())
        option &= ~aris::plan::Plan::NOT_CHECK_POS_FOLLOWING_ERROR;
    } else if (cmd_param.first == "not_check_pos_following_error") {
      for (auto& option : plan.motorOptions())
        option |= aris::plan::Plan::NOT_CHECK_POS_FOLLOWING_ERROR;
    } else if (cmd_param.first == "check_vel_min") {
      for (auto& option : plan.motorOptions())
        option &= ~aris::plan::Plan::NOT_CHECK_VEL_MIN;
    } else if (cmd_param.first == "not_check_vel_min") {
      for (auto& option : plan.motorOptions())
        option |= aris::plan::Plan::NOT_CHECK_VEL_MIN;
    } else if (cmd_param.first == "check_vel_max") {
      for (auto& option : plan.motorOptions())
        option &= ~aris::plan::Plan::NOT_CHECK_VEL_MAX;
    } else if (cmd_param.first == "not_check_vel_max") {
      for (auto& option : plan.motorOptions())
        option |= aris::plan::Plan::NOT_CHECK_VEL_MAX;
    } else if (cmd_param.first == "check_vel_continuous") {
      for (auto& option : plan.motorOptions())
        option &= ~aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS;
    } else if (cmd_param.first == "not_check_vel_continuous") {
      for (auto& option : plan.motorOptions())
        option |= aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS;
    } else if (cmd_param.first == "check_vel_following_error") {
      for (auto& option : plan.motorOptions())
        option &= ~aris::plan::Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
    } else if (cmd_param.first == "not_check_vel_following_error") {
      for (auto& option : plan.motorOptions())
        option |= aris::plan::Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
    }
  }
}

auto check_eul_validity(std::string_view eul_type) -> bool {
  if (eul_type.size() < 3) return false;

  for (int i = 0; i < 3; ++i)
    if (eul_type[i] > '3' || eul_type[i] < '1') return false;

  if (eul_type[0] == eul_type[1] || eul_type[1] == eul_type[2]) return false;

  return true;
}

auto find_pq(const std::map<std::string_view, std::string_view>& params,
             aris::plan::Plan& plan, double* pq_out) -> bool {
  double pos_unit;
  auto pos_unit_found = params.find("pos_unit");
  if (pos_unit_found == params.end())
    pos_unit = 1.0;
  else if (pos_unit_found->second == "m")
    pos_unit = 1.0;
  else if (pos_unit_found->second == "mm")
    pos_unit = 0.001;
  else if (pos_unit_found->second == "cm")
    pos_unit = 0.01;
  else
    THROW_FILE_LINE("");

  for (const auto& cmd_param : params) {
    if (cmd_param.first == "pq") {
      auto pq_mat = plan.matrixParam(cmd_param.first);
      if (pq_mat.size() != 7) THROW_FILE_LINE("");
      aris::dynamic::s_vc(7, pq_mat.data(), pq_out);
      aris::dynamic::s_nv(3, pos_unit, pq_out);
      return true;
    } else if (cmd_param.first == "pm") {
      auto pm_mat = plan.matrixParam(cmd_param.first);
      if (pm_mat.size() != 16) THROW_FILE_LINE("");
      aris::dynamic::s_pm2pq(pm_mat.data(), pq_out);
      aris::dynamic::s_nv(3, pos_unit, pq_out);
      return true;
    } else if (cmd_param.first == "pe") {
      double ori_unit;
      auto ori_unit_found = params.find("ori_unit");
      if (ori_unit_found == params.end())
        ori_unit = 1.0;
      else if (ori_unit_found->second == "rad")
        ori_unit = 1.0;
      else if (ori_unit_found->second == "degree")
        ori_unit = aris::PI / 180.0;
      else
        THROW_FILE_LINE("");

      std::string eul_type;
      auto eul_type_found = params.find("eul_type");
      if (eul_type_found == params.end())
        eul_type = "321";
      else if (check_eul_validity(eul_type_found->second.data()))
        eul_type = eul_type_found->second;
      else
        THROW_FILE_LINE("");

      auto pe_mat = plan.matrixParam(cmd_param.first);
      if (pe_mat.size() != 6) THROW_FILE_LINE("");
      aris::dynamic::s_nv(3, ori_unit, pe_mat.data() + 3);
      aris::dynamic::s_pe2pq(pe_mat.data(), pq_out, eul_type.data());
      aris::dynamic::s_nv(3, pos_unit, pq_out);
      return true;
    }
  }

  THROW_FILE_LINE("No pose input");
}

struct SireMoveJParam {
  std::vector<double> joint_vel, joint_acc, joint_dec, ee_pq, joint_pos_begin,
      joint_pos_end;
  std::vector<aris::Size> total_count;
};
struct SireMoveJ::Imp {};
auto SireMoveJ::prepareNrt() -> void {
  set_check_option(cmdParams(), *this);

  SireMoveJParam mvj_param;

  // find ee pq //
  mvj_param.ee_pq.resize(7);
  find_pq(cmdParams(), *this, mvj_param.ee_pq.data());

  mvj_param.joint_pos_begin.resize(model()->motionPool().size(), 0.0);
  mvj_param.joint_pos_end.resize(model()->motionPool().size(), 0.0);
  mvj_param.total_count.resize(model()->motionPool().size(), 0);

  // find joint acc/vel/dec/
  for (const auto& cmd_param : cmdParams()) {
    auto c = controller();
    if (cmd_param.first == "joint_acc") {
      mvj_param.joint_acc.clear();
      mvj_param.joint_acc.resize(model()->motionPool().size(), 0.0);

      auto acc_mat = matrixParam(cmd_param.first);
      if (acc_mat.size() == 1)
        std::fill(mvj_param.joint_acc.begin(), mvj_param.joint_acc.end(),
                  acc_mat.toDouble());
      else if (acc_mat.size() == model()->motionPool().size())
        std::copy(acc_mat.begin(), acc_mat.end(), mvj_param.joint_acc.begin());
      else
        THROW_FILE_LINE("");

      for (int i = 0; i < controller()->motorPool().size(); ++i)
        mvj_param.joint_acc[i] *= controller()->motorPool()[i].maxAcc();

      // check value validity //
      for (aris::Size i = 0;
           i < std::min(model()->motionPool().size(), c->motorPool().size());
           ++i)
        if (mvj_param.joint_acc[i] <= 0 ||
            mvj_param.joint_acc[i] > c->motorPool()[i].maxAcc())
          THROW_FILE_LINE("");
    } else if (cmd_param.first == "joint_vel") {
      mvj_param.joint_vel.clear();
      mvj_param.joint_vel.resize(model()->motionPool().size(), 0.0);

      auto vel_mat = matrixParam(cmd_param.first);
      if (vel_mat.size() == 1)
        std::fill(mvj_param.joint_vel.begin(), mvj_param.joint_vel.end(),
                  vel_mat.toDouble());
      else if (vel_mat.size() == model()->motionPool().size())
        std::copy(vel_mat.begin(), vel_mat.end(), mvj_param.joint_vel.begin());
      else
        THROW_FILE_LINE("");

      for (int i = 0; i < controller()->motorPool().size(); ++i)
        mvj_param.joint_vel[i] *= controller()->motorPool()[i].maxVel();

      // check value validity //
      for (aris::Size i = 0;
           i < std::min(model()->motionPool().size(), c->motorPool().size());
           ++i)
        if (mvj_param.joint_vel[i] <= 0 ||
            mvj_param.joint_vel[i] > c->motorPool()[i].maxVel())
          THROW_FILE_LINE("");
    } else if (cmd_param.first == "joint_dec") {
      mvj_param.joint_dec.clear();
      mvj_param.joint_dec.resize(model()->motionPool().size(), 0.0);

      auto dec_mat = matrixParam(cmd_param.first);
      if (dec_mat.size() == 1)
        std::fill(mvj_param.joint_dec.begin(), mvj_param.joint_dec.end(),
                  dec_mat.toDouble());
      else if (dec_mat.size() == model()->motionPool().size())
        std::copy(dec_mat.begin(), dec_mat.end(), mvj_param.joint_dec.begin());
      else
        THROW_FILE_LINE("");

      for (int i = 0; i < controller()->motorPool().size(); ++i)
        mvj_param.joint_dec[i] *= controller()->motorPool()[i].maxAcc();

      // check value validity //
      for (aris::Size i = 0;
           i < std::min(model()->motionPool().size(), c->motorPool().size());
           ++i)
        if (mvj_param.joint_dec[i] <= 0 ||
            mvj_param.joint_dec[i] > c->motorPool()[i].maxAcc())
          THROW_FILE_LINE("");
    }
  }

  this->param() = mvj_param;

  for (auto& option : motorOptions())
    option |= aris::plan::Plan::USE_TARGET_POS;

  std::vector<std::pair<std::string, std::any>> ret_value;
  ret() = ret_value;
}
auto SireMoveJ::executeRT() -> int {
  auto mvj_param = std::any_cast<SireMoveJParam>(&this->param());

  // 取得起始位置 //
  double p, v, a;
  static aris::Size max_total_count;
  if (count() == 1) {
    // begin pos //
    model()->getInputPos(mvj_param->joint_pos_begin.data());

    // inverse kinematic //
    auto& gm = model()->generalMotionPool().at(0);
    double end_pe321[6]{0.0};
    aris::dynamic::s_pq2pe(mvj_param->ee_pq.data(), end_pe321, "321");

    gm.setP(end_pe321);
    if (model()->solverPool().at(0).kinPos()) return -1;

    // compute max count //
    for (aris::Size i = 0; i < std::min(controller()->motorPool().size(),
                                        model()->motionPool().size());
         ++i) {
      mvj_param->joint_pos_end[i] = *model()->motionPool()[i].p();
      aris::plan::moveAbsolute(
          static_cast<double>(count()), mvj_param->joint_pos_begin[i],
          mvj_param->joint_pos_end[i], mvj_param->joint_vel[i] / 1000,
          mvj_param->joint_acc[i] / 1000 / 1000,
          mvj_param->joint_dec[i] / 1000 / 1000, p, v, a,
          mvj_param->total_count[i]);
    }

    max_total_count = *std::max_element(mvj_param->total_count.begin(),
                                        mvj_param->total_count.end());
  }

  for (aris::Size i = 0; i < std::min(controller()->motorPool().size(),
                                      model()->motionPool().size());
       ++i) {
    aris::plan::moveAbsolute(
        static_cast<double>(count()) * mvj_param->total_count[i] /
            max_total_count,
        mvj_param->joint_pos_begin[i], mvj_param->joint_pos_end[i],
        mvj_param->joint_vel[i] / 1000, mvj_param->joint_acc[i] / 1000 / 1000,
        mvj_param->joint_dec[i] / 1000 / 1000, p, v, a,
        mvj_param->total_count[i]);

    model()->setInputPosAt(p, i);
    model()->setInputVelAt(v, i);
    model()->setInputAccAt(a, i);
    model()->forwardKinematics();
    model()->forwardKinematicsVel();
    model()->forwardKinematicsAcc();
  }

  return max_total_count == 0 ? 0 : static_cast<int>(max_total_count - count());
}
SireMoveJ::~SireMoveJ() = default;
SireMoveJ::SireMoveJ(const std::string& name) : imp_(new Imp) {
  aris::core::fromXmlString(
      command(),
      "<Command name=\"sire_mvj\">"
      "	<GroupParam>"
      "		<Param name=\"pos_unit\" default=\"m\"/>"
      "		<UniqueParam default=\"pq\">"
      "			<Param name=\"pq\" default=\"{0,0,0,0,0,0,1}\"/>"
      "			<Param name=\"pm\" "
      "default=\"{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}\"/>"
      "			<GroupParam>"
      "				<Param name=\"pe\" default=\"{0,0,0,0,0,0}\"/>"
      "				<Param name=\"ori_unit\" default=\"rad\"/>"
      "				<Param name=\"eul_type\" default=\"321\"/>"
      "			</GroupParam>"
      "		</UniqueParam>"
      "		<Param name=\"joint_acc\" default=\"0.1\"/>"
      "		<Param name=\"joint_vel\" default=\"0.1\"/>"
      "		<Param name=\"joint_dec\" default=\"0.1\"/>" CHECK_PARAM_STRING
      "	</GroupParam>"
      "</Command>");
}
ARIS_DEFINE_BIG_FOUR_CPP(SireMoveJ);

ARIS_REGISTRATION {
  aris::core::class_<Get>("SireGet").inherit<aris::plan::Plan>();
  aris::core::class_<ForceControlTest>("SireForceControlTest")
      .inherit<aris::plan::Plan>();
  aris::core::class_<SireMoveJ>("SireMoveJ").inherit<aris::plan::Plan>();
  aris::core::class_<GetForceSensorData>("SireGetForceSensorData")
      .inherit<aris::plan::Plan>();
}
}  // namespace sire::server