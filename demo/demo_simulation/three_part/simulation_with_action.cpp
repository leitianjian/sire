#include "./simulation_with_action.hpp"

#include "sire/core/constants.hpp"
#include "sire/ext/json.hpp"
#include "sire/middleware/sire_middleware.hpp"
#include "sire/server/interface.hpp"

#include <random>

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
  uint32_t frame_skip_{25};
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
  par.motors_f.resize(model()->motionPool().size()); // size = 2
  par.general_motion_p.resize(model()->generalMotionPool().size());
  par.general_motion_v.resize(model()->generalMotionPool().size()); // size = 1
  auto& cs = *controlServer();
  auto& middleware = dynamic_cast<middleware::SireMiddleware&>(cs.middleWare());
  auto& simulator = middleware.simulationLoop();
  uint32_t reset_flag = 0;
  uint32_t delay = 0;
  std::vector<double> position;
  // random generator
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> uniform(0.9, 1.1);
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
      imp_->frame_skip_ = (uint32_t) temp;

    } else if (cmd_param.first == "reset") {
      reset_flag = uint32Param(cmd_param.first);
    
    } else if (cmd_param.first == "delay") {
      delay = std::min(uint32Param(cmd_param.first), imp_->frame_skip_);

    } else if (cmd_param.first == "position") {
      position.clear();
      position.resize(
          model()->motionPool().size() + model()->jointPool().size(), 0.0);
      auto temp = matrixParam(cmd_param.first);
      if (temp.size() == 1)
        std::fill(position.begin(), position.end(), temp.toDouble());
      else 
        std::copy(temp.begin(), temp.end(), position.begin());
    }
  }
  // reset
  if (reset_flag > 0) {
    auto xmlpath = std::filesystem::absolute(".");  // 获取当前工程所在的路径
    const std::string xmlfile = "model.xml";
    //auto& cs = aris::server::ControlServer::instance();
    xmlpath = xmlpath / xmlfile;
    aris::core::fromXmlFile(*model(), xmlpath);
    model()->init();

    // dynamics randomization
    if (reset_flag == 2) {
      for (std::size_t i = 1; i < model()->partPool().size(); ++i) {
        const double* inertial = model()->partPool().at(i).prtIv();
        // print
        std::cout << "init: ";
        for (int i = 0; i < 10; i++) {
          std::cout << inertial[i] << " ";
        }
        std::cout << std::endl;
        // random
        const double rand_inertial[10] = {
            uniform(gen) * inertial[0], 0, 0, 0, 0, 0,
            uniform(gen) * inertial[6], 0, 0, 0};
        model()->partPool().at(i).setPrtIv(rand_inertial);
        model()->init();
        // print
        std::cout << "rand: ";
        for (int i = 0; i < 10; i++) {
          std::cout << inertial[i] << " ";
        }
        std::cout << std::endl << std::endl;
      }
    }

    // 被动关节角度随机化
    double joint_pos[6] = {0, 0, 0, 0, 0, position[0]};
    model()->jointPool().at(0).makI()->setPe(
        *model()->jointPool().at(0).makJ(), joint_pos, "123");
    // 主动关节角度随机化
    model()->motionPool().at(0).setMp(position[1]);
    model()->motionPool().at(1).setMp(position[2]);
    model()->forwardKinematics();
    // imp_->frame_skip_ = 0;                          // reset时设置仿真0步
    
  } else {                                          // reset时不设置force
    simulator.step(delay);  // 设置延迟
    for (sire::Size i = 0; i < model()->motionPool().size(); ++i) {
      dynamic_cast<aris::dynamic::SingleComponentForce&>(
          model()->forcePool().at(i))
          .setFce(par.motors_f[i]);
    }
    simulator.step(imp_->frame_skip_ - delay);
  }

  for (std::size_t i = 0; i < model()->generalMotionPool().size(); ++i) {
    auto& general_motion = model()->generalMotionPool().at(i);
    // update general_motion
    general_motion.updP();  
    // get general_motion_p
    std::array<double, 6> temp_p{0.0};
    general_motion.getP(temp_p.data());
    par.general_motion_p[i].assign(temp_p.begin(), temp_p.end());
    // get general_motion_v
    std::array<double, 6> temp_v{0.0};
    general_motion.getV(temp_v.data());
    par.general_motion_v[i].assign(temp_v.begin(), temp_v.end());
    //aris::dynamic::dsp(1, 6, temp_v.data());
  }
  double joint0Pos[6]{0.0};
  model()->jointPool().at(0).makI()->getPe(joint0Pos, "123");
  double joint0Vel[6]{0.0};
  model()->jointPool().at(0).makI()->getVa(joint0Vel);
  // std::cout << joint1Pos << std::endl;
  // std::cout << joint1Vel << std::endl;

  for (std::size_t i = 0; i < model()->motionPool().size(); ++i) {
    auto& motion = model()->motionPool().at(i);
    // get motors_p
    par.motors_p[i] = motion.mp();
    // get motors_v
    par.motors_v[i] = motion.mv();
    // get motors_a
    par.motors_a[i] = motion.ma();
  }
  auto& inter =
      dynamic_cast<server::ProgramWebInterface&>(cs.interfacePool().at(0));
  std::vector<std::pair<std::string, std::any>> out_param;
  // push back general_motion_p
  out_param.push_back(std::make_pair<std::string, std::any>(
      "general_motion_p", nlohmann::json(par.general_motion_p)));
  // push back general_motion_v
  out_param.push_back(std::make_pair<std::string, std::any>(
      "general_motion_v", nlohmann::json(par.general_motion_v)));
  // push back joint 0
  out_param.push_back(std::make_pair<std::string, std::any>(
      "joint_p", nlohmann::json(joint0Pos)));
  out_param.push_back(std::make_pair<std::string, std::any>(
      "joint_v", nlohmann::json(joint0Vel)));
  // push back motors_p
  out_param.push_back(std::make_pair<std::string, std::any>(
      "motors_p", nlohmann::json(par.motors_p)));
  // push back motors_v
  out_param.push_back(std::make_pair<std::string, std::any>(
      "motors_v", nlohmann::json(par.motors_v)));
  // push back motors_a
  out_param.push_back(std::make_pair<std::string, std::any>(
      "motors_a", nlohmann::json(par.motors_a)));
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
      "	 <Param name=\"reset\" abbreviation=\"r\" default=\"0\"/>"
      "	 <Param name=\"delay\" abbreviation=\"d\" default=\"0\"/>"
      "  <Param name=\"position\" abbreviation=\"p\" default=\"{0,0}\"/>"
      "	</GroupParam>"
      "</Command>");
}
ARIS_DEFINE_BIG_FOUR_CPP(SimulationWithAction);

ARIS_REGISTRATION {
  aris::core::class_<SimulationWithAction>("SimulationWithAction")
      .inherit<aris::plan::Plan>();
}
}  // namespace sire::plan