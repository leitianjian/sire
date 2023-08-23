#include "sire/middleware/sire_middleware.hpp"

#include <aris/core/core.hpp>
#include <aris/core/object.hpp>
#include <aris/server/control_server.hpp>

#include "sire/ext/json.hpp"
#include "sire/server/api.hpp"

namespace sire::middleware {
struct SireMiddleware::Imp {
  unique_ptr<simulator::Simulator> simulator_;
  unique_ptr<physics::PhysicsEngine> physics_engine_;
  unique_ptr<simulator::SimulatorModules> simulator_modules_;
};
SireMiddleware::SireMiddleware() : imp_(new Imp) {}
SireMiddleware::~SireMiddleware() = default;
SireMiddleware::SireMiddleware(SireMiddleware&& other) = default;
SireMiddleware& SireMiddleware::operator=(SireMiddleware&& other) = default;

auto SireMiddleware::init() -> void {
  imp_->physics_engine_->init();
  imp_->simulator_modules_->init(this);
  imp_->simulator_->init(this);
}
auto SireMiddleware::executeCmd(std::string_view str,
                                std::function<void(std::string)> send_ret,
                                aris::server::Interface* interface) -> int {
  (void)(interface);

  std::string cmd(str);

  auto cmd_id = MiddleWare::cmd_id_++;

  // std::cout << "INTERFACE -- " << interface->name() << std::endl;
  // std::cout << "       ID -- " << cmd_id << std::endl;
  // std::cout << "      CMD -- " << cmd << std::endl;
  // std::cout << std::endl;

  auto send_and_print = [cmd_id, cmd, send_ret](std::string ret) -> void {
    const std::string pre = "get";
    if (cmd.compare(0, pre.size(), pre) != 0) {
      std::cout << "    ID -- " << cmd_id << std::endl;
      std::cout << "   CMD -- " << cmd << std::endl;
      std::cout << "RETURN -- " << ret << std::endl;
      std::cout << std::endl;
    }
    send_ret(ret);
  };

  aris::server::ControlServer::instance().executeCmdInCmdLine(
      cmd, [send_and_print](aris::plan::Plan& plan) -> void {
        // LOG_INFO << "cmd " << plan.cmdId() << " return code   :" <<
        // plan.retCode() << "\n" << std::setw(aris::core::LOG_SPACE_WIDTH) <<
        // '|' << "return message:" << plan.retMsg() << std::endl;

        // only copy if it is a str
        if (auto js =
                std::any_cast<std::vector<std::pair<std::string, std::any>>>(
                    &plan.ret())) {
          js->push_back(std::make_pair<std::string, std::any>(
              "return_code", plan.executeRetCode()));
          js->push_back(std::make_pair<std::string, std::any>(
              "return_message", std::string(plan.executeRetMsg())));
          send_and_print(sire::server::parse_ret_value(*js, false));
        } else {
          std::vector<std::pair<std::string, std::any>> ret_js;
          ret_js.push_back(std::make_pair<std::string, std::any>(
              "return_code", plan.executeRetCode()));
          ret_js.push_back(std::make_pair<std::string, std::any>(
              "return_message", std::string(plan.executeRetMsg())));
          send_and_print(sire::server::parse_ret_value(ret_js, false));
        }
      });

  return 0;
}
auto SireMiddleware::resetSimulator(simulator::Simulator* simulator) -> void {
  imp_->simulator_.reset(simulator);
}
auto SireMiddleware::simulator() const -> const simulator::Simulator& {
  return *imp_->simulator_;
}
auto SireMiddleware::resetPhysicsEngine(physics::PhysicsEngine* engine)
    -> void {
  imp_->physics_engine_.reset(engine);
}
auto SireMiddleware::physicsEngine() const -> const physics::PhysicsEngine& {
  return *imp_->physics_engine_;
}
auto SireMiddleware::resetSimulatorModules(simulator::SimulatorModules* pool)
    -> void {
  imp_->simulator_modules_.reset(pool);
}
auto SireMiddleware::simulatorModules() const
    -> const simulator::SimulatorModules& {
  return *imp_->simulator_modules_;
}

ARIS_REGISTRATION {
  typedef simulator::Simulator& (SireMiddleware::*SimulatorFunc)();
  typedef physics::PhysicsEngine& (SireMiddleware::*PhysicsEngineFunc)();
  typedef simulator::SimulatorModules& (
      SireMiddleware::*SimulatorMudulesFunc)();
  aris::core::class_<SireMiddleware>("SireMiddleware")
      .inherit<aris::server::MiddleWare>()
      .prop("simulator", &SireMiddleware::resetSimulator,
            SimulatorFunc(&SireMiddleware::simulator))
      .prop("physics_engine", &SireMiddleware::resetPhysicsEngine,
            PhysicsEngineFunc(&SireMiddleware::physicsEngine))
      .prop("simulator_modules", &SireMiddleware::resetSimulatorModules,
            SimulatorMudulesFunc(&SireMiddleware::simulatorModules));
}
}  // namespace sire::middleware