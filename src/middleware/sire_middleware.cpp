#include "sire/middleware/sire_middleware.hpp"

#include <aris/core/core.hpp>
#include <aris/core/object.hpp>
#include <aris/server/control_server.hpp>

#include "sire/ext/json.hpp"
#include "sire/server/api.hpp"

namespace sire::middleware {
struct SireMiddleware::Imp {
  unique_ptr<aris::core::PointerArray<core::SireModuleBase>> modules_pool_;
};
auto SireMiddleware::init() -> void {
  for (auto& sire_module : *imp_->modules_pool_) {
    sire_module.init();
  }
}
auto SireMiddleware::executeCmd(std::string_view str,
                                std::function<void(std::string)> send_ret,
                                aris::server::Interface* interface) -> int {
  (void)(interface);

  std::string cmd(str);

  auto cmd_id = MiddleWare::cmd_id_++;

  std::cout << "INTERFACE -- " << interface->name() << std::endl;
  std::cout << "       ID -- " << cmd_id << std::endl;
  std::cout << "      CMD -- " << cmd << std::endl;
  std::cout << std::endl;

  auto send_and_print = [cmd_id, send_ret](std::string ret) -> void {
    std::cout << "    ID -- " << cmd_id << std::endl;
    std::cout << "RETURN -- " << ret << std::endl;
    std::cout << std::endl;

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
          send_and_print(aris::server::parse_ret_value(*js));
        } else {
          std::vector<std::pair<std::string, std::any>> ret_js;
          ret_js.push_back(std::make_pair<std::string, std::any>(
              "return_code", plan.executeRetCode()));
          ret_js.push_back(std::make_pair<std::string, std::any>(
              "return_message", std::string(plan.executeRetMsg())));
          send_and_print(aris::server::parse_ret_value(ret_js));
        }
      });

  return 0;
}
auto SireMiddleware::modulesPool()
    -> aris::core::PointerArray<core::SireModuleBase>& {
  return *imp_->modules_pool_;
}
auto SireMiddleware::resetModulesPool(
    aris::core::PointerArray<core::SireModuleBase>* pool) -> void {
  imp_->modules_pool_.reset(pool);
}
SireMiddleware::SireMiddleware() : imp_(new Imp) {}
SireMiddleware::SireMiddleware(SireMiddleware&& other) = default;
SireMiddleware& SireMiddleware::operator=(SireMiddleware&& other) = default;
SireMiddleware::~SireMiddleware() = default;

ARIS_REGISTRATION {
  aris::core::class_<aris::core::PointerArray<core::SireModuleBase>>(
      "SireModulesPoolObject")
      .asRefArray();
  typedef aris::core::PointerArray<core::SireModuleBase>& (
      SireMiddleware::*ModulesPoolFunc)();
  aris::core::class_<SireMiddleware>("SireMiddleware")
      .inherit<aris::server::MiddleWare>()
      .prop("modules_pool", &SireMiddleware::resetModulesPool,
            ModulesPoolFunc(&SireMiddleware::modulesPool));
}
}  // namespace sire::middleware