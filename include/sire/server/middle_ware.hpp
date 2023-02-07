#ifndef SIRE_SERVER_MIDDLE_WARE_HPP_
#define SIRE_SERVER_MIDDLE_WARE_HPP_

#include "sire/core/module_base.hpp"
#include "sire/server/interface.hpp"
#include <sire_lib_export.h>
#include <aris/server/interface.hpp>
#include <aris/server/middle_ware.hpp>
#include <functional>
#include <string_view>

namespace sire::server {
using namespace std;
// middleware可以在controlserver中调用init，所以有需要的可以放这里，而不是custom_module
class SIRE_API ProgramMiddleware : public aris::server::MiddleWare {
 public:
  auto isAutoMode() -> bool;
  auto isAutoRunning() -> bool;
  auto isAutoPaused() -> bool;
  auto isAutoStopped() -> bool;
  auto lastError() -> std::string;
  auto lastErrorCode() -> int;
  auto lastErrorLine() -> int;
  auto currentFileLine() -> std::tuple<std::string, int>;
  auto executeCmd(std::string_view str,
                  std::function<void(std::string)> send_ret,
                  aris::server::Interface* interface) -> int override;
  virtual ~ProgramMiddleware();

  ProgramMiddleware();
  ProgramMiddleware(ProgramMiddleware&& other);
  ProgramMiddleware& operator=(ProgramMiddleware&& other);

 private:
  struct Imp;
  std::unique_ptr<Imp> imp_;
};

class SIRE_API SireMiddleware : public ProgramMiddleware {
 public:
  auto virtual init() -> void override;
  auto modulesPool() -> aris::core::PointerArray<core::SireModuleBase>&;
  auto resetModulesPool(aris::core::PointerArray<core::SireModuleBase>* pool)
      -> void;

  virtual ~SireMiddleware();
  SireMiddleware();
  SireMiddleware(SireMiddleware&& other);
  SireMiddleware& operator=(SireMiddleware&& other);

 private:
  struct Imp;
  std::unique_ptr<Imp> imp_;
};

}  // namespace sire::server

#endif  // ARIS_SERVER_MIDDLE_WARE_HPP_