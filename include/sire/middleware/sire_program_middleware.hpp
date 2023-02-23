﻿#ifndef SIRE_MODULE_PROGRAM_MIDDLEWARE_HPP_
#define SIRE_MODULE_PROGRAM_MIDDLEWARE_HPP_

#include <functional>
#include <string_view>

#include <sire_lib_export.h>

#include <aris/server/interface.hpp>
#include <aris/server/middle_ware.hpp>

#include "sire/core/module_base.hpp"
#include "sire/server/interface.hpp"

namespace sire::middleware {
using namespace std;
// middleware可以在controlserver中调用init，所以有需要的可以放这里，而不是custom_module
class SIRE_API SireProgramMiddleware : public aris::server::MiddleWare {
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
  auto modulesPool() -> aris::core::PointerArray<core::SireModuleBase>&;
  auto resetModulesPool(aris::core::PointerArray<core::SireModuleBase>* pool)
      -> void;
  virtual ~SireProgramMiddleware();

  SireProgramMiddleware();
  SireProgramMiddleware(SireProgramMiddleware&& other);
  SireProgramMiddleware& operator=(SireProgramMiddleware&& other);

 private:
  struct Imp;
  std::unique_ptr<Imp> imp_;
};
}  // namespace sire::middleware

#endif  // SIRE_MODULE_PROGRAM_MIDDLEWARE_HPP_