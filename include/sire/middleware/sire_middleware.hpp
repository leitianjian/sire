#ifndef SIRE_MIDDLEWARE_HPP_
#define SIRE_MIDDLEWARE_HPP_

#include <functional>
#include <iostream>
#include <string_view>

#include <sire_lib_export.h>

#include <aris/server/interface.hpp>
#include <aris/server/middle_ware.hpp>

#include "sire/core/module_base.hpp"
#include "sire/server/interface.hpp"

namespace sire::middleware {
using namespace std;
class SIRE_API SireMiddleware : public aris::server::MiddleWare {
 public:
  auto virtual init() -> void override;
  auto virtual executeCmd(std::string_view str,
                          std::function<void(std::string)> send_ret,
                          aris::server::Interface* interface) -> int override;
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

}  // namespace sire::middleware

#endif  // SIRE_MIDDLEWARE_HPP_