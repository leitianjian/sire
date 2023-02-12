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
}  // namespace sire::server