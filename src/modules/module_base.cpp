#include "sire/modules/module_base.hpp"
#include <aris/core/reflection.hpp>

namespace sire::modules {
ARIS_REGISTRATION {
  aris::core::class_<SireModuleBase>("SireModuleBase")
      .inherit<aris::core::NamedObject>();
}
}  // namespace sire::modules