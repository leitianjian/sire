#include "sire/core/module_base.hpp"
#include <aris/core/reflection.hpp>

namespace sire::core {
ARIS_REGISTRATION {
  aris::core::class_<SireModuleBase>("SireModuleBase")
      .inherit<aris::core::NamedObject>();
}
}  // namespace sire::modules