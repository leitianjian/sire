#ifndef MODULE_BASE_H_
#define MODULE_BASE_H_

#include <sire_lib_export.h>

#include <aris/core/object.hpp>

namespace sire::core {
using namespace std;
class SIRE_API SireModuleBase : aris::core::NamedObject {
 public:
  auto virtual init() -> void{};
  SireModuleBase() = default;
  virtual ~SireModuleBase() = default;
};
}  // namespace sire::core
#endif