#ifndef SIRE_MODULE_BASE_HPP_
#define SIRE_MODULE_BASE_HPP_

#include <sire_lib_export.h>

#include <aris/core/object.hpp>

namespace sire::core {
using namespace std;
class SIRE_API SireModuleBase : public aris::core::NamedObject {
 public:
  auto virtual init() -> void{};
  SireModuleBase() = default;
  virtual ~SireModuleBase() = default;
};
}  // namespace sire::core
#endif