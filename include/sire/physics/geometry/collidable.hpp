#ifndef SIRE_COLLIDABLE_HPP_
#define SIRE_COLLIDABLE_HPP_

#include <atomic>
#include <string>
#include <utility>

#include <sire_lib_export.h>

#include <hpp/fcl/collision_object.h>

#include <aris/dynamic/model_basic.hpp>
#include <aris/dynamic/model_coordinate.hpp>

#include "sire/core/geometry/geometry_base.hpp"
#include "sire/core/sire_decl_def_macro.hpp"

namespace sire::physics {
namespace geometry {
/* unique geometry id for every added collision geometry */
using namespace std;
using namespace hpp;

class SIRE_API Collidable {
 public:
  auto getCollisionObject() -> fcl::CollisionObject*;
  auto resetCollisionObject(fcl::CollisionObject* object) -> void;
  auto addContactProp(const string& name, double value) -> void;
  auto rmContactProp(const string& name) -> bool;
  auto updContactProp(const string& name, double new_value) -> void;
  auto getContactProp(const string& name) -> double;
  auto getContactPropOrDefault(const string& name, double default_value)
      -> double;
  auto virtual updateLocation(const double* pm) -> void = 0;
  auto virtual init() -> void = 0;
  explicit Collidable();
  virtual ~Collidable();
  SIRE_DECLARE_MOVE_CTOR(Collidable);

 private:
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;
};
}  // namespace geometry

using CollisionObjectsPair =
    std::pair<sire::geometry::GeometryId, sire::geometry::GeometryId>;
}  // namespace sire::physics
#endif