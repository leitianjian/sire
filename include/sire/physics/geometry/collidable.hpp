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

namespace sire::physics{
namespace geometry {
/* unique geometry id for every added collision geometry */
using namespace std;
using namespace hpp;

class SIRE_API Collidable {
 public:
  auto getCollisionObject() -> fcl::CollisionObject*;
  auto resetCollisionObject(fcl::CollisionObject* object) -> void;
  auto virtual updateLocation(const double* pm) -> void = 0;
  auto virtual init() -> void = 0;
  explicit Collidable();
  virtual ~Collidable();
  Collidable(const Collidable& other) = delete;
  // 不明白为啥这里不能是default，明明下面的都没啥问题
  Collidable(Collidable&& other) = delete;
  Collidable& operator=(const Collidable& other) = delete;
  Collidable& operator=(Collidable&& other) = delete;

 private:
  struct Imp;
  unique_ptr<Imp> imp_;
};
}  // namespace geometry

using CollisionObjectsPair =
    std::pair<sire::core::geometry::GeometryId, sire::core::geometry::GeometryId>;

}  // namespace  sire::physics
#endif