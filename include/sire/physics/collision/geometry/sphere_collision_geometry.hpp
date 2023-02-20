#ifndef SIRE_SPHERE_COLLISION_GEOMETRY_HPP_
#define SIRE_SPHERE_COLLISION_GEOMETRY_HPP_

#include <atomic>
#include <string>
#include <utility>

#include <sire_lib_export.h>

#include <hpp/fcl/collision_object.h>

#include <aris/dynamic/model_basic.hpp>
#include <aris/dynamic/model_coordinate.hpp>

#include "sire/physics/collision/geometry/collision_geometry.hpp"

namespace sire::collision {
namespace geometry {
/* unique geometry id for every added collision geometry */
using namespace std;
using namespace hpp;
using GeometryId = int64_t;
class SIRE_API SphereGeometry : public CollisionGeometry {
 public:
  auto radius() -> double;
  auto setRadius(double radius) -> void;
  auto init() -> void override;
  explicit SphereGeometry(double radius = 0, const double* prt_pm = nullptr);
  virtual ~SphereGeometry();
  SphereGeometry(const SphereGeometry& other) = delete;
  SphereGeometry(SphereGeometry&& other) = delete;
  SphereGeometry& operator=(const SphereGeometry& other) = delete;
  SphereGeometry& operator=(SphereGeometry&& other) = delete;

 private:
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;
};
}  // namespace geometry
}  // namespace sire::collision
#endif