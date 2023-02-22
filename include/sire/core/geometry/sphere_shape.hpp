#ifndef SIRE_SPHERE_SHAPE_HPP_
#define SIRE_SPHERE_SHAPE_HPP_

#include "sire/core/geometry/shape_base.hpp"

namespace sire::geometry {
class SphereShape : public ShapeBase {
 private:
  double radius_;

 public:
  auto setRadius(double radius_in) -> void;
  auto getRadius() const -> double;
  auto radius() const -> double;
  auto radius() -> double&;

  explicit SphereShape(double radius_in = 0.1);
  virtual ~SphereShape();
};
}  // namespace sire::geometry

#endif