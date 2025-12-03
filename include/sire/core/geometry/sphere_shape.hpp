#ifndef SIRE_SPHERE_SHAPE_HPP_
#define SIRE_SPHERE_SHAPE_HPP_

#include "sire/core/geometry/shape_base.hpp"

namespace sire::geometry {
class SphereShape final : public ShapeBase {
 private:
  double radius_{};

 public:
  auto setRadius(double radius_in) -> void;
  auto getRadius() const -> double;
  auto radius() const -> double;
  auto radius() -> double&;

  /** Constructs a sphere with the given `radius`.
   @throws std::exception if `radius` is negative. Note that a zero radius is
   considered valid. */
  explicit SphereShape(double radius_in = 0.1);
  virtual ~SphereShape();
};
}  // namespace sire::geometry

#endif