#ifndef SIRE_CYLINDER_SHAPE_HPP_
#define SIRE_CYLINDER_SHAPE_HPP_

#include <array>

#include "sire/core/geometry/shape_base.hpp"

namespace sire::geometry {
class CylinderShape final : public ShapeBase {
 public:
  ARIS_DECLARE_BIG_FOUR(CylinderShape)

  /** Constructs a box with the given `length` and `radius`, which
   specify the cylinder's dimension.
   @throws std::exception if `length` or `radius` are not strictly
   positive. */
  CylinderShape(double radius, double length);

  virtual ~CylinderShape();

  /** Returns the box's dimension along the y axis. */
  double radius() const { return radius_; }

  /** Returns the box's dimension along the x axis. */
  double length() const { return length_; }

  auto setRadius(double radius) -> void;
  auto setLength(double length) -> void;

 private:
  double radius_;
  double length_;
};
}  // namespace sire::geometry

#endif