#ifndef SIRE_CAPSULE_SHAPE_HPP_
#define SIRE_CAPSULE_SHAPE_HPP_

#include <array>

#include "sire/core/geometry/shape_base.hpp"

namespace sire::geometry {
class CapsuleShape final : public ShapeBase {
 public:
  ARIS_DECLARE_BIG_FOUR(CapsuleShape)

  /** Constructs a box with the given `length` and `radius`, which
   specify the capsule's dimension.
   @throws std::exception if `length` or `radius` are not strictly
   positive. */
  CapsuleShape(double radius, double length);

  virtual ~CapsuleShape();

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