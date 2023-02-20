#ifndef SIRE_SPHERE_SHAPE_HPP_
#define SIRE_SPHERE_SHAPE_HPP_

namespace sire::geometry {
class SphereShape {
 private:
  double radius_;

 public:
  auto virtual setRadius(double radius_in) -> void;
  auto virtual radius() -> double;

  explicit SphereShape(double radius_in);
  virtual ~SphereShape();
};
}  // namespace sire::geometry

#endif