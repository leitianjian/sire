#ifndef SIRE_SHAPE_CALCULATOR_HPP_
#define SIRE_SHAPE_CALCULATOR_HPP_

#include <sire_lib_export.h>

#include <aris/core/object.hpp>

#include "sire/ext/json.hpp"
// #include "sire/core/geometry/box_shape.hpp"
// #include "sire/core/geometry/sphere_shape.hpp"
// #include "sire/core/geometry/mesh_shape.hpp"

namespace sire::geometry {
class BoxShape;
class MeshShape;
class SphereShape;
class CapsuleShape;
class CylinderShape;
class ShapeCalculator {
 public:
  virtual ~ShapeCalculator();

  virtual void ImplementGeometry(const BoxShape& box, void* user_data);
  virtual void ImplementGeometry(const CapsuleShape& capsule, void* user_data);
  // virtual void ImplementGeometry(const Capsule& capsule, void* user_data);
  // virtual void ImplementGeometry(const Convex& convex, void* user_data);
  virtual void ImplementGeometry(const CylinderShape& cylinder, void* user_data);
  // virtual void ImplementGeometry(const Ellipsoid& ellipsoid, void* user_data);
  // virtual void ImplementGeometry(const HalfSpace& half_space, void* user_data);
  virtual void ImplementGeometry(const MeshShape& mesh, void* user_data);
  // virtual void ImplementGeometry(const MeshcatCone& cone, void* user_data);
  virtual void ImplementGeometry(const SphereShape& sphere, void* user_data);

 protected:
  ARIS_DEFINE_BIG_FOUR(ShapeCalculator)
  ShapeCalculator() = default;

  /** Derived ShapeCalculators can replace the default message for unsupported
   geometries by overriding this method. The name of the unsupported shape type
   is given as the single parameter.  */
  virtual void ThrowUnsupportedGeometry(const std::string& shape_name);
};

/** Class that turns a Shape into a std::string representation. This reifier has
 a string() member that gets updated for each shape reified. The expected
 workflow would be:

 ```c++
 ShapeToString reifier;
 SceneGraphInspector inspector = ...;  // Get the inspector from somewhere.
 for (GeometryId id : inspector.GetAllGeometryIds()) {
   shape.Reify(id, reifier);
   std::cout << reifier.string() << "\n";
 }
 ```

 This will write out a string representation of every geometry registered to
 SceneGraph.  */
class ShapeToName final : public ShapeCalculator {
 public:
  /** @name  Implementation of ShapeReifier interface  */
  //@{
  using ShapeCalculator::ImplementGeometry;
  void ImplementGeometry(const BoxShape& box, void* user_data) final;
  void ImplementGeometry(const CapsuleShape& capsule, void* user_data) final;
  // void ImplementGeometry(const Capsule& capsule, void* user_data) final;
  // void ImplementGeometry(const Convex& convex, void* user_data) final;
  void ImplementGeometry(const CylinderShape& cylinder, void* user_data) final;
  // void ImplementGeometry(const Ellipsoid& ellipsoid, void* user_data) final;
  // void ImplementGeometry(const HalfSpace& half_space, void* user_data) final;
  void ImplementGeometry(const MeshShape& mesh, void* user_data) final;
  // void ImplementGeometry(const MeshcatCone& cone, void* user_data) final;
  void ImplementGeometry(const SphereShape& sphere, void* user_data) final;

  //@}
  const std::string& string() const { return string_; }

 private:
  std::string string_;
};
// Mass should be in the user_data first element.
class ShapeToInertia final : public ShapeCalculator {
 public:
  /** @name  Implementation of ShapeReifier interface  */
  //@{
  using ShapeCalculator::ImplementGeometry;
  void ImplementGeometry(const BoxShape& box, void* user_data) final;
  void ImplementGeometry(const CapsuleShape& capsule, void* user_data) final;
  // void ImplementGeometry(const Capsule& capsule, void* user_data) final;
  // void ImplementGeometry(const Convex& convex, void* user_data) final;
  void ImplementGeometry(const CylinderShape& cylinder, void* user_data) final;
  // void ImplementGeometry(const Ellipsoid& ellipsoid, void* user_data) final;
  // void ImplementGeometry(const HalfSpace& half_space, void* user_data) final;
  void ImplementGeometry(const MeshShape& mesh, void* user_data) final;
  // void ImplementGeometry(const MeshcatCone& cone, void* user_data) final;
  void ImplementGeometry(const SphereShape& sphere, void* user_data) final;
};
}  // namespace sire::geometry
#endif