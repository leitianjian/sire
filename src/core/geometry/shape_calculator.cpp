#include "sire/core/geometry/shape_calculator.hpp"

#include <typeinfo>

#include <aris/core/reflection.hpp>

#include "sire/core/geometry/box_shape.hpp"
#include "sire/core/geometry/sphere_shape.hpp"
#include "sire/core/nice_type_name.hpp"
#include "sire/core/sire_assert.hpp"
#include "sire/core/string_utils.hpp"

namespace sire::geometry {
ShapeCalculator::~ShapeCalculator() = default;

void ShapeCalculator::ImplementGeometry(const BoxShape&, void*) {
  ThrowUnsupportedGeometry("Box");
}

// void ShapeCalculator::ImplementGeometry(const Capsule&, void*) {
//   ThrowUnsupportedGeometry("Capsule");
// }

// void ShapeCalculator::ImplementGeometry(const Convex&, void*) {
//   ThrowUnsupportedGeometry("Convex");
// }

// void ShapeCalculator::ImplementGeometry(const Cylinder&, void*) {
//   ThrowUnsupportedGeometry("Cylinder");
// }

// void ShapeCalculator::ImplementGeometry(const Ellipsoid&, void*) {
//   ThrowUnsupportedGeometry("Ellipsoid");
// }

// void ShapeCalculator::ImplementGeometry(const HalfSpace&, void*) {
//   ThrowUnsupportedGeometry("HalfSpace");
// }

void ShapeCalculator::ImplementGeometry(const MeshShape&, void*) {
  ThrowUnsupportedGeometry("Mesh");
}

// void ShapeCalculator::ImplementGeometry(const MeshcatCone&, void*) {
//   ThrowUnsupportedGeometry("MeshcatCone");
// }

void ShapeCalculator::ImplementGeometry(const SphereShape&, void*) {
  ThrowUnsupportedGeometry("Sphere");
}

void ShapeCalculator::ThrowUnsupportedGeometry(const std::string& shape_name) {
  throw std::runtime_error(
      sire::core::string_format("This class (%s) does not support %s.",
                                core::NiceTypeName::Get(*this), shape_name));
}
void ShapeToName::ImplementGeometry(const BoxShape& box, void*) {
  string_ = "box";
}

// void ShapeToString::ImplementGeometry(const Capsule& capsule, void*) {
//   string_ = fmt::format("Capsule(r: {}, l: {})", capsule.radius(),
//                         capsule.length());
// }

// void ShapeToString::ImplementGeometry(const Convex& convex, void*) {
//   string_ =
//       fmt::format("Convex(s: {}, path: {})", convex.scale(),
//       convex.filename());
// }

// void ShapeToString::ImplementGeometry(const Cylinder& cylinder, void*) {
//   string_ = fmt::format("Cylinder(r: {}, l: {})", cylinder.radius(),
//                         cylinder.length());
// }

// void ShapeToString::ImplementGeometry(const Ellipsoid& ellipsoid, void*) {
//   string_ = fmt::format("Ellipsoid(a: {}, b: {}, c: {})", ellipsoid.a(),
//                         ellipsoid.b(), ellipsoid.c());
// }

// void ShapeToString::ImplementGeometry(const HalfSpace&, void*) {
//   string_ = "Halfspace";
// }

void ShapeToName::ImplementGeometry(const MeshShape& mesh, void*) {
  string_ = "mesh";
}

// void ShapeToString::ImplementGeometry(const MeshcatCone& cone, void*) {
//   string_ = fmt::format("MeshcatCone(height: {}, a: {}, b: {})",
//   cone.height(),
//                         cone.a(), cone.b());
// }

void ShapeToName::ImplementGeometry(const SphereShape& sphere, void*) {
  string_ = "sphere";
}

void ShapeToInertia::ImplementGeometry(const MeshShape& mesh, void* user_data) {
  ThrowUnsupportedGeometry("mesh inertia calc not support");
}

// void ShapeToString::ImplementGeometry(const MeshcatCone& cone, void*) {
//   string_ = fmt::format("MeshcatCone(height: {}, a: {}, b: {})",
//   cone.height(),
//                         cone.a(), cone.b());
// }

void ShapeToInertia::ImplementGeometry(const SphereShape& sphere,
                                       void* user_data) {
  double* iv = static_cast<double*>(user_data);
  double ixyz = 0.4 * iv[0] * sphere.radius() * sphere.radius();
  iv[4] = iv[5] = iv[6] = ixyz;
}
void ShapeToInertia::ImplementGeometry(const BoxShape& box,
                                       void* user_data) {
  double* iv = static_cast<double*>(user_data);
  double mass = iv[0], x{box.width()}, y{box.length()}, z{box.height()};
  iv[4] = mass * (y * y + z * z) / 12;  // ix
  iv[5] = mass * (x * x + z * z) / 12;  // iy
  iv[6] = mass * (x * x + y * y) / 12;  // iz
}
}  // namespace sire::geometry
