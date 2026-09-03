#include "sire/core/geometry/shape_calculator.hpp"

#include <typeinfo>

#include <aris/core/reflection.hpp>

#include "sire/core/geometry/box_shape.hpp"
#include "sire/core/geometry/capsule_shape.hpp"
#include "sire/core/geometry/cylinder_shape.hpp"
#include "sire/core/geometry/sphere_shape.hpp"
#include "sire/core/geometry/height_field_shape.hpp"
#include "sire/core/geometry/mesh_shape.hpp"
#include "sire/core/nice_type_name.hpp"
#include "sire/core/sire_assert.hpp"
#include "sire/core/string_utils.hpp"

namespace sire::geometry {
ShapeCalculator::~ShapeCalculator() = default;

void ShapeCalculator::ImplementGeometry(const BoxShape&, void*) {
  ThrowUnsupportedGeometry("Box");
}

void ShapeCalculator::ImplementGeometry(const CapsuleShape&, void*) {
  ThrowUnsupportedGeometry("Capsule");
}

void ShapeCalculator::ImplementGeometry(const CylinderShape&, void*) {
  ThrowUnsupportedGeometry("Cylinder");
}

void ShapeCalculator::ImplementGeometry(const MeshShape&, void*) {
  ThrowUnsupportedGeometry("Mesh");
}

void ShapeCalculator::ImplementGeometry(const SphereShape&, void*) {
  ThrowUnsupportedGeometry("Sphere");
}

void ShapeCalculator::ImplementGeometry(const HeightFieldShape&, void*) {
  ThrowUnsupportedGeometry("HeightField");
}

void ShapeCalculator::ThrowUnsupportedGeometry(const std::string& shape_name) {
  throw std::runtime_error(
      sire::core::string_format("This class (%s) does not support %s.",
                                core::NiceTypeName::Get(*this), shape_name));
}

void ShapeToName::ImplementGeometry(const BoxShape& box, void*) {
  string_ = "box";
}

void ShapeToName::ImplementGeometry(const CapsuleShape& capsule, void*) {
  string_ = "capsule";
}

void ShapeToName::ImplementGeometry(const CylinderShape& cylinder, void*) {
  string_ = "cylinder";
}

void ShapeToName::ImplementGeometry(const MeshShape& mesh, void*) {
  string_ = "mesh";
}

void ShapeToName::ImplementGeometry(const SphereShape& sphere, void*) {
  string_ = "sphere";
}

void ShapeToName::ImplementGeometry(const HeightFieldShape& height_field, void*) {
  string_ = "hfield";
}

void ShapeToInertia::ImplementGeometry(const MeshShape& mesh, void* user_data) {
  ThrowUnsupportedGeometry("mesh inertia calc not support");
}

void ShapeToInertia::ImplementGeometry(const SphereShape& sphere, void* user_data) {
  double* iv = static_cast<double*>(user_data);
  double ixyz = 0.4 * iv[0] * sphere.radius() * sphere.radius();
  iv[4] = iv[5] = iv[6] = ixyz;
}

void ShapeToInertia::ImplementGeometry(const BoxShape& box, void* user_data) {
  double* iv = static_cast<double*>(user_data);
  double mass = iv[0], x{box.width()}, y{box.length()}, z{box.height()};
  iv[4] = mass * (y * y + z * z) / 12;  // ix
  iv[5] = mass * (x * x + z * z) / 12;  // iy
  iv[6] = mass * (x * x + y * y) / 12;  // iz
}

void ShapeToInertia::ImplementGeometry(const CapsuleShape& capsule, void* user_data) {
  double* iv = static_cast<double*>(user_data);
  double mass = iv[0], radius{capsule.radius()}, length{capsule.length()};

  // The capsule axis is local z.  `length` is the cylindrical section length;
  // the two hemispheres add 2 * radius to the total end-to-end length.
  // Assume uniform density and split the supplied total mass by volume.
  const double volume_length = length + 4.0 * radius / 3.0;
  const double cylinder_mass = mass * length / volume_length;
  const double hemispheres_mass = mass - cylinder_mass;
  const double hemisphere_center_offset = length / 2.0 + 3.0 * radius / 8.0;

  const double axial_inertia =
      0.5 * cylinder_mass * radius * radius +
      0.4 * hemispheres_mass * radius * radius;
  const double transverse_inertia =
      cylinder_mass * (3.0 * radius * radius + length * length) / 12.0 +
      hemispheres_mass *
          (83.0 * radius * radius / 320.0 +
           hemisphere_center_offset * hemisphere_center_offset);

  iv[4] = iv[5] = transverse_inertia;
  iv[6] = axial_inertia;
}

void ShapeToInertia::ImplementGeometry(const CylinderShape& cylinder, void* user_data) {
  double* iv = static_cast<double*>(user_data);
  double mass = iv[0], radius{cylinder.radius()}, length{cylinder.length()};

  // Coal cylinders use the local z axis as their symmetry axis.
  iv[4] = iv[5] =
      mass * (3.0 * radius * radius + length * length) / 12.0;
  iv[6] = 0.5 * mass * radius * radius;
}

void ShapeToInertia::ImplementGeometry(const HeightFieldShape&, void*) {
  ThrowUnsupportedGeometry("heightfield inertia calc not support");
}

}  // namespace sire::geometry
