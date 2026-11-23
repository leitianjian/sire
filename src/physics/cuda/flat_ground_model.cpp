#include "flat_ground_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <aris/dynamic/model.hpp>
#include <coal/shape/geometric_shapes.h>

#include "sire/core/material_manager.hpp"
#include "sire/physics/collision/collision_filter.hpp"
#include "sire/physics/contact/admm_contact_solver.hpp"
#include "sire/physics/geometry/collidable_geometry.hpp"
#include "sire/physics/physics_engine.hpp"

namespace sire::physics::contact::cuda {
namespace {

auto geometryObject(geometry::CollidableGeometry& shape) -> coal::CollisionObject& {
  auto* object = shape.getCollisionObject();
  if (!object) throw std::invalid_argument("Initialize Sire before loading the CUDA collision model");
  return *object;
}

auto materialCollider(geometry::CollidableGeometry& shape, const core::PropMap& material)
    -> GroundCollider {
  GroundCollider collider{};
  collider.geometry = shape.geometryId();
  collider.body = shape.partId();
  std::copy_n(*shape.pm(), 12, collider.local_pose);
  collider.stiffness = material.getPropValueOrDefault("k", 2e8);
  collider.damping = material.getPropValueOrDefault("d", 5e3);
  collider.friction = material.getPropValueOrDefault("cof", 0.3);

  const auto& object = *geometryObject(shape).collisionGeometry();
  std::size_t dimensions = 0;
  switch (object.getNodeType()) {
    case coal::GEOM_SPHERE:
      collider.shape = GroundShape::Sphere;
      collider.size[0] = static_cast<const coal::Sphere&>(object).radius;
      dimensions = 1;
      break;
    case coal::GEOM_BOX:
      collider.shape = GroundShape::Box;
      std::copy_n(static_cast<const coal::Box&>(object).halfSide.data(), 3, collider.size);
      dimensions = 3;
      break;
    case coal::GEOM_CYLINDER: {
      collider.shape = GroundShape::Cylinder;
      const auto& cylinder = static_cast<const coal::Cylinder&>(object);
      collider.size[0] = cylinder.radius;
      collider.size[1] = cylinder.halfLength;
      dimensions = 2;
      break;
    }
    default:
      throw std::invalid_argument("Unsupported Sire collision shape for CUDA flat ground");
  }
  if (static_cast<const coal::ShapeBase&>(object).getSweptSphereRadius() != 0 ||
      !std::all_of(collider.size, collider.size + dimensions,
                   [](double value) { return std::isfinite(value) && value > 0; }) ||
      !std::all_of(collider.local_pose, collider.local_pose + 12,
                   [](double value) { return std::isfinite(value); })) {
    throw std::invalid_argument("Invalid dimension, margin, or local pose in CUDA collision geometry");
  }
  return collider;
}

}  // namespace

auto loadFlatGroundModel(PhysicsEngine& engine) -> FlatGroundModel {
  auto* solver =
      dynamic_cast<admm_contact::AdmmContactSolver*>(&engine.contactSolver());
  if (!solver || !solver->singlePointContactMode()) {
    throw std::invalid_argument(
        "CUDA flat-ground collision requires an ADMM-family solver in single-point mode");
  }
  auto& anchors = engine.anchoredObjectsMap();
  if (anchors.size() != 1) {
    throw std::invalid_argument("CUDA flat-ground collision requires one anchored box");
  }
  auto& ground_shape = *anchors.begin()->second;
  const auto& ground = geometryObject(ground_shape);
  if (ground.getNodeType() != coal::GEOM_BOX || !ground.getRotation().isIdentity(0.0)) {
    throw std::invalid_argument("CUDA flat-ground collision requires a world-aligned anchored box");
  }
  const auto& box = static_cast<const coal::Box&>(*ground.collisionGeometry());
  if (box.getSweptSphereRadius() != 0 || !box.halfSide.allFinite() ||
      (box.halfSide.array() <= 0).any() || !ground.getTranslation().allFinite()) {
    throw std::invalid_argument("Invalid anchored box for CUDA flat-ground collision");
  }
  FlatGroundModel model{};
  model.body_count = engine.currentModel()->partPool().size();
  model.ground.geometry = ground_shape.geometryId();
  model.ground.body = ground_shape.partId();
  if (model.ground.body >= model.body_count) throw std::invalid_argument("Invalid anchored body index");
  std::copy_n(box.halfSide.data(), 3, model.ground.half_extent);
  std::copy_n(ground.getTranslation().data(), 3, model.ground.center);

  auto& filter = engine.collisionFilter();
  const auto& dynamic = engine.dynamicObjectsMap();
  for (const auto& [id, shape] : dynamic) {
    for (const auto& [other, unused] : dynamic) {
      if (id < other && filter.canCollideWith(id, other)) {
        throw std::invalid_argument("CUDA flat-ground collision requires dynamic pairs to be filtered");
      }
    }
    if (!filter.canCollideWith(id, model.ground.geometry)) continue;
    const auto& material = solver->materialManager().getPropMapOrDefault(
        {ground_shape.material(), shape->material()});
    auto collider = materialCollider(*shape, material);
    if (collider.geometry <= model.ground.geometry || collider.body >= model.body_count) {
      throw std::invalid_argument("CUDA flat-ground collision requires ground-first geometry IDs and valid bodies");
    }
    model.colliders.push_back(collider);
  }
  std::sort(model.colliders.begin(), model.colliders.end(),
            [](const auto& a, const auto& b) { return a.geometry < b.geometry; });
  return model;
}

}  // namespace sire::physics::contact::cuda
