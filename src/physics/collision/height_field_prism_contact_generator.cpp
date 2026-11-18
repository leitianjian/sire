#include "sire/physics/collision/height_field_contact_generator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>

#include <coal/hfield.h>
#include <coal/internal/shape_shape_func.h>
#include <coal/narrowphase/narrowphase.h>
#include <coal/shape/convex.h>
#include <coal/shape/geometric_shapes_utility.h>

namespace sire::physics::collision {
namespace {
using coal::Scalar;
using coal::Vec3s;

template <class Shape>
auto queryPrisms(const coal::CollisionObject& terrain,
                 const coal::CollisionObject& shape_object,
                 const Shape& shape, std::size_t maximum,
                 HeightFieldPrismQueryStats& stats)
    -> std::vector<coal::Contact> {
  const auto& grid = static_cast<const coal::HeightField<coal::AABB>&>(
      *terrain.collisionGeometry());
  const auto& xs = grid.getXGrid();
  const auto& ys = grid.getYGrid();
  const auto& heights = grid.getHeights();
  const auto pose = terrain.getTransform().inverseTimes(shape_object.getTransform());
  coal::AABB bounds;
  coal::computeBV<coal::AABB>(shape, pose, bounds);
  if (bounds.max_.x() < xs[0] || bounds.min_.x() > xs[xs.size() - 1] ||
      bounds.max_.y() < ys[ys.size() - 1] || bounds.min_.y() > ys[0] ||
      bounds.max_.z() < grid.getMinHeight() ||
      bounds.min_.z() > grid.getMaxHeight())
    return {};

  const auto index = [](Scalar coordinate, Scalar origin, Scalar spacing,
                        int last) {
    // Clamp in floating point before conversion, also for far-away shapes.
    return static_cast<int>(std::clamp(
        std::floor((coordinate - origin) / spacing), Scalar(0), Scalar(last)));
  };
  const Scalar dx = xs[1] - xs[0], dy = ys[0] - ys[1];
  // Include cells on both sides when a bound lies on a grid boundary.
  constexpr Scalar tolerance = Scalar(1e-9);
  const int c0 = index(bounds.min_.x() - tolerance, xs[0], dx, xs.size() - 2);
  const int c1 = index(bounds.max_.x() + tolerance, xs[0], dx, xs.size() - 2);
  const int r0 = index(ys[0] - bounds.max_.y() - tolerance, 0, dy, ys.size() - 2);
  const int r1 = index(ys[0] - bounds.min_.y() + tolerance, 0, dy, ys.size() - 2);

  // Allocate topology and GJK/EPA workspace once per geometry-pair query.
  // All prisms have identical topology; only their six vertex positions vary.
  auto vertices = std::make_shared<std::vector<Vec3s>>(6);
  (*vertices)[0] = Vec3s(0, 0, -1);
  (*vertices)[1] = Vec3s(0, 1, -1);
  (*vertices)[2] = Vec3s(1, 0, -1);
  (*vertices)[3] = Vec3s(0, 0, 0);
  (*vertices)[4] = Vec3s(0, 1, 0);
  (*vertices)[5] = Vec3s(1, 0, 0);
  auto faces = std::make_shared<std::vector<coal::Triangle32>>(8);
  (*faces)[0].set(0, 2, 1);
  (*faces)[1].set(3, 4, 5);
  (*faces)[2].set(0, 1, 3);
  (*faces)[3].set(3, 1, 4);
  (*faces)[4].set(1, 2, 5);
  (*faces)[5].set(1, 5, 4);
  (*faces)[6].set(0, 5, 2);
  (*faces)[7].set(5, 0, 3);
  coal::ConvexTpl<coal::Triangle32> prism(vertices, 6, faces, 8);
  coal::CollisionRequest request;
  request.gjk_tolerance = Scalar(2e-8);
  request.epa_tolerance = Scalar(2e-8);
  coal::GJKSolver solver(request);
  const coal::Transform3s identity;
  std::vector<coal::Contact> contacts;
  contacts.reserve(std::min(maximum, std::size_t(64)));
  for (int r = r0; r <= r1; ++r) {
    for (int c = c0; c <= c1; ++c) {
      ++stats.cells_visited;
      const Vec3s p00(xs[c], ys[r], heights(r, c));
      const Vec3s p01(xs[c], ys[r + 1], heights(r + 1, c));
      const Vec3s p10(xs[c + 1], ys[r], heights(r, c + 1));
      const Vec3s p11(xs[c + 1], ys[r + 1], heights(r + 1, c + 1));
      const std::array<std::array<Vec3s, 3>, 2> triangles{{
          {{p00, p01, p10}}, {{p01, p11, p10}}}};
      for (int triangle = 0; triangle < 2; ++triangle) {
        const auto& top = triangles[triangle];
        const Scalar high = std::max({top[0].z(), top[1].z(), top[2].z()});
        if (high < bounds.min_.z() || high <= grid.getMinHeight()) {
          ++stats.prisms_height_rejected;
          continue;
        }
        for (int i = 0; i < 3; ++i) {
          (*vertices)[i] = Vec3s(top[i].x(), top[i].y(), grid.getMinHeight());
          (*vertices)[i + 3] = top[i];
        }
        // ConvexBase::center is cached at construction; update it as well as
        // the AABB when reusing the six-vertex buffer for another prism.
        prism.center = Vec3s::Zero();
        for (const auto& vertex : *vertices) prism.center += vertex;
        prism.center /= Scalar(6);
        prism.computeLocalAABB();
        Vec3s p1, p2, normal;
        ++stats.prisms_tested;
        const Scalar gap = coal::internal::ShapeShapeDistance<
            coal::ConvexTpl<coal::Triangle32>, Shape>(
            &prism, identity, &shape, pose, &solver, true, p1, p2, normal);
        if (!std::isfinite(gap) || gap >= -tolerance || !p1.allFinite() ||
            !p2.allFinite() || !normal.allFinite() ||
            normal.squaredNorm() < Scalar(0.5))
          continue;
        const auto& world = terrain.getTransform();
        // b1 identifies a prism in row-major grid order, not a Coal BVH node.
        // Persistent manifold IDs deliberately do not depend on this index.
        const int prism_id = 2 * (r * (xs.size() - 1) + c) + triangle;
        contacts.emplace_back(
            terrain.collisionGeometry().get(), shape_object.collisionGeometry().get(),
            prism_id, coal::Contact::NONE, world.transform(p1), world.transform(p2),
            world.rotation() * normal, gap);
        if (contacts.size() >= maximum) {
          stats.limit_reached = true;
          return contacts;
        }
      }
    }
  }
  return contacts;
}
}  // namespace

auto generateHeightFieldPrismContacts(
    const coal::CollisionObject& terrain, const coal::CollisionObject& shape,
    std::size_t maximum, HeightFieldPrismQueryStats* output_stats)
    -> std::vector<coal::Contact> {
  HeightFieldPrismQueryStats stats;
  const auto run = [&]() -> std::vector<coal::Contact> {
    if (maximum == 0 || terrain.collisionGeometry()->getNodeType() != coal::HF_AABB)
      return {};
    const auto& geometry = *shape.collisionGeometry();
    switch (geometry.getNodeType()) {
#define SIRE_QUERY_PRISMS(node, type) \
      case coal::node: \
        return queryPrisms(terrain, shape, static_cast<const coal::type&>(geometry), \
                           maximum, stats)
      SIRE_QUERY_PRISMS(GEOM_SPHERE, Sphere);
      SIRE_QUERY_PRISMS(GEOM_BOX, Box);
      SIRE_QUERY_PRISMS(GEOM_CAPSULE, Capsule);
      SIRE_QUERY_PRISMS(GEOM_CYLINDER, Cylinder);
      SIRE_QUERY_PRISMS(GEOM_CONE, Cone);
      SIRE_QUERY_PRISMS(GEOM_ELLIPSOID, Ellipsoid);
      SIRE_QUERY_PRISMS(GEOM_CONVEX16, ConvexBase16);
      SIRE_QUERY_PRISMS(GEOM_CONVEX32, ConvexBase32);
#undef SIRE_QUERY_PRISMS
      default: return {};
    }
  };
  auto contacts = run();
  if (output_stats) *output_stats = stats;
  return contacts;
}
}  // namespace sire::physics::collision
