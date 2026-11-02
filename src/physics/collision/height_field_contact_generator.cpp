#include "sire/physics/collision/height_field_contact_generator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <tuple>
#include <vector>

#include <coal/hfield.h>
#include <coal/internal/traversal_node_hfield_shape.h>
#include <coal/collision.h>
#include <coal/contact_patch.h>
#include <coal/distance.h>
#include <coal/shape/geometric_shapes.h>

namespace sire::physics::collision {
namespace {
using coal::Scalar;
using coal::Vec3s;

constexpr Scalar kGeometryTolerance = 1e-9;

struct Triangle {
  Vec3s a, b, c;
  Vec3s normal;
  bool top;
};

struct Candidate {
  coal::Contact contact;
  bool box_vertex;
};

auto makeTriangle(const Vec3s& a, const Vec3s& b, const Vec3s& c, bool top)
    -> Triangle {
  const Vec3s cross = (b - a).cross(c - a);
  return {a, b, c, cross.normalized(), top};
}

auto insideTriangle(const Vec3s& point, const Triangle& triangle) -> bool {
  const std::array<Vec3s, 3> vertices{triangle.a, triangle.b, triangle.c};
  for (int i = 0; i < 3; ++i) {
    if (triangle.normal.dot((vertices[(i + 1) % 3] - vertices[i])
                                .cross(point - vertices[i])) <
        -kGeometryTolerance)
      return false;
  }
  return true;
}

auto closestPointOnTriangle(const Vec3s& point, const Triangle& triangle)
    -> Vec3s {
  // Closest-point regions of a finite triangle (Ericson's barycentric test).
  const Vec3s ab = triangle.b - triangle.a;
  const Vec3s ac = triangle.c - triangle.a;
  const Vec3s ap = point - triangle.a;
  const Scalar d1 = ab.dot(ap), d2 = ac.dot(ap);
  if (d1 <= 0 && d2 <= 0) return triangle.a;

  const Vec3s bp = point - triangle.b;
  const Scalar d3 = ab.dot(bp), d4 = ac.dot(bp);
  if (d3 >= 0 && d4 <= d3) return triangle.b;
  const Scalar vc = d1 * d4 - d3 * d2;
  if (vc <= 0 && d1 >= 0 && d3 <= 0)
    return triangle.a + (d1 / (d1 - d3)) * ab;

  const Vec3s cp = point - triangle.c;
  const Scalar d5 = ab.dot(cp), d6 = ac.dot(cp);
  if (d6 >= 0 && d5 <= d6) return triangle.c;
  const Scalar vb = d5 * d2 - d1 * d6;
  if (vb <= 0 && d2 >= 0 && d6 <= 0)
    return triangle.a + (d2 / (d2 - d6)) * ac;

  const Scalar va = d3 * d6 - d5 * d4;
  if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0)
    return triangle.b + ((d4 - d3) / (d4 - d3 + d5 - d6)) *
                            (triangle.c - triangle.b);
  const Scalar sum = va + vb + vc;
  if (std::abs(sum) <= std::numeric_limits<Scalar>::epsilon() *
                           ab.squaredNorm() * ac.squaredNorm())
    return triangle.a;
  return triangle.a + (vb / sum) * ab + (vc / sum) * ac;
}

auto clipToTriangle(std::vector<Vec3s> polygon, const Triangle& triangle)
    -> std::vector<Vec3s> {
  const std::array<Vec3s, 3> vertices{triangle.a, triangle.b, triangle.c};
  for (int edge = 0; edge < 3 && !polygon.empty(); ++edge) {
    std::vector<Vec3s> clipped;
    const Vec3s& a = vertices[edge];
    const Vec3s& b = vertices[(edge + 1) % 3];
    Vec3s previous = polygon.back();
    Scalar previous_side = triangle.normal.dot((b - a).cross(previous - a));
    for (const Vec3s& current : polygon) {
      const Scalar current_side =
          triangle.normal.dot((b - a).cross(current - a));
      const bool previous_inside = previous_side >= -kGeometryTolerance;
      const bool current_inside = current_side >= -kGeometryTolerance;
      if (previous_inside != current_inside) {
        const Scalar denominator = previous_side - current_side;
        if (std::abs(denominator) > kGeometryTolerance)
          clipped.push_back(previous +
                            (previous_side / denominator) *
                                (current - previous));
      }
      if (current_inside) clipped.push_back(current);
      previous = current;
      previous_side = current_side;
    }
    polygon = std::move(clipped);
  }
  return polygon;
}

auto orderedFacePolygon(const std::array<Vec3s, 8>& vertices,
                        const coal::Matrix3s& box_rotation,
                        const Triangle& triangle, Vec3s& face_normal,
                        Scalar& face_offset)
    -> std::vector<Vec3s> {
  const Vec3s& normal = triangle.normal;
  int face_axis = 0;
  for (int axis = 1; axis < 3; ++axis)
    if (std::abs(normal.dot(box_rotation.col(axis))) >
        std::abs(normal.dot(box_rotation.col(face_axis))))
      face_axis = axis;
  const Scalar orientation = normal.dot(box_rotation.col(face_axis));
  const int face_bit = orientation > 0 ? 0 : 1;
  face_normal = (face_bit ? 1 : -1) * box_rotation.col(face_axis);
  std::vector<Vec3s> polygon;
  for (int i = 0; i < 8; ++i) {
    if (((i >> face_axis) & 1) == face_bit) {
      const Vec3s& vertex = vertices[i];
      polygon.push_back(vertex - normal.dot(vertex - triangle.a) * normal);
      face_offset = face_normal.dot(vertex);
    }
  }
  if (polygon.size() > 2) {
    Vec3s centroid = Vec3s::Zero();
    for (const Vec3s& point : polygon) centroid += point;
    centroid /= static_cast<Scalar>(polygon.size());
    const Vec3s reference = std::abs(normal.z()) < Scalar(0.9)
                                ? Vec3s::UnitZ()
                                : Vec3s::UnitX();
    const Vec3s tangent = normal.cross(reference).normalized();
    const Vec3s bitangent = normal.cross(tangent);
    std::sort(polygon.begin(), polygon.end(), [&](const Vec3s& a,
                                                  const Vec3s& b) {
      const Vec3s da = a - centroid, db = b - centroid;
      return std::atan2(da.dot(bitangent), da.dot(tangent)) <
             std::atan2(db.dot(bitangent), db.dot(tangent));
    });
  }
  return polygon;
}

auto makeContact(const coal::CollisionObject& terrain,
                 const coal::CollisionObject& shape,
                 const Vec3s& terrain_point_local,
                 const Vec3s& shape_point_local, const Vec3s& normal_local,
                 Scalar gap) -> coal::Contact {
  const auto& terrain_pose = terrain.getTransform();
  return coal::Contact(terrain.collisionGeometry().get(),
                       shape.collisionGeometry().get(), coal::Contact::NONE,
                       coal::Contact::NONE,
                       terrain_pose.transform(terrain_point_local),
                       terrain_pose.transform(shape_point_local),
                       terrain_pose.getRotation() * normal_local, gap);
}

auto addCandidate(std::vector<Candidate>& candidates, Candidate candidate)
    -> void {
  for (Candidate& old : candidates) {
    if ((old.contact.pos - candidate.contact.pos).squaredNorm() < 1e-16 &&
        old.contact.normal.dot(candidate.contact.normal) > Scalar(0.9999)) {
      if (candidate.contact.penetration_depth < old.contact.penetration_depth)
        old.contact = candidate.contact;
      old.box_vertex = old.box_vertex || candidate.box_vertex;
      return;
    }
  }
  candidates.push_back(std::move(candidate));
}

auto generateSphereCandidates(const coal::CollisionObject& terrain,
                              const coal::CollisionObject& shape,
                              const coal::Sphere& sphere,
                              const Vec3s& center, const Triangle& triangle,
                              std::vector<Candidate>& candidates,
                              std::vector<Candidate>& edge_candidates) -> void {
  const Scalar center_plane = triangle.normal.dot(center - triangle.a);
  if (!triangle.top && center_plane < -kGeometryTolerance) return;
  const Vec3s projected = center - center_plane * triangle.normal;
  Vec3s terrain_point, normal;
  Scalar gap;
  if (triangle.top) {
    // An internal cell diagonal is not an exposed edge.  Only the triangle
    // containing the centre's projection normally creates a top contact.
    // Keep edge witnesses only as a fallback for steep terrain where no top
    // triangle contains the orthogonal projection.
    if (!insideTriangle(projected, triangle)) {
      if (center_plane < 0) return;
      terrain_point = closestPointOnTriangle(center, triangle);
      const Vec3s offset = center - terrain_point;
      const Scalar distance = offset.norm();
      if (distance <= kGeometryTolerance ||
          distance - sphere.radius >= -kGeometryTolerance)
        return;
      normal = offset / distance;
      gap = distance - sphere.radius;
      addCandidate(edge_candidates,
                   {makeContact(terrain, shape, terrain_point,
                                center - sphere.radius * normal, normal, gap),
                    false});
      return;
    }
    terrain_point = projected;
    normal = triangle.normal;
    gap = center_plane - sphere.radius;
  } else {
    terrain_point = closestPointOnTriangle(center, triangle);
    const Vec3s offset = center - terrain_point;
    const Scalar distance = offset.norm();
    normal = distance > kGeometryTolerance ? offset / distance
                                           : triangle.normal;
    gap = distance - sphere.radius;
  }
  if (gap >= -kGeometryTolerance) return;
  addCandidate(candidates,
               {makeContact(terrain, shape, terrain_point,
                            center - sphere.radius * normal, normal, gap),
                false});
}

auto generateBoxCandidates(const coal::CollisionObject& terrain,
                           const coal::CollisionObject& shape,
                           const std::array<Vec3s, 8>& vertices,
                           const coal::Matrix3s& box_rotation,
                           const Vec3s& center, const Triangle& triangle,
                           std::vector<Candidate>& candidates) -> void {
  if (!triangle.top &&
      triangle.normal.dot(center - triangle.a) < -kGeometryTolerance)
    return;
  Vec3s face_normal;
  Scalar face_offset = 0;
  std::vector<Vec3s> polygon = orderedFacePolygon(
      vertices, box_rotation, triangle, face_normal, face_offset);
  const Scalar denominator = face_normal.dot(triangle.normal);
  if (denominator >= -kGeometryTolerance) return;
  polygon = clipToTriangle(std::move(polygon), triangle);
  for (const Vec3s& terrain_point : polygon) {
    const Scalar gap =
        (face_offset - face_normal.dot(terrain_point)) / denominator;
    if (gap >= -kGeometryTolerance) continue;
    const Vec3s shape_point = terrain_point + gap * triangle.normal;
    bool is_vertex = false;
    for (const Vec3s& vertex : vertices) {
      if ((vertex - shape_point).squaredNorm() < Scalar(1e-16)) {
        is_vertex = true;
        break;
      }
    }
    addCandidate(candidates,
                 {makeContact(terrain, shape, terrain_point, shape_point,
                              triangle.normal, gap),
                  is_vertex});
  }
}

auto reduceCandidates(const std::vector<Candidate>& raw, std::size_t maximum)
    -> std::vector<coal::Contact> {
  if (raw.empty() || maximum == 0) return {};
  std::vector<std::size_t> vertex_count(raw.size(), 0);
  for (std::size_t i = 0; i < raw.size(); ++i) {
    for (const Candidate& candidate : raw) {
      if (!candidate.box_vertex) continue;
      if (raw[i].contact.normal.dot(candidate.contact.normal) >
              Scalar(0.9999) &&
          std::abs(raw[i].contact.normal.dot(
              raw[i].contact.nearest_points[0] -
              candidate.contact.nearest_points[0])) < Scalar(1e-5))
        ++vertex_count[i];
    }
  }
  std::vector<std::size_t> eligible;
  for (std::size_t i = 0; i < raw.size(); ++i) {
    if (raw[i].box_vertex || vertex_count[i] < 3)
      eligible.push_back(i);
  }
  if (eligible.empty()) return {};
  std::vector<std::size_t> chosen;
  auto position = [&](std::size_t index) -> const Vec3s& {
    return raw[index].contact.pos;
  };
  auto lexicographic = [&](std::size_t a, std::size_t b) {
    return std::make_tuple(position(a).x(), position(a).y(), position(a).z()) <
           std::make_tuple(position(b).x(), position(b).y(), position(b).z());
  };
  std::size_t first = eligible.front();
  for (std::size_t i : eligible) {
    const Scalar depth = raw[i].contact.penetration_depth;
    const Scalar best = raw[first].contact.penetration_depth;
    if (depth < best - Scalar(1e-12) ||
        (std::abs(depth - best) <= Scalar(1e-12) && lexicographic(i, first)))
      first = i;
  }
  chosen.push_back(first);
  const Vec3s patch_normal = raw[first].contact.normal;
  const Vec3s tangent = patch_normal.unitOrthogonal();
  const Vec3s bitangent = patch_normal.cross(tangent);
  const auto hull_area = [&](const std::vector<std::size_t>& indices) {
    std::vector<coal::Vec2s> points;
    points.reserve(indices.size());
    for (std::size_t index : indices)
      points.emplace_back(position(index).dot(tangent),
                          position(index).dot(bitangent));
    std::sort(points.begin(), points.end(), [](const auto& a, const auto& b) {
      return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
    });
    std::vector<coal::Vec2s> hull;
    const auto cross = [](const auto& a, const auto& b, const auto& c) {
      const auto ab = b - a, ac = c - a;
      return ab.x() * ac.y() - ab.y() * ac.x();
    };
    for (const auto& p : points) {
      while (hull.size() >= 2 &&
             cross(hull[hull.size() - 2], hull.back(), p) <= 0)
        hull.pop_back();
      hull.push_back(p);
    }
    const std::size_t lower_size = hull.size();
    for (auto it = points.rbegin(); it != points.rend(); ++it) {
      while (hull.size() > lower_size &&
             cross(hull[hull.size() - 2], hull.back(), *it) <= 0)
        hull.pop_back();
      hull.push_back(*it);
    }
    if (hull.size() < 4) return Scalar(0);
    hull.pop_back();
    Scalar area = 0;
    for (std::size_t i = 0; i < hull.size(); ++i)
      area += cross(coal::Vec2s::Zero(), hull[i], hull[(i + 1) % hull.size()]);
    return std::abs(area) * Scalar(0.5);
  };
  while (chosen.size() < maximum && chosen.size() < eligible.size()) {
    std::size_t best = raw.size();
    Scalar best_score = -1;
    for (std::size_t i : eligible) {
      if (std::find(chosen.begin(), chosen.end(), i) != chosen.end()) continue;
      Scalar score = (std::numeric_limits<Scalar>::max)();
      for (std::size_t j : chosen)
        score = std::min(score, (position(i) - position(j)).squaredNorm());
      if (chosen.size() == 2) {
        score = ((position(chosen[1]) - position(chosen[0]))
                     .cross(position(i) - position(chosen[0])))
                    .squaredNorm();
      } else if (chosen.size() >= 3) {
        auto with_candidate = chosen;
        with_candidate.push_back(i);
        score = hull_area(with_candidate);
      }
      if (score > best_score + Scalar(1e-18) ||
          (std::abs(score - best_score) <= Scalar(1e-18) &&
           (best == raw.size() || lexicographic(i, best)))) {
        best = i;
        best_score = score;
      }
    }
    if (best == raw.size() || best_score < Scalar(1e-16) ||
        (chosen.size() >= 3 &&
         best_score <= hull_area(chosen) + Scalar(1e-12)))
      break;
    chosen.push_back(best);
  }
  std::vector<coal::Contact> contacts;
  contacts.reserve(chosen.size());
  for (std::size_t index : chosen) contacts.push_back(raw[index].contact);
  return contacts;
}

// hpp-fcl returns one narrow-phase contact per HeightField leaf. Rebuild its
// two convex triangular prisms in Sire, then query the standard shape-shape
// patch API. The hpp-fcl source and binary remain unmodified.
auto leafPatchPoints(const coal::HeightField<coal::AABB>& grid,
                     const coal::CollisionObject& terrain,
                     const coal::CollisionObject& shape,
                     const coal::Contact& contact) -> std::vector<Vec3s> {
  if (contact.b1 < 0) return {};
  const auto& node = grid.getBV(contact.b1);
  if (!node.isLeaf()) return {};
  const int col = static_cast<int>(node.x_id);
  const int row = static_cast<int>(node.y_id);
  const auto& xs = grid.getXGrid();
  const auto& ys = grid.getYGrid();
  const auto& heights = grid.getHeights();
  const Vec3s p00(xs[col], ys[row], heights(row, col));
  const Vec3s p01(xs[col], ys[row + 1], heights(row + 1, col));
  const Vec3s p10(xs[col + 1], ys[row], heights(row, col + 1));
  const Vec3s p11(xs[col + 1], ys[row + 1], heights(row + 1, col + 1));
  const std::array<std::array<Vec3s, 3>, 2> triangles{
      std::array<Vec3s, 3>{p00, p01, p10},
      std::array<Vec3s, 3>{p01, p11, p10}};
  const Vec3s witness =
      terrain.getTransform().inverseTransform(contact.nearest_points[0]);
  const Vec3s normal =
      terrain.getTransform().rotation().transpose() * contact.normal;
  const auto on_top = [&](const std::array<Vec3s, 3>& tri) {
    const Vec3s cross = (tri[1] - tri[0]).cross(tri[2] - tri[0]);
    if (cross.dot(normal) < Scalar(0.9) * cross.norm()) return false;
    if (std::abs(cross.normalized().dot(witness - tri[0])) > Scalar(1e-7))
      return false;
    for (int edge = 0; edge < 3; ++edge)
      if (cross.dot((tri[(edge + 1) % 3] - tri[edge])
                         .cross(witness - tri[edge])) < Scalar(-1e-7))
        return false;
    return true;
  };
  if (!on_top(triangles[0]) && !on_top(triangles[1])) return {};

  coal::CollisionResult witness_result;
  witness_result.addContact(contact);
  const coal::ContactPatchRequest request(1, 16, Scalar(1e-3));
  coal::ConvexTpl<coal::Triangle32> convex1, convex2;
  int active1 = 0, active2 = 0;
  coal::details::buildConvexTriangles(node, grid, convex1, active1, convex2,
                                      active2);
  std::vector<Vec3s> points;
  for (const coal::CollisionGeometry* prism :
       {static_cast<const coal::CollisionGeometry*>(&convex1),
        static_cast<const coal::CollisionGeometry*>(&convex2)}) {
    coal::ContactPatchResult result(request);
    coal::computeContactPatch(prism, terrain.getTransform(),
                              shape.collisionGeometry().get(),
                              shape.getTransform(), witness_result, request,
                              result);
    if (result.numContactPatches() == 0) continue;
    const auto& patch = result.getContactPatch(0);
    for (std::size_t i = 0; i < patch.size(); ++i)
      points.push_back(patch.getPoint(i));
  }
  return points;
}

}  // namespace

auto generateHeightFieldPatchContacts(const coal::CollisionObject& terrain,
                                      const coal::CollisionObject& shape,
                                      std::size_t max_contacts)
    -> std::vector<coal::Contact> {
  if (max_contacts == 0 ||
      terrain.collisionGeometry()->getObjectType() != coal::OT_HFIELD)
    return {};

  coal::CollisionRequest collision_request;
  collision_request.enable_contact = true;
  collision_request.num_max_contacts = 512;
  collision_request.gjk_tolerance = 2e-8;
  coal::CollisionResult collision_result;
  coal::collide(&terrain, &shape, collision_request, collision_result);
  if (collision_result.numContacts() == 0) return {};

  const auto* grid = terrain.collisionGeometry()->getNodeType() == coal::HF_AABB
                         ? static_cast<const coal::HeightField<coal::AABB>*>(
                               terrain.collisionGeometry().get())
                         : nullptr;
  const auto surface_point = [&](const Vec3s& world_point)
      -> std::optional<Vec3s> {
    if (!grid) return std::nullopt;
    const Vec3s local = terrain.getTransform().inverseTransform(world_point);
    const auto& xs = grid->getXGrid();
    const auto& ys = grid->getYGrid();
    if (local.x() < xs[0] || local.x() > xs[xs.size() - 1] ||
        local.y() > ys[0] || local.y() < ys[ys.size() - 1])
      return std::nullopt;
    const Scalar dx = xs[1] - xs[0], dy = ys[0] - ys[1];
    const int col = std::clamp(
        static_cast<int>(std::floor((local.x() - xs[0]) / dx)), 0,
        static_cast<int>(xs.size()) - 2);
    const int row = std::clamp(
        static_cast<int>(std::floor((ys[0] - local.y()) / dy)), 0,
        static_cast<int>(ys.size()) - 2);
    const Scalar u = (local.x() - xs[col]) / dx;
    const Scalar v = (ys[row] - local.y()) / dy;
    const auto& h = grid->getHeights();
    const Scalar z = u + v <= 1
                         ? h(row, col) + u * (h(row, col + 1) - h(row, col)) +
                               v * (h(row + 1, col) - h(row, col))
                         : h(row + 1, col + 1) +
                               (1 - u) * (h(row + 1, col) - h(row + 1, col + 1)) +
                               (1 - v) * (h(row, col + 1) - h(row + 1, col + 1));
    return terrain.getTransform().transform(
        Vec3s(local.x(), local.y(), z));
  };

  const coal::Sphere point_geometry(Scalar(1e-8));
  const coal::DistanceRequest distance_request(true, true);
  const auto* box = shape.collisionGeometry()->getNodeType() == coal::GEOM_BOX
                        ? static_cast<const coal::Box*>(
                              shape.collisionGeometry().get())
                        : nullptr;
  const auto box_surface = [&](const Vec3s& terrain_point,
                               const Vec3s& normal) -> std::optional<Vec3s> {
    if (!box) return std::nullopt;
    const Vec3s origin =
        shape.getTransform().inverseTransform(terrain_point);
    const Vec3s direction =
        shape.getTransform().rotation().transpose() * (-normal);
    Scalar first = -(std::numeric_limits<Scalar>::max)();
    Scalar last = (std::numeric_limits<Scalar>::max)();
    bool inside = true;
    for (int axis = 0; axis < 3; ++axis) {
      const Scalar extent = box->halfSide[axis];
      inside &= std::abs(origin[axis]) <= extent + kGeometryTolerance;
      if (std::abs(direction[axis]) < kGeometryTolerance) {
        if (std::abs(origin[axis]) > extent + kGeometryTolerance)
          return std::nullopt;
        continue;
      }
      Scalar a = (-extent - origin[axis]) / direction[axis];
      Scalar b = (extent - origin[axis]) / direction[axis];
      if (a > b) std::swap(a, b);
      first = std::max(first, a);
      last = std::min(last, b);
    }
    if (first > last || last < kGeometryTolerance)
      return std::nullopt;
    const Scalar travel = inside ? last : first;
    if (travel < kGeometryTolerance) return std::nullopt;
    return shape.getTransform().transform(origin + travel * direction);
  };
  const auto valid_shape_witness = [&](const Vec3s& point) {
    if (!point.allFinite()) return false;
    if (box) {
      const Vec3s local = shape.getTransform().inverseTransform(point);
      Scalar distance_to_face = (std::numeric_limits<Scalar>::max)();
      for (int axis = 0; axis < 3; ++axis) {
        if (std::abs(local[axis]) > box->halfSide[axis] + Scalar(5e-5))
          return false;
        distance_to_face = std::min(
            distance_to_face,
            std::abs(std::abs(local[axis]) - box->halfSide[axis]));
      }
      return distance_to_face <= Scalar(5e-5);
    }
    coal::DistanceResult check;
    coal::distance(shape.collisionGeometry().get(), shape.getTransform(),
                   &point_geometry, coal::Transform3s(point),
                   distance_request, check);
    return std::isfinite(check.min_distance) &&
           std::abs(check.min_distance) <= Scalar(5e-5);
  };
  std::vector<Candidate> candidates;
  for (std::size_t i = 0; i < collision_result.numContacts(); ++i) {
    const coal::Contact& original = collision_result.getContact(i);
    if (original.penetration_depth >= -kGeometryTolerance ||
        !original.normal.allFinite())
      continue;
    const auto patch_points = grid ? leafPatchPoints(*grid, terrain, shape,
                                                     original)
                                   : std::vector<Vec3s>{};
    if (patch_points.size() <= 1) {
      if (valid_shape_witness(original.nearest_points[1]))
        addCandidate(candidates, {original, false});
      continue;
    }
    bool accepted_patch_point = false;
    Vec3s patch_centroid = Vec3s::Zero();
    for (const Vec3s& point : patch_points) patch_centroid += point;
    patch_centroid /= static_cast<Scalar>(patch_points.size());
    for (const Vec3s& point : patch_points) {
      const auto terrain_point = surface_point(point);
      if (!terrain_point) continue;
      if (box) {
        const auto shape_point = box_surface(*terrain_point, original.normal);
        if (!shape_point || !valid_shape_witness(*shape_point)) continue;
        const Vec3s delta = *shape_point - *terrain_point;
        const Scalar gap = delta.dot(original.normal);
        if (gap >= -kGeometryTolerance ||
            (delta - gap * original.normal).norm() > Scalar(1e-6))
          continue;
        addCandidate(candidates,
                     {coal::Contact(terrain.collisionGeometry().get(),
                                    shape.collisionGeometry().get(), original.b1,
                                    original.b2, *terrain_point, *shape_point,
                                    original.normal, gap),
                      false});
        accepted_patch_point = true;
        continue;
      }
      Vec3s inward = patch_centroid - point;
      inward -= inward.dot(original.normal) * original.normal;
      if (inward.squaredNorm() > Scalar(1e-16))
        inward = Scalar(1e-5) * inward.normalized();
      const Vec3s probe = point -
                          (std::abs(original.penetration_depth) + Scalar(1e-4)) *
                              original.normal + inward;
      const coal::Transform3s point_pose(probe);
      coal::DistanceResult shape_distance;
      coal::distance(shape.collisionGeometry().get(), shape.getTransform(),
                     &point_geometry, point_pose, distance_request,
                     shape_distance);
      const Vec3s shape_point = shape_distance.nearest_points[0] - inward;
      if (!shape_point.allFinite()) continue;
      if (!valid_shape_witness(shape_point)) continue;
      const Vec3s delta = shape_point - *terrain_point;
      const Scalar gap = delta.dot(original.normal);
      if (gap >= -kGeometryTolerance ||
          (delta - gap * original.normal).norm() > Scalar(1e-3))
        continue;
      addCandidate(candidates,
                   {coal::Contact(terrain.collisionGeometry().get(),
                                  shape.collisionGeometry().get(), original.b1,
                                  original.b2, *terrain_point, shape_point,
                                  original.normal, gap),
                    false});
      accepted_patch_point = true;
    }
    if (!accepted_patch_point &&
        valid_shape_witness(original.nearest_points[1]))
      addCandidate(candidates, {original, false});
  }
  if (shape.collisionGeometry()->getNodeType() == coal::GEOM_SPHERE) {
    for (auto& contact : generateHeightFieldContacts(terrain, shape,
                                                     max_contacts))
      addCandidate(candidates, {std::move(contact), false});
  }
  return reduceCandidates(candidates, max_contacts);
}

auto generateHeightFieldContacts(const coal::CollisionObject& terrain,
                                 const coal::CollisionObject& shape,
                                 std::size_t max_contacts)
    -> std::vector<coal::Contact> {
  if (max_contacts == 0 ||
      terrain.collisionGeometry()->getNodeType() != coal::HF_AABB)
    return {};
  const auto type = shape.collisionGeometry()->getNodeType();
  if (type != coal::GEOM_SPHERE && type != coal::GEOM_BOX) return {};
  const auto& height_field = static_cast<const coal::HeightField<coal::AABB>&>(
      *terrain.collisionGeometry());
  const auto& x = height_field.getXGrid();
  const auto& y = height_field.getYGrid();
  const auto& heights = height_field.getHeights();
  if (x.size() < 2 || y.size() < 2) return {};

  const coal::Transform3s shape_local =
      terrain.getTransform().inverseTimes(shape.getTransform());
  const Vec3s center = shape_local.getTranslation();
  Vec3s lower, upper;
  std::array<Vec3s, 8> box_vertices{};
  if (type == coal::GEOM_SPHERE) {
    const auto& sphere =
        static_cast<const coal::Sphere&>(*shape.collisionGeometry());
    lower = center.array() - sphere.radius;
    upper = center.array() + sphere.radius;
  } else {
    const auto& box = static_cast<const coal::Box&>(*shape.collisionGeometry());
    lower = Vec3s::Constant((std::numeric_limits<Scalar>::max)());
    upper = Vec3s::Constant((std::numeric_limits<Scalar>::lowest)());
    for (int i = 0; i < 8; ++i) {
      const Vec3s vertex((i & 1) ? box.halfSide.x() : -box.halfSide.x(),
                         (i & 2) ? box.halfSide.y() : -box.halfSide.y(),
                         (i & 4) ? box.halfSide.z() : -box.halfSide.z());
      box_vertices[i] = shape_local.transform(vertex);
      lower = lower.cwiseMin(box_vertices[i]);
      upper = upper.cwiseMax(box_vertices[i]);
    }
  }
  if (upper.x() < x[0] || lower.x() > x[x.size() - 1] ||
      upper.y() < y[y.size() - 1] || lower.y() > y[0] ||
      upper.z() < height_field.getMinHeight() ||
      lower.z() > height_field.getMaxHeight())
    return {};

  const Scalar dx = x[1] - x[0];
  const Scalar dy = y[0] - y[1];
  const auto cellIndex = [](Scalar value, Scalar origin, Scalar spacing,
                            int maximum) {
    return std::clamp(static_cast<int>(std::floor((value - origin) / spacing)),
                      0, maximum);
  };
  const int first_x = cellIndex(lower.x(), x[0], dx, x.size() - 2);
  const int last_x = cellIndex(upper.x(), x[0], dx, x.size() - 2);
  const int first_y = cellIndex(y[0] - upper.y(), 0, dy, y.size() - 2);
  const int last_y = cellIndex(y[0] - lower.y(), 0, dy, y.size() - 2);

  std::vector<Candidate> candidates;
  std::vector<Candidate> sphere_edge_candidates;
  const auto addFace = [&](const Triangle& triangle) {
    if (!triangle.normal.allFinite()) return;
    if (type == coal::GEOM_SPHERE) {
      const auto& sphere =
          static_cast<const coal::Sphere&>(*shape.collisionGeometry());
      generateSphereCandidates(terrain, shape, sphere, center, triangle,
                               candidates, sphere_edge_candidates);
    } else {
      generateBoxCandidates(terrain, shape, box_vertices,
                            shape_local.getRotation(), center, triangle,
                            candidates);
    }
  };
  for (int row = first_y; row <= last_y; ++row) {
    for (int col = first_x; col <= last_x; ++col) {
      const Scalar x0 = x[col], x1 = x[col + 1];
      const Scalar y0 = y[row], y1 = y[row + 1];
      const Scalar floor = height_field.getMinHeight();
      const Vec3s p00(x0, y0, heights(row, col));
      const Vec3s p01(x0, y1, heights(row + 1, col));
      const Vec3s p10(x1, y0, heights(row, col + 1));
      const Vec3s p11(x1, y1, heights(row + 1, col + 1));
      const Vec3s b00(x0, y0, floor), b01(x0, y1, floor);
      const Vec3s b10(x1, y0, floor), b11(x1, y1, floor);
      if (center.z() >= floor - kGeometryTolerance) {
        addFace(makeTriangle(p00, p01, p10, true));
        addFace(makeTriangle(p01, p11, p10, true));
      }
      if (row == 0) {
        addFace(makeTriangle(b00, p00, b10, false));
        addFace(makeTriangle(p00, p10, b10, false));
      }
      if (row == y.size() - 2) {
        addFace(makeTriangle(b01, b11, p01, false));
        addFace(makeTriangle(p01, b11, p11, false));
      }
      if (col == 0) {
        addFace(makeTriangle(b00, b01, p00, false));
        addFace(makeTriangle(p00, b01, p01, false));
      }
      if (col == x.size() - 2) {
        addFace(makeTriangle(b10, p10, b11, false));
        addFace(makeTriangle(p10, p11, b11, false));
      }
      if (center.z() < floor + kGeometryTolerance) {
        addFace(makeTriangle(b00, b10, b01, false));
        addFace(makeTriangle(b01, b10, b11, false));
      }
    }
  }
  if (candidates.empty() && !sphere_edge_candidates.empty()) {
    candidates = std::move(sphere_edge_candidates);
  } else if (type == coal::GEOM_SPHERE && !candidates.empty()) {
    // A steep ridge can support a sphere at an edge while the triangle below
    // its centre also produces a face contact. A coplanar internal grid edge
    // must not add another constraint.
    for (const Candidate& edge : sphere_edge_candidates) {
      bool independent_feature = true;
      for (const Candidate& face : candidates) {
        const Scalar offset = std::abs(face.contact.normal.dot(
            edge.contact.nearest_points[0] -
            face.contact.nearest_points[0]));
        if (offset < Scalar(5e-4) ||
            (edge.contact.pos - face.contact.pos).norm() < Scalar(0.01)) {
          independent_feature = false;
          break;
        }
      }
      if (independent_feature) addCandidate(candidates, edge);
    }
  }
  return reduceCandidates(candidates, max_contacts);
}

}  // namespace sire::physics::collision
