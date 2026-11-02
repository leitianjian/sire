#include "sire/physics/collision/height_field_contact_callback.hpp"

#include <cmath>
#include <limits>
#include <algorithm>
#include <numeric>
#include <optional>
#include <unordered_set>

#include <coal/broadphase/broadphase_dynamic_AABB_tree.h>
#include <coal/collision.h>
#include <coal/distance.h>
#include <coal/hfield.h>
#include <coal/math/transform.h>

#include <aris/core/reflection.hpp>
#include <aris/server/control_server.hpp>

#include "sire/core/sire_assert.hpp"
#include "sire/core/profiler.hpp"
#include "sire/physics/collision/height_field_contact_generator.hpp"
#include "sire/physics/geometry/collidable_geometry.hpp"

namespace sire::physics::collision {
using namespace coal;

auto HeightFieldContactManifold::KeyHash::operator()(const Key& key) const
    -> std::size_t {
  const auto a = reinterpret_cast<std::uintptr_t>(key.terrain);
  const auto b = reinterpret_cast<std::uintptr_t>(key.shape);
  return std::hash<std::uintptr_t>{}(a ^ (b + 0x9e3779b9 + (a << 6) +
                                            (a >> 2)));
}

void HeightFieldContactManifold::clear() { entries_.clear(); }

void HeightFieldContactManifold::beginQuery() { ++query_; }

void HeightFieldContactManifold::endQuery() {
  for (auto it = entries_.begin(); it != entries_.end();) {
    if (it->second.last_query != query_)
      it = entries_.erase(it);
    else
      ++it;
  }
}

auto HeightFieldContactManifold::select(
    const CollisionObject& terrain, const CollisionObject& shape,
    const std::vector<Contact>& input_candidates, std::size_t maximum)
    -> std::vector<HeightFieldTrackedContact> {
  if (maximum == 0) return {};
  auto& entry = entries_[Key{terrain.collisionGeometry().get(),
                             shape.collisionGeometry().get()}];
  entry.last_query = query_;
  const auto previous = std::move(entry.anchors);
  std::vector<Contact> candidates = input_candidates;
  const auto& height_field =
      static_cast<const HeightField<AABB>&>(*terrain.collisionGeometry());
  const auto& x_grid = height_field.getXGrid();
  const auto& y_grid = height_field.getYGrid();
  const auto& heights = height_field.getHeights();
  struct SurfaceSample {
    Vec3s point;
    Vec3s normal;
  };
  const auto sample_surface = [&](const Vec3s& point)
      -> std::optional<SurfaceSample> {
    int x_id = -1, y_id = -1;
    for (int i = 0; i + 1 < x_grid.size(); ++i)
      if (point.x() >= x_grid[i] - Scalar(1e-9) &&
          point.x() <= x_grid[i + 1] + Scalar(1e-9)) {
        x_id = i;
        break;
      }
    for (int i = 0; i + 1 < y_grid.size(); ++i)
      if (point.y() <= y_grid[i] + Scalar(1e-9) &&
          point.y() >= y_grid[i + 1] - Scalar(1e-9)) {
        y_id = i;
        break;
      }
    if (x_id < 0 || y_id < 0) return std::nullopt;
    const Scalar x0 = x_grid[x_id], x1 = x_grid[x_id + 1];
    const Scalar y0 = y_grid[y_id], y1 = y_grid[y_id + 1];
    const Scalar u = std::clamp((point.x() - x0) / (x1 - x0),
                                Scalar(0), Scalar(1));
    const Scalar v = std::clamp((y0 - point.y()) / (y0 - y1),
                                Scalar(0), Scalar(1));
    const Scalar h00 = heights(y_id, x_id);
    const Scalar h01 = heights(y_id + 1, x_id);
    const Scalar h10 = heights(y_id, x_id + 1);
    const Scalar h11 = heights(y_id + 1, x_id + 1);
    Scalar height, dh_du, dh_dv;
    if (u + v <= Scalar(1)) {
      height = h00 + u * (h10 - h00) + v * (h01 - h00);
      dh_du = h10 - h00;
      dh_dv = h01 - h00;
    } else {
      height = h11 + (Scalar(1) - u) * (h01 - h11) +
               (Scalar(1) - v) * (h10 - h11);
      dh_du = h11 - h01;
      dh_dv = h11 - h10;
    }
    const Scalar dh_dx = dh_du / (x1 - x0);
    const Scalar dh_dy = dh_dv / (y1 - y0);
    return SurfaceSample{
        Vec3s(point.x(), point.y(), height),
        Vec3s(-dh_dx, -dh_dy, Scalar(1)).normalized()};
  };
  const bool needs_refresh = candidates.empty() && previous.size() == 1;
  if (needs_refresh) {
    for (const auto& anchor : previous) {
    const Vec3s shape_world = shape.getTransform().transform(anchor.shape_local);
    const Vec3s shape_terrain = terrain.getTransform().inverseTransform(shape_world);
    Vec3s projected = shape_terrain;
    std::optional<SurfaceSample> surface;
    for (int iteration = 0; iteration < 3; ++iteration) {
      surface = sample_surface(projected);
      if (!surface) break;
      const Scalar distance =
          surface->normal.dot(shape_terrain - surface->point);
      projected = shape_terrain - distance * surface->normal;
    }
    if (!surface) continue;
    surface = sample_surface(projected);
    if (!surface) continue;
    const Vec3s normal_world =
        terrain.getTransform().rotation() * surface->normal;
    if (normal_world.dot(terrain.getTransform().rotation() *
                         anchor.normal_local) < Scalar(0.978147601))
      continue;
    const Vec3s terrain_world = terrain.getTransform().transform(surface->point);
    const Scalar depth = (shape_world - terrain_world).dot(normal_world);
    const Vec3s residual =
        shape_world - terrain_world - depth * normal_world;
    if (depth >= -Scalar(1e-9) || residual.norm() > Scalar(2e-4)) continue;
    bool duplicate = false;
    for (const auto& candidate : candidates) {
      if ((candidate.nearest_points[0] - terrain_world).norm() <
          Scalar(2e-4)) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) {
      candidates.emplace_back(
          terrain.collisionGeometry().get(), shape.collisionGeometry().get(),
          Contact::NONE, Contact::NONE, terrain_world, shape_world,
          normal_world, depth);
    }
    }
  }
  if (candidates.empty()) return {};
  const auto& aabb = shape.getAABB();
  const Scalar extent = (aabb.max_ - aabb.min_).norm();
  // Curved footprints can replace a rim support by roughly one HeightField
  // cell while preserving the same physical patch. Allow that motion within
  // the shape footprint; the normal test below still prevents cross-face IDs.
  const Scalar match_distance = std::max(Scalar(0.002), extent * Scalar(0.5));
  const Scalar match_depth = std::max(Scalar(1e-4), extent * Scalar(1e-3));
  const Scalar cluster_distance = std::max(Scalar(0.005), extent * Scalar(1.5));
  const Scalar cluster_plane_tolerance =
      std::max(Scalar(2e-4), extent * Scalar(0.01));
  constexpr Scalar kMatchNormal = Scalar(0.978147601);  // 12 degrees
  constexpr Scalar kMergeNormal = Scalar(0.996194698);  // 5 degrees
  constexpr Scalar kNewNormal = Scalar(0.906307787);    // 25 degrees
  constexpr Scalar kKeepNormal = Scalar(0.906307787);   // 25 degrees
  std::vector<int> matched(candidates.size(), -1);
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    const auto& c = candidates[i];
    const Vec3s terrain_local =
        terrain.getTransform().inverseTransform(c.nearest_points[0]);
    const Vec3s shape_local =
        shape.getTransform().inverseTransform(c.nearest_points[1]);
    const Vec3s normal_local =
        terrain.getTransform().rotation().transpose() * c.normal;
    Scalar best = (std::numeric_limits<Scalar>::max)();
    for (std::size_t j = 0; j < previous.size(); ++j) {
      if (normal_local.dot(previous[j].normal_local) < kMatchNormal)
        continue;
      if (std::abs(c.penetration_depth - previous[j].penetration_depth) >
          match_depth)
        continue;
      const Scalar distance =
          (terrain_local - previous[j].terrain_local).norm() +
          (shape_local - previous[j].shape_local).norm();
      if (distance < match_distance && distance < best) {
        best = distance;
        matched[i] = static_cast<int>(j);
      }
    }
  }

  std::vector<std::vector<std::size_t>> clusters;
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    bool grouped = false;
    for (auto& cluster : clusters) {
      for (std::size_t j : cluster) {
        const Scalar threshold =
            matched[i] >= 0 && matched[j] >= 0
                ? (previous[matched[i]].patch_id ==
                           previous[matched[j]].patch_id
                       ? kKeepNormal
                       : kMergeNormal)
                : kNewNormal;
        const Vec3s delta = candidates[i].pos - candidates[j].pos;
        const Scalar normal_dot =
            candidates[i].normal.dot(candidates[j].normal);
        const bool nearly_parallel = normal_dot >= kMergeNormal;
        const bool same_plane =
            !nearly_parallel ||
            (std::abs(candidates[i].normal.dot(delta)) <=
                 cluster_plane_tolerance &&
             std::abs(candidates[j].normal.dot(delta)) <=
                 cluster_plane_tolerance);
        if (normal_dot >= threshold && delta.norm() <= cluster_distance &&
            same_plane) {
          cluster.push_back(i);
          grouped = true;
          break;
        }
      }
      if (grouped) break;
    }
    if (!grouped) clusters.push_back({i});
  }

  struct ReducedCluster {
    const std::vector<std::size_t>* members;
    std::vector<std::size_t> chosen;
    Vec3s centroid;
    Scalar deepest;
    std::uint64_t patch_id;
  };
  std::vector<std::size_t> cluster_order(clusters.size());
  std::iota(cluster_order.begin(), cluster_order.end(), 0);
  const auto cluster_deepest = [&](std::size_t cluster_index) {
    Scalar deepest = (std::numeric_limits<Scalar>::max)();
    for (std::size_t i : clusters[cluster_index])
      deepest = std::min(deepest, candidates[i].penetration_depth);
    return deepest;
  };
  std::sort(cluster_order.begin(), cluster_order.end(),
            [&](std::size_t a, std::size_t b) {
              const Scalar depth_a = cluster_deepest(a);
              const Scalar depth_b = cluster_deepest(b);
              if (depth_a != depth_b) return depth_a < depth_b;
              return clusters[a].size() > clusters[b].size();
            });

  std::unordered_set<std::uint64_t> claimed_patch_ids;
  std::vector<ReducedCluster> reduced;
  reduced.reserve(std::min(maximum, clusters.size()));
  for (std::size_t cluster_index : cluster_order) {
    if (reduced.size() >= maximum) break;
    const auto& cluster = clusters[cluster_index];
    std::uint64_t patch_id = 0;
    for (std::size_t i : cluster)
      if (matched[i] >= 0 &&
          claimed_patch_ids.count(previous[matched[i]].patch_id) == 0) {
        patch_id = previous[matched[i]].patch_id;
        break;
      }
    if (!patch_id) patch_id = next_patch_id_++;
    claimed_patch_ids.insert(patch_id);
    Vec3s cluster_centroid = Vec3s::Zero();
    Scalar deepest = (std::numeric_limits<Scalar>::max)();
    for (std::size_t i : cluster) {
      cluster_centroid += candidates[i].pos;
      deepest = std::min(deepest, candidates[i].penetration_depth);
    }
    cluster_centroid /= static_cast<Scalar>(cluster.size());
    // On a flat patch many candidates have identical depth. Starting from an
    // arbitrary traversal candidate lets persistent IDs drift to one side of
    // a circular footprint. Start at the patch perimeter while still keeping
    // the deepest layer, then use farthest-point coverage.
    constexpr Scalar kDepthTieTolerance = Scalar(1e-5);
    std::size_t first = cluster.front();
    Scalar farthest_from_centroid = Scalar(-1);
    for (std::size_t i : cluster) {
      if (candidates[i].penetration_depth > deepest + kDepthTieTolerance)
        continue;
      const Scalar distance =
          (candidates[i].pos - cluster_centroid).squaredNorm();
      if (distance > farthest_from_centroid) {
        first = i;
        farthest_from_centroid = distance;
      }
    }
    std::vector<std::size_t> chosen{first};
    while (chosen.size() < 4 && chosen.size() < cluster.size()) {
      std::size_t best = candidates.size();
      Scalar best_distance = Scalar(1e-16);
      for (std::size_t i : cluster) {
        if (std::find(chosen.begin(), chosen.end(), i) != chosen.end())
          continue;
        Scalar distance = (std::numeric_limits<Scalar>::max)();
        for (std::size_t j : chosen)
          distance = std::min(
              distance, (candidates[i].pos - candidates[j].pos).squaredNorm());
        if (distance > best_distance) {
          best = i;
          best_distance = distance;
        }
      }
      if (best == candidates.size()) break;
      chosen.push_back(best);
    }
    reduced.push_back(
        {&cluster, std::move(chosen), cluster_centroid, deepest, patch_id});
  }

  std::vector<HeightFieldTrackedContact> selected;
  std::unordered_set<std::uint64_t> claimed_contact_ids;
  for (const auto& patch : reduced) {
    for (const std::size_t i : patch.chosen) {
      if (selected.size() >= maximum) break;
      std::uint64_t contact_id = 0;
      if (matched[i] >= 0 &&
          claimed_contact_ids.count(previous[matched[i]].contact_id) == 0) {
        contact_id = previous[matched[i]].contact_id;
      }
      if (!contact_id) contact_id = next_contact_id_++;
      claimed_contact_ids.insert(contact_id);
      selected.push_back({candidates[i], contact_id, patch.patch_id});
      entry.anchors.push_back({
          terrain.getTransform().inverseTransform(
              candidates[i].nearest_points[0]),
          shape.getTransform().inverseTransform(candidates[i].nearest_points[1]),
          terrain.getTransform().rotation().transpose() *
              candidates[i].normal,
          candidates[i].penetration_depth, contact_id, patch.patch_id});
    }
    if (selected.size() >= maximum) break;
  }
  return selected;
}

auto HeightFieldContactCallback::contactToPointPair(
    const Contact& contact, sire::geometry::GeometryId id_A,
    sire::geometry::GeometryId id_B) -> common::PenetrationAsPointPair {
  common::PenetrationAsPointPair pair;
  pair.id_A = id_A;
  pair.id_B = id_B;
  pair.p_WCa = contact.nearest_points[0];
  pair.p_WCb = contact.nearest_points[1];
  pair.p_WC = contact.pos;
  pair.nhat_AB_W = contact.normal;
  pair.depth = std::abs(contact.penetration_depth);
  pair.modifiedDepth = pair.depth;
  return pair;
}

auto HeightFieldContactCallback::collide(
    CollisionObject* fcl_object_A_ptr,
    CollisionObject* fcl_object_B_ptr) -> bool {
  SIRE_DEMAND(point_pairs_ != nullptr);

  // 统一 id 顺序：id_A < id_B
  sire::geometry::GeometryId id_A = filter_->queryGeometryIdByPtr(
      fcl_object_A_ptr->collisionGeometry().get());
  sire::geometry::GeometryId id_B = filter_->queryGeometryIdByPtr(
      fcl_object_B_ptr->collisionGeometry().get());

  if (id_A > id_B) {
    std::swap(id_A, id_B);
    std::swap(fcl_object_A_ptr, fcl_object_B_ptr);
  }

  if (!filter_->canCollideWith(fcl_object_A_ptr, fcl_object_B_ptr))
    return false;

  // 判断是否涉及 HeightField
  const bool is_hfield =
      fcl_object_A_ptr->collisionGeometry()->getObjectType() == OT_HFIELD ||
      fcl_object_B_ptr->collisionGeometry()->getObjectType() == OT_HFIELD;

  if (is_hfield) {
    const bool terrain_is_A =
        fcl_object_A_ptr->collisionGeometry()->getObjectType() == OT_HFIELD;
    const CollisionObject& terrain =
        terrain_is_A ? *fcl_object_A_ptr : *fcl_object_B_ptr;
    const CollisionObject& shape =
        terrain_is_A ? *fcl_object_B_ptr : *fcl_object_A_ptr;
    const auto type = shape.collisionGeometry()->getNodeType();
    const bool patch_shape =
        type == GEOM_BOX || type == GEOM_CAPSULE || type == GEOM_CYLINDER ||
        type == GEOM_CONE || type == GEOM_ELLIPSOID ||
        type == GEOM_CONVEX16 || type == GEOM_CONVEX32;
    if (terrain.collisionGeometry()->getNodeType() == HF_AABB &&
        (type == GEOM_SPHERE || type == GEOM_BOX ||
         (use_generic_patch_ && patch_shape))) {
      SIRE_PROFILE_SCOPE("height_field/generate_and_reduce");
      std::vector<HeightFieldTrackedContact> selected;
      bool contacts_are_terrain_shape_ordered = false;
      if (use_generic_patch_) {
        std::vector<Contact> contacts;
        if (use_prism_contacts_) {
          SIRE_PROFILE_SCOPE("height_field/prism_candidates");
          HeightFieldPrismQueryStats stats;
          contacts = generateHeightFieldPrismContacts(
              terrain, shape, prism_max_contacts_, &stats);
          SIRE_PROFILE_PLOT("height_field.prism.cells",
                            static_cast<std::int64_t>(stats.cells_visited));
          SIRE_PROFILE_PLOT("height_field.prism.tests",
                            static_cast<std::int64_t>(stats.prisms_tested));
          SIRE_PROFILE_PLOT("height_field.prism.candidates",
                            static_cast<std::int64_t>(contacts.size()));
          SIRE_PROFILE_PLOT("height_field.prism.limit_reached",
                            static_cast<std::int64_t>(stats.limit_reached));
          contacts_are_terrain_shape_ordered = true;
        } else if (type == GEOM_SPHERE) {
          // The analytic sphere path includes triangle-edge fallback and does
          // not lose contact after the centre crosses a rough cell boundary.
          contacts = generateHeightFieldContacts(terrain, shape,
                                                 kMaxContactsPerPair);
          contacts_are_terrain_shape_ordered = true;
        } else {
          CollisionRequest hf_req;
          hf_req.num_max_contacts = 512;
          hf_req.enable_contact = true;
          hf_req.enable_height_field_contact_patches = true;
          hf_req.gjk_tolerance = 2e-8;
          CollisionResult result;
          coal::collide(fcl_object_A_ptr, fcl_object_B_ptr, hf_req, result);
          contacts.reserve(result.numContacts());
          for (const Contact& contact : result.getContacts())
            if (std::abs(contact.penetration_depth) >
                std::numeric_limits<double>::epsilon())
              contacts.push_back(contact);
        }
        if (manifold_)
          selected = manifold_->select(terrain, shape, contacts,
                                       kMaxContactsPerPair);
        else
          for (const auto& contact : contacts)
            selected.push_back({contact, 0, 0});
      } else {
        const auto contacts = generateHeightFieldContacts(
            terrain, shape, kMaxContactsPerPair);
        contacts_are_terrain_shape_ordered = true;
        if (manifold_)
          selected = manifold_->select(terrain, shape, contacts,
                                       kMaxContactsPerPair);
        else
          for (const auto& contact : contacts)
            selected.push_back({contact, 0, 0});
      }
      for (const auto& tracked : selected) {
        Contact contact = tracked.contact;
        // The Sire generator returns contacts in terrain/shape order. COAL's
        // collision query already follows its input A/B order.
        if (contacts_are_terrain_shape_ordered && !terrain_is_A) {
          std::swap(contact.nearest_points[0], contact.nearest_points[1]);
          contact.normal *= -1;
        }
        auto pair = contactToPointPair(contact, id_A, id_B);
        pair.contact_id = tracked.contact_id;
        pair.patch_id = tracked.patch_id;
        point_pairs_->push_back(std::move(pair));
      }
    } else {
      // Other shape types retain the existing COAL narrow-phase behaviour.
      CollisionRequest hf_req;
      hf_req.num_max_contacts = kMaxContactsPerPair;
      hf_req.enable_contact = true;
      hf_req.gjk_tolerance = 2e-8;
      CollisionResult result;
      coal::collide(fcl_object_A_ptr, fcl_object_B_ptr, hf_req, result);
      for (const Contact& contact : result.getContacts()) {
        if (std::abs(contact.penetration_depth) >
            std::numeric_limits<double>::epsilon())
          point_pairs_->push_back(contactToPointPair(contact, id_A, id_B));
      }
    }

  } else {
    // ── 非 HeightField 路径：与原始 PenetrationAsPointPairCallback 一致 ─
    CollisionResult result;
    coal::collide(fcl_object_A_ptr, fcl_object_B_ptr, request, result);

    if (!result.isCollision()) return false;

    const Contact& contact = result.getContact(0);
    const double depth = std::abs(contact.penetration_depth);

    // FCL issue #375: osculation case, normal may be degenerate
    if (depth <= std::numeric_limits<double>::epsilon()) return false;

    common::PenetrationAsPointPair pair;
    pair.depth = depth;
    pair.modifiedDepth = depth;
    pair.nhat_AB_W = contact.normal;
    pair.p_WCa = contact.nearest_points[0];
    pair.p_WCb = contact.nearest_points[1];
    pair.p_WC = contact.pos;
    pair.id_A = id_A;
    pair.id_B = id_B;
    point_pairs_->push_back(std::move(pair));
  }

  // false = 不终止 broadphase 遍历
  return false;
}

HeightFieldContactCallback::HeightFieldContactCallback(
    CollisionFilter* filter_in,
    std::vector<common::PenetrationAsPointPair>* point_pairs_in,
    HeightFieldContactManifold* manifold, bool use_generic_patch,
    bool use_prism_contacts, std::size_t prism_max_contacts)
    : CollisionCallBackBase(),
      filter_(filter_in),
      point_pairs_(point_pairs_in),
      manifold_(manifold),
      use_generic_patch_(use_generic_patch || use_prism_contacts),
      use_prism_contacts_(use_prism_contacts),
      prism_max_contacts_(prism_max_contacts) {
  request.num_max_contacts = 1;
  request.enable_contact = true;
  request.gjk_tolerance = 2e-8;
}

}  // namespace sire::physics::collision
