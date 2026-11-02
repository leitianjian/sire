// Equal-surface narrow-phase comparison. No simulation, force solve, or render.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <coal/collision.h>
#include <coal/contact_patch.h>
#include <coal/hfield.h>
#include <coal/shape/geometric_shapes.h>

#include "sire/physics/collision/height_field_contact_callback.hpp"
#include "sire/physics/collision/height_field_contact_generator.hpp"

namespace {
using Clock = std::chrono::steady_clock;
using coal::Contact;
using coal::CollisionObject;
using coal::Vec3s;
volatile std::uint64_t sink = 0;
constexpr std::size_t cap = 4096;

auto collide(const CollisionObject& ground, const CollisionObject& shape,
             bool leaf_patch = false, std::size_t limit = cap)
    -> coal::CollisionResult {
  coal::CollisionRequest request;
  request.num_max_contacts = limit;
  request.enable_contact = true;
  request.gjk_tolerance = 2e-8;
  request.enable_height_field_contact_patches = leaf_patch;
  coal::CollisionResult result;
  coal::collide(&ground, &shape, request, result);
  return result;
}

auto candidates(const CollisionObject& ground, const CollisionObject& shape,
                bool heightfield, std::size_t limit = cap)
    -> std::vector<Contact> {
  if (heightfield) return collide(ground, shape, true, limit).getContacts();
  // Large num_max_contacts does not force generic convex collision to return
  // a manifold. Expand its witness with Coal's actual support-face algorithm.
  const auto raw = collide(ground, shape, false, limit);
  const coal::ContactPatchRequest request(
      std::max(std::size_t(1), raw.numContacts()), 16, coal::Scalar(1e-3));
  coal::ContactPatchResult patches(request);
  coal::computeContactPatch(&ground, &shape, raw, request, patches);
  std::vector<Contact> result;
  for (std::size_t p = 0; p < patches.numContactPatches(); ++p) {
    const auto& patch = patches.getContactPatch(p);
    const auto normal = patch.getNormal();
    for (std::size_t i = 0; i < patch.size(); ++i) {
      const Vec3s midpoint = patch.getPoint(i);
      const auto depth = patch.penetration_depth;
      result.emplace_back(ground.collisionGeometry().get(),
                          shape.collisionGeometry().get(), Contact::NONE,
                          Contact::NONE, midpoint - 0.5 * depth * normal,
                          midpoint + 0.5 * depth * normal, normal, depth);
    }
  }
  return result;
}

auto footprint_coverage(const std::vector<Contact>& contacts) -> double;

// Identical reducer on both representations: duplicate positions/normals,
// deepest perimeter point, farthest opposite point, then maximum hull area.
auto reduce(const std::vector<Contact>& input) -> std::vector<Contact> {
  std::vector<Contact> unique;
  for (const auto& contact : input) {
    bool duplicate = false;
    for (const auto& old : unique) {
      if ((contact.pos - old.pos).squaredNorm() < 1e-12 &&
          contact.normal.dot(old.normal) > 0.999999) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) unique.push_back(contact);
  }
  if (unique.empty()) return {};
  Vec3s centroid = Vec3s::Zero();
  double deepest = std::numeric_limits<double>::infinity();
  for (const auto& c : unique) {
    centroid += c.pos;
    deepest = std::min(deepest, c.penetration_depth);
  }
  centroid /= static_cast<double>(unique.size());
  std::size_t first = 0;
  double farthest = -1;
  for (std::size_t i = 0; i < unique.size(); ++i) {
    if (unique[i].penetration_depth > deepest + 1e-5) continue;
    const double distance = (unique[i].pos - centroid).squaredNorm();
    if (distance > farthest) {
      farthest = distance;
      first = i;
    }
  }
  std::vector<Contact> selected{unique[first]};
  while (selected.size() < 4 && selected.size() < unique.size()) {
    double best_distance = 1e-16;
    const Contact* best = nullptr;
    for (const auto& contact : unique) {
      double distance = std::numeric_limits<double>::infinity();
      for (const auto& old : selected)
        distance = std::min(distance, (contact.pos - old.pos).squaredNorm());
      if (distance <= 1e-16) continue;
      if (selected.size() >= 2) {
        auto with_candidate = selected;
        with_candidate.push_back(contact);
        distance = footprint_coverage(with_candidate);
      }
      if (distance > best_distance) {
        best = &contact;
        best_distance = distance;
      }
    }
    if (!best) break;
    selected.push_back(*best);
  }
  return selected;
}

auto footprint_coverage(const std::vector<Contact>& contacts) -> double {
  if (contacts.size() < 3) return 0;
  std::vector<coal::Vec2s> points;
  for (const auto& c : contacts) points.push_back(c.nearest_points[0].head<2>());
  std::sort(points.begin(), points.end(), [](const auto& a, const auto& b) {
    return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
  });
  const auto cross = [](const auto& a, const auto& b, const auto& c) {
    const coal::Vec2s ab = b - a, ac = c - a;
    return ab.x() * ac.y() - ab.y() * ac.x();
  };
  std::vector<coal::Vec2s> hull;
  for (const auto& p : points) {
    while (hull.size() >= 2 && cross(hull[hull.size()-2], hull.back(), p) <= 0)
      hull.pop_back();
    hull.push_back(p);
  }
  const auto lower_size = hull.size();
  for (auto it = points.rbegin() + 1; it != points.rend(); ++it) {
    while (hull.size() > lower_size && cross(hull[hull.size()-2], hull.back(), *it) <= 0)
      hull.pop_back();
    hull.push_back(*it);
  }
  double area = 0;
  for (std::size_t i = 1; i < hull.size(); ++i)
    area += hull[i-1].x() * hull[i].y() - hull[i-1].y() * hull[i].x();
  return std::abs(area) / (2 * 0.4 * 0.3);
}

auto validate(const std::vector<Contact>& contacts, const CollisionObject& shape,
              bool box) -> void {
  if (contacts.size() != (box ? 4u : 1u))
    throw std::runtime_error("Equivalent-surface support count mismatch");
  for (const auto& c : contacts) {
    if (!c.pos.allFinite() || std::abs(c.nearest_points[0].z()) > 2e-5 ||
        c.normal.z() < 0.99999 ||
        std::abs((c.nearest_points[1] - c.nearest_points[0]).dot(c.normal) -
                 c.penetration_depth) > 2e-5)
      throw std::runtime_error("Invalid contact witness");
  }
  for (const auto& c : contacts) {
    const Vec3s local = shape.getTransform().inverseTransform(c.nearest_points[1]);
    if (box && (std::abs(local.x()) > 0.2 + 2e-5 ||
                std::abs(local.y()) > 0.15 + 2e-5 ||
                std::abs(local.z() + 0.05) > 2e-5))
      throw std::runtime_error("Contact lies outside box surface");
    if (!box && std::abs(local.norm() - 0.05) > 2e-5)
      throw std::runtime_error("Contact lies outside sphere surface");
  }
}

template <class F>
auto measure(F&& fn) -> double {
  constexpr int calls = 96, batches = 9;
  for (int i = 0; i < 96; ++i) sink += fn(i % 32);
  std::vector<double> times;
  for (int batch = 0; batch < batches; ++batch) {
    const auto start = Clock::now();
    for (int i = 0; i < calls; ++i) sink += fn(i % 32);
    times.push_back(std::chrono::duration<double, std::micro>(
                        Clock::now() - start).count() / calls);
  }
  std::sort(times.begin(), times.end());
  return times[times.size() / 2];
}

auto run_case(int grid, bool box, bool heightfield) -> void {
  std::shared_ptr<coal::CollisionGeometry> ground_geometry;
  if (heightfield) {
    ground_geometry = std::make_shared<coal::HeightField<coal::AABB>>(
        10, 10, coal::MatrixXs::Zero(grid, grid), -0.2);
  } else {
    ground_geometry = std::make_shared<coal::Box>(10, 10, 0.2);
  }
  CollisionObject ground(ground_geometry, coal::Transform3s(
      Vec3s(0, 0, heightfield ? 0 : -0.1)));
  std::shared_ptr<coal::CollisionGeometry> shape_geometry = box
      ? std::static_pointer_cast<coal::CollisionGeometry>(
          std::make_shared<coal::Box>(0.4, 0.3, 0.1))
      : std::static_pointer_cast<coal::CollisionGeometry>(
          std::make_shared<coal::Sphere>(0.05));
  std::vector<CollisionObject> shapes;
  for (int i = 0; i < 32; ++i)
    shapes.emplace_back(shape_geometry, coal::Transform3s(Vec3s(
        -0.2 + (i % 8) * 0.053, -0.12 + (i / 8) * 0.071, 0.049)));
  std::vector<std::vector<Contact>> cache;
  double raw_count = 0, candidate_count = 0, production_count = 0;
  double coverage_sum = 0, coverage_min = 1, candidate_coverage_sum = 0;
  double production_coverage_sum = 0, production_coverage_min = 1;
  for (const auto& shape : shapes) {
    const auto raw = collide(ground, shape);
    const auto expanded = candidates(ground, shape, heightfield);
    // Test all sampled poses at a doubled cap. A capped output is an invalid
    // sample rather than a fast measurement.
    if (raw.numContacts() >= cap || expanded.size() >= cap ||
        raw.numContacts() != collide(ground, shape, false, 2 * cap).numContacts() ||
        expanded.size() != candidates(ground, shape, heightfield, 2 * cap).size())
      throw std::runtime_error("Contact cap truncates the candidate set");
    if (heightfield && box &&
        expanded.size() != candidates(ground, shape, true, 512).size())
      throw std::runtime_error("Production 512-contact cap truncates candidates");
    const auto selected = reduce(expanded);
    validate(selected, shape, box);
    const double coverage = box ? footprint_coverage(selected) : 1;
    candidate_coverage_sum += box ? footprint_coverage(expanded) : 1;
    coverage_sum += coverage;
    coverage_min = std::min(coverage_min, coverage);
    raw_count += raw.numContacts();
    candidate_count += expanded.size();
    cache.push_back(expanded);
  }
  const auto raw_us = measure([&](int i) {
    return collide(ground, shapes[i]).numContacts();
  });
  const auto candidate_us = measure([&](int i) {
    return candidates(ground, shapes[i], heightfield).size();
  });
  const auto reduce_us = measure([&](int i) { return reduce(cache[i]).size(); });
  const auto total_us = measure([&](int i) {
    return reduce(candidates(ground, shapes[i], heightfield)).size();
  });
  double production_candidates_us = 0, production_manifold_us = 0;
  double production_total_us = 0;
  if (heightfield) {
    cache.clear();
    for (const auto& shape : shapes) {
      cache.push_back(box ? candidates(ground, shape, true)
          : sire::physics::collision::generateHeightFieldContacts(ground, shape, 4));
      production_count += cache.back().size();
    }
    production_candidates_us = measure([&](int i) {
      return box ? candidates(ground, shapes[i], true).size()
          : sire::physics::collision::generateHeightFieldContacts(
              ground, shapes[i], 4).size();
    });
    // Independent histories per pose. Shared shape geometry pointers must not
    // alias the state of unrelated sampled poses in this measurement.
    std::vector<sire::physics::collision::HeightFieldContactManifold> manifolds(32);
    for (int i = 0; i < 32; ++i) {
      auto& manifold = manifolds[i];
      manifold.beginQuery();
      const auto selected = manifold.select(ground, shapes[i], cache[i], 4);
      manifold.endQuery();
      std::vector<Contact> contacts;
      for (const auto& item : selected) contacts.push_back(item.contact);
      validate(contacts, shapes[i], box);
      const double coverage = box ? footprint_coverage(contacts) : 1;
      production_coverage_sum += coverage;
      production_coverage_min = std::min(production_coverage_min, coverage);
    }
    production_manifold_us = measure([&](int i) {
      auto& manifold = manifolds[i];
      manifold.beginQuery();
      const auto selected = manifold.select(ground, shapes[i], cache[i], 4);
      manifold.endQuery();
      return selected.size();
    });
    production_total_us = measure([&](int i) {
      auto input = box ? candidates(ground, shapes[i], true)
          : sire::physics::collision::generateHeightFieldContacts(ground, shapes[i], 4);
      auto& manifold = manifolds[i];
      manifold.beginQuery();
      const auto selected = manifold.select(ground, shapes[i], input, 4);
      manifold.endQuery();
      return selected.size();
    });
  }
  std::cout << (box ? "box" : "sphere") << ','
            << (heightfield ? "heightfield" : "box_ground") << ',' << grid
            << ',' << cap << ',' << raw_count / 32 << ',' << candidate_count / 32
            << ',' << (box ? 4 : 1) << ',' << raw_us << ',' << candidate_us
            << ',' << reduce_us << ',' << total_us << ',' << coverage_sum / 32
            << ',' << coverage_min << ',' << candidate_coverage_sum / 32;
  if (heightfield)
    std::cout << ',' << production_count / 32 << ',' << production_candidates_us
              << ',' << production_manifold_us << ',' << production_total_us
              << ',' << production_coverage_sum / 32 << ',' << production_coverage_min;
  else std::cout << ",,,,,,";
  std::cout << '\n';
}
}  // namespace

int main() {
  try {
    std::cout << std::fixed << std::setprecision(3)
              << "shape,ground,grid,query_cap,raw_mean,candidate_mean,selected,"
                 "raw_query_us,candidate_generation_us,common_reduce_us,"
                 "common_pipeline_us,coverage_mean,coverage_min,candidate_coverage_mean,"
                 "production_candidate_mean,"
                 "production_candidates_us,production_manifold_us,"
                 "production_geometry_pipeline_us,production_coverage_mean,"
                 "production_coverage_min\n";
    for (bool box : {false, true}) {
      run_case(0, box, false);
      for (int grid : {65, 129, 257}) run_case(grid, box, true);
    }
    return sink ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
