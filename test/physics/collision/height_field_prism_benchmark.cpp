#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <coal/collision.h>
#include <coal/hfield.h>
#include <coal/shape/geometric_shapes.h>

#include "sire/physics/collision/height_field_contact_callback.hpp"
#include "sire/physics/collision/height_field_contact_generator.hpp"

namespace {
using coal::Contact;
using coal::Vec3s;
using namespace sire::physics::collision;
volatile std::size_t sink = 0;

auto area(const std::vector<Contact>& contacts) -> double {
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
  const auto lower = hull.size();
  for (auto it = points.rbegin() + 1; it != points.rend(); ++it) {
    while (hull.size() > lower && cross(hull[hull.size()-2], hull.back(), *it) <= 0)
      hull.pop_back();
    hull.push_back(*it);
  }
  double result = 0;
  for (std::size_t i = 1; i < hull.size(); ++i)
    result += hull[i-1].x() * hull[i].y() - hull[i-1].y() * hull[i].x();
  return std::abs(result) / 2;
}

template <class F> auto measure(F&& fn) -> double {
  for (int i = 0; i < 64; ++i) sink += fn(i % 32);
  std::vector<double> batches;
  for (int batch = 0; batch < 7; ++batch) {
    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < 64; ++i) sink += fn(i % 32);
    batches.push_back(std::chrono::duration<double, std::micro>(
        std::chrono::steady_clock::now() - start).count() / 64);
  }
  std::sort(batches.begin(), batches.end());
  return batches[batches.size() / 2];
}

auto oldCandidates(const coal::CollisionObject& terrain,
                    const coal::CollisionObject& shape) -> std::vector<Contact> {
  if (shape.collisionGeometry()->getNodeType() == coal::GEOM_SPHERE)
    return generateHeightFieldContacts(terrain, shape, 4);
  coal::CollisionRequest request;
  request.num_max_contacts = 4096;
  request.gjk_tolerance = 2e-8;
  request.enable_height_field_contact_patches = true;
  coal::CollisionResult result;
  coal::collide(&terrain, &shape, request, result);
  return result.getContacts();
}

auto run(int grid, const std::string& name) -> void {
  coal::CollisionObject terrain(std::make_shared<coal::HeightField<coal::AABB>>(
      10, 10, coal::MatrixXs::Zero(grid, grid), -0.2));
  std::shared_ptr<coal::CollisionGeometry> geometry;
  if (name == "box") geometry = std::make_shared<coal::Box>(0.4, 0.3, 0.1);
  if (name == "sphere") geometry = std::make_shared<coal::Sphere>(0.05);
  if (name == "capsule") geometry = std::make_shared<coal::Capsule>(0.03, 0.04);
  if (name == "cylinder") geometry = std::make_shared<coal::Cylinder>(0.05, 0.1);
  std::vector<coal::CollisionObject> shapes;
  for (int i = 0; i < 32; ++i)
    shapes.emplace_back(geometry, coal::Transform3s(Vec3s(
        -0.2 + (i % 8) * 0.053, -0.12 + (i / 8) * 0.071, 0.049)));
  for (int method = 0; method < 3; ++method) {
    const std::size_t cap = method == 2 ? 50 : 4096;
    const auto generate = [&](int i, HeightFieldPrismQueryStats* stats = nullptr) {
      return method == 0 ? oldCandidates(terrain, shapes[i]) :
          generateHeightFieldPrismContacts(terrain, shapes[i], cap, stats);
    };
    std::vector<std::vector<Contact>> cache;
    std::vector<HeightFieldContactManifold> manifolds(32);
    double count = 0, chosen = 0, coverage = 0, minimum = 1;
    double cells = 0, prisms = 0, rejected = 0, capped_poses = 0;
    for (int i = 0; i < 32; ++i) {
      HeightFieldPrismQueryStats stats;
      cache.push_back(generate(i, &stats));
      if (cache.back().empty()) throw std::runtime_error("Missed flat-ground overlap");
      if (method == 1) {
        HeightFieldPrismQueryStats audit;
        const auto doubled = generateHeightFieldPrismContacts(terrain, shapes[i], 8192, &audit);
        if (stats.limit_reached || audit.limit_reached || doubled.size() != cache.back().size())
          throw std::runtime_error("Full-query benchmark is truncated");
      }
      count += cache.back().size();
      cells += stats.cells_visited;
      prisms += stats.prisms_tested;
      rejected += stats.prisms_height_rejected;
      capped_poses += stats.limit_reached;
      for (const auto& c : cache.back())
        if (!c.pos.allFinite() || !c.normal.allFinite() ||
            std::abs((c.nearest_points[1] - c.nearest_points[0]).dot(c.normal) -
                     c.penetration_depth) > 2e-5)
          throw std::runtime_error("Invalid geometry witness");
      auto& manifold = manifolds[i];
      manifold.beginQuery();
      const auto selected = manifold.select(terrain, shapes[i], cache.back(), 4);
      manifold.endQuery();
      chosen += selected.size();
      std::vector<Contact> selected_contacts;
      for (const auto& c : selected) selected_contacts.push_back(c.contact);
      const double ratio = name == "box" ? area(selected_contacts) / (0.4 * 0.3) : 0;
      coverage += ratio;
      minimum = std::min(minimum, ratio);
    }
    const auto generation = measure([&](int i) { return generate(i).size(); });
    const auto total = measure([&](int i) {
      auto input = generate(i);
      auto& manifold = manifolds[i];
      manifold.beginQuery();
      const auto selected = manifold.select(terrain, shapes[i], input, 4);
      manifold.endQuery();
      return selected.size();
    });
    std::cout << name << ',' << grid << ','
              << (method == 0 ? "old_patch" : method == 1 ? "prism_full" : "prism_50")
              << ',' << cap << ',' << count/32 << ',' << chosen/32 << ','
              << generation << ',' << total << ',' << coverage/32 << ',' << minimum
              << ',' << cells/32 << ',' << prisms/32 << ',' << rejected/32
              << ',' << capped_poses << '\n';
  }
}
}  // namespace

int main() {
  try {
    std::cout << std::fixed << std::setprecision(3)
              << "shape,grid,method,cap,candidates_mean,selected_mean,generation_us,"
                 "geometry_pipeline_us,box_coverage_mean,box_coverage_min,"
                 "cells_mean,prisms_tested_mean,height_rejected_mean,capped_poses\n";
    for (const std::string name : {"box", "sphere", "capsule", "cylinder"})
      for (int grid : {2, 5, 65, 129, 257}) run(grid, name);
    return sink ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
