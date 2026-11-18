#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include <coal/collision.h>
#include <coal/collision_object.h>
#include <coal/hfield.h>
#include <coal/shape/geometric_shapes.h>

#include "sire/physics/collision/height_field_contact_generator.hpp"

namespace {
using Clock = std::chrono::steady_clock;
using coal::Scalar;

volatile std::uint64_t contact_sink = 0;

auto terrainHeight(Scalar x, Scalar y, bool rough) -> Scalar {
  return rough ? Scalar(0.03) + Scalar(0.006) * std::sin(4 * x) +
                     Scalar(0.004) * std::cos(3 * y)
               : Scalar(0);
}

auto makeTerrain(int grid_size, bool rough) -> coal::CollisionObject {
  coal::MatrixXs heights(grid_size, grid_size);
  const Scalar spacing = Scalar(4) / (grid_size - 1);
  for (int row = 0; row < grid_size; ++row) {
    for (int col = 0; col < grid_size; ++col) {
      heights(row, col) = terrainHeight(-2 + col * spacing,
                                         2 - row * spacing, rough);
    }
  }
  return coal::CollisionObject(
      std::make_shared<coal::HeightField<coal::AABB>>(4, 4, heights, -0.2));
}

auto makeShapes(bool box, bool rough) -> std::vector<coal::CollisionObject> {
  std::vector<coal::CollisionObject> shapes;
  shapes.reserve(64);
  std::shared_ptr<coal::CollisionGeometry> geometry =
      box ? std::static_pointer_cast<coal::CollisionGeometry>(
                std::make_shared<coal::Box>(0.12, 0.08, 0.06))
          : std::static_pointer_cast<coal::CollisionGeometry>(
                std::make_shared<coal::Sphere>(0.05));
  for (int i = 0; i < 64; ++i) {
    const Scalar x = -0.48 + Scalar(i % 8) * 0.13;
    const Scalar y = -0.42 + Scalar(i / 8) * 0.12;
    const Scalar z = terrainHeight(x, y, rough) +
                     (box ? Scalar(0.026) : Scalar(0.046));
    shapes.emplace_back(geometry, coal::Transform3s(coal::Vec3s(x, y, z)));
  }
  return shapes;
}

auto queryCoal(const coal::CollisionObject& terrain,
               const coal::CollisionObject& shape,
               const coal::CollisionRequest& request) -> std::size_t {
  coal::CollisionResult result;
  coal::collide(&terrain, &shape, request, result);
  return result.numContacts();
}

auto querySire(const coal::CollisionObject& terrain,
               const coal::CollisionObject& shape) -> std::size_t {
  return sire::physics::collision::generateHeightFieldContacts(terrain, shape,
                                                               8)
      .size();
}

auto queryGenericPatch(const coal::CollisionObject& terrain,
                       const coal::CollisionObject& shape) -> std::size_t {
  return sire::physics::collision::generateHeightFieldPatchContacts(terrain,
                                                                    shape, 4)
      .size();
}

struct Summary {
  double median_us;
  double mean_contacts;
  std::size_t min_contacts;
  std::size_t max_contacts;
};

template <typename Query>
auto measure(const coal::CollisionObject& terrain,
             const std::vector<coal::CollisionObject>& shapes, Query&& query)
    -> Summary {
  constexpr int warmup_calls = 256;
  constexpr int batches = 7;
  constexpr int calls_per_batch = 256;
  for (int i = 0; i < warmup_calls; ++i)
    contact_sink += query(terrain, shapes[i % shapes.size()]);

  std::vector<double> timings;
  std::uint64_t total_contacts = 0;
  std::size_t minimum = (std::numeric_limits<std::size_t>::max)();
  std::size_t maximum = 0;
  for (int batch = 0; batch < batches; ++batch) {
    const auto start = Clock::now();
    for (int i = 0; i < calls_per_batch; ++i) {
      const std::size_t contacts =
          query(terrain, shapes[(batch * calls_per_batch + i) % shapes.size()]);
      total_contacts += contacts;
      minimum = std::min(minimum, contacts);
      maximum = std::max(maximum, contacts);
      contact_sink += contacts;
    }
    const auto elapsed = std::chrono::duration<double, std::micro>(
                             Clock::now() - start)
                             .count();
    timings.push_back(elapsed / calls_per_batch);
  }
  std::sort(timings.begin(), timings.end());
  return {timings[timings.size() / 2],
          static_cast<double>(total_contacts) / (batches * calls_per_batch),
          minimum, maximum};
}

}  // namespace

int main(int argc, char** argv) {
  const std::string dll_label = argc > 1 ? argv[1] : "unspecified";
  coal::CollisionRequest request;
  request.num_max_contacts = 8;
  request.enable_contact = true;
  request.gjk_tolerance = 2e-8;

  std::cout << "dll,grid,terrain,shape,method,median_us,mean_contacts,"
               "min_contacts,max_contacts\n";
  for (int grid_size : {65, 129, 257}) {
    for (bool rough : {false, true}) {
      auto terrain = makeTerrain(grid_size, rough);
      for (bool box : {false, true}) {
        const auto shapes = makeShapes(box, rough);
        // Reverse method order on alternating cases to limit clock/order bias.
        Summary coal_result, sire_result;
        const auto old_query = [&](const auto& t, const auto& s) {
          return queryCoal(t, s, request);
        };
        auto patch_request = request;
        patch_request.num_max_contacts = 512;
        patch_request.enable_height_field_contact_patches = true;
        const auto coal_leaf_patch_query = [&](const auto& t, const auto& s) {
          return queryCoal(t, s, patch_request);
        };
        if (box == rough) {
          coal_result = measure(terrain, shapes, old_query);
          sire_result = measure(terrain, shapes, querySire);
        } else {
          sire_result = measure(terrain, shapes, querySire);
          coal_result = measure(terrain, shapes, old_query);
        }
        for (const auto& entry :
             {std::pair<const char*, Summary>{"coal_8", coal_result},
              {"sire_selected", sire_result}}) {
          const Summary& s = entry.second;
          std::cout << dll_label << ',' << grid_size << ','
                    << (rough ? "rough" : "flat") << ','
                    << (box ? "box" : "sphere") << ',' << entry.first << ','
                    << std::fixed << std::setprecision(3) << s.median_us << ','
                    << s.mean_contacts << ',' << s.min_contacts << ','
                    << s.max_contacts << '\n';
        }
        if (grid_size == 65) {
          const Summary generic = measure(terrain, shapes, queryGenericPatch);
          const Summary coal_leaf_patch =
              measure(terrain, shapes, coal_leaf_patch_query);
          std::cout << dll_label << ',' << grid_size << ','
                    << (rough ? "rough" : "flat") << ','
                    << (box ? "box" : "sphere") << ",generic_patch_4,"
                    << std::fixed << std::setprecision(3)
                    << generic.median_us << ',' << generic.mean_contacts << ','
                    << generic.min_contacts << ',' << generic.max_contacts
                    << '\n';
          std::cout << dll_label << ',' << grid_size << ','
                    << (rough ? "rough" : "flat") << ','
                    << (box ? "box" : "sphere")
                    << ",coal_leaf_patch_raw," << std::fixed
                    << std::setprecision(3) << coal_leaf_patch.median_us << ','
                    << coal_leaf_patch.mean_contacts << ','
                    << coal_leaf_patch.min_contacts << ','
                    << coal_leaf_patch.max_contacts << '\n';
        }
      }
    }
  }
  return contact_sink == 0 ? 1 : 0;
}
