#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include <aris/core/serialization.hpp>
#include <aris/ext/json.hpp>
#include <coal/collision.h>
#include <coal/shape/geometric_shapes.h>

#include "flat_ground_model.hpp"
#include "../contact/cuda_test_buffer.hpp"
#include "sire/simulator/simulator.hpp"

namespace {

using namespace sire::physics::contact::cuda;
using sire::test::DeviceBuffer;
using sire::test::checkCuda;

struct CollisionBatch {
  std::vector<ContactHistoryConfig> configs;
  std::vector<WorldContact> contacts;
  std::vector<GroundCollisionStatus> status;
  std::vector<DetectedContact> framed;
};

class Stream {
 public:
  Stream() { checkCuda(cudaStreamCreateWithFlags(&value_, cudaStreamNonBlocking)); }
  ~Stream() { cudaStreamDestroy(value_); }
  Stream(const Stream&) = delete;
  auto get() const -> cudaStream_t { return value_; }

 private:
  cudaStream_t value_{};
};

auto runCollision(const FlatGroundModel& model, std::size_t count, std::size_t capacity,
                  const std::vector<double>& poses, const std::vector<double>& velocities = {})
    -> CollisionBatch {
  DeviceBuffer<GroundCollider> colliders(model.colliders.size());
  colliders.upload(model.colliders);
  DeviceBuffer<double> body_poses(count * model.body_count * 12);
  body_poses.upload(poses);
  DeviceBuffer<double> body_velocities(count * model.body_count * 6);
  if (!velocities.empty()) body_velocities.upload(velocities);
  DeviceBuffer<ContactHistoryConfig> configs(count);
  configs.upload(std::vector<ContactHistoryConfig>(count, {0, 0.001, 1000, 1e-8, false, true, false}));
  DeviceBuffer<WorldContact> contacts(count * capacity);
  DeviceBuffer<GroundCollisionStatus> status(count);
  DeviceBuffer<DetectedContact> framed(count * capacity);
  Stream stream;
  checkCuda(launchFlatGroundCollision(count, model.body_count, capacity, model.ground,
      colliders.data(), model.colliders.size(), body_poses.data(), configs.data(), contacts.data(),
      status.data(), stream.get()));
  if (!velocities.empty()) {
    checkCuda(launchFrameContacts(count, capacity, model.body_count, configs.data(), contacts.data(),
                                  body_velocities.data(), framed.data(), stream.get()));
  }
  checkCuda(cudaStreamSynchronize(stream.get()));
  return {configs.download(), contacts.download(), status.download(),
          velocities.empty() ? std::vector<DetectedContact>{} : framed.download()};
}

struct Errors {
  double point{0}, depth{0}, normal{0}, velocity{0};
  std::size_t witnesses{0};
};

void compare(const WorldContact& actual, const std::array<double, 3>& point,
             const std::array<double, 3>& normal, double depth, Errors& errors) {
  double point_sq = 0, normal_sq = 0;
  for (int k = 0; k < 3; ++k) {
    point_sq += std::pow(actual.point[k] - point[k], 2);
    normal_sq += std::pow(actual.normal[k] - normal[k], 2);
  }
  const double p = std::sqrt(point_sq), n = std::sqrt(normal_sq), d = std::abs(actual.depth - depth);
  if (!(p <= 2e-5 && d <= 2e-8 && n <= 1e-8)) {
    std::cerr << "geometry " << actual.geometry_b << ", errors in point, depth and normal: "
              << p << ", " << d << ", " << n << '\n';
    throw std::runtime_error("GPU collision witness differs from Sire");
  }
  errors.point = std::max(errors.point, p);
  errors.depth = std::max(errors.depth, d);
  errors.normal = std::max(errors.normal, n);
  ++errors.witnesses;
}

auto transform(const double* pm) -> coal::Transform3s {
  coal::Matrix3s rotation;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) rotation(i, j) = pm[4 * i + j];
  }
  return {rotation, coal::Vec3s(pm[3], pm[7], pm[11])};
}

void synthetic() {
  constexpr std::size_t count = 1024, capacity = 3;
  const GroundBox ground{0, 0, {0, 0, -0.1}, {1, 1, 0.1}};
  const double identity[12]{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
  FlatGroundModel model{ground, 4, {
      {1, 1, GroundShape::Sphere, {}, {0.05}, 2e8, 5000, 0.3},
      {2, 2, GroundShape::Box, {}, {0.03, 0.02, 0.015}, 2e8, 5000, 0.4},
      {3, 3, GroundShape::Cylinder, {}, {0.018, 0.04}, 2e8, 5000, 0.5}}};
  for (auto& shape : model.colliders) std::copy_n(identity, 12, shape.local_pose);
  std::vector<double> poses(count * model.body_count * 12);
  for (std::size_t i = 0; i < count; ++i) {
    auto* pose = poses.data() + i * model.body_count * 12;
    std::copy_n(identity, 12, pose);
    const double x = i % 4 == 0 ? 1.024 : i % 4 == 1 ? 1.1 : 0.0;
    const double y = i % 8 == 0 ? 1.021 : 0.1;
    std::copy_n(identity, 12, pose + 12);
    pose[15] = x;
    pose[19] = y;
    pose[23] = i % 7 == 0 ? -0.206 : 0.005 * (i % 16);
    for (std::size_t j = 2; j < 4; ++j) {
      const double ax = 0.2 + 0.001 * (i % 43), ay = 0.45 + 0.004 * (i % 67);
      const double cx = std::cos(ax), sx = std::sin(ax), cy = std::cos(ay), sy = std::sin(ay);
      const double z = i % 13 == 0 ? -0.205 : 0.005 * ((i + j) % 13);
      const double pm[12]{cy, sy * sx, sy * cx, -0.27,
                          0, cx, -sx, 0.13,
                          -sy, cy * sx, cy * cx, z};
      std::copy_n(pm, 12, pose + j * 12);
    }
  }
  const auto result = runCollision(model, count, capacity, poses);
  const coal::Box slab(2, 2, 0.2), box(0.06, 0.04, 0.03);
  const coal::Sphere sphere(0.05);
  const coal::Cylinder cylinder(0.018, 0.08);
  const coal::CollisionGeometry* geometry[] = {&sphere, &box, &cylinder};
  const coal::Transform3s ground_pose(coal::Vec3s(0, 0, -0.1));
  coal::CollisionRequest request;
  // Use a tighter geometric reference for these small boxes. The Go2 trace
  // replay below uses the unmodified Sire collision request.
  request.gjk_tolerance = 1e-10;
  request.epa_tolerance = 1e-10;
  request.num_max_contacts = 1;
  request.enable_contact = true;
  Errors errors;
  for (std::size_t env = 0; env < count; ++env) {
    if (result.status[env] != GroundCollisionStatus::Success) throw std::runtime_error("Synthetic collision failed");
    std::size_t slot = 0;
    for (std::size_t j = 0; j < capacity; ++j) {
      coal::CollisionResult cpu;
      coal::collide(&slab, ground_pose, geometry[j], transform(poses.data() + (4 * env + j + 1) * 12),
                    request, cpu);
      if (!cpu.isCollision()) continue;
      const auto& witness = cpu.getContact(0);
      const double depth = std::abs(witness.penetration_depth);
      if (depth <= std::numeric_limits<double>::epsilon()) continue;
      if (slot >= result.configs[env].detected) throw std::runtime_error("Missing synthetic collision");
      const auto& contact = result.contacts[env * capacity + slot++];
      if (contact.geometry_b != j + 1 || contact.body_b != j + 1) {
        throw std::runtime_error("Wrong synthetic collision pair");
      }
      compare(contact, {witness.pos[0], witness.pos[1], witness.pos[2]},
              {witness.normal[0], witness.normal[1], witness.normal[2]}, depth, errors);
    }
    if (slot != result.configs[env].detected) throw std::runtime_error("Additional synthetic collision");
  }
  const auto small = runCollision(model, count, 1, poses);
  for (std::size_t env = 0; env < count; ++env) {
    if (result.configs[env].detected > 1 &&
        (small.status[env] != GroundCollisionStatus::CapacityExceeded || small.configs[env].detected != 2)) {
      throw std::runtime_error("Missing capacity error");
    }
  }
  poses[27] = 0.99;  // A dynamic box straddles the lateral boundary.
  const auto edge = runCollision(model, count, capacity, poses);
  if (edge.status[0] != GroundCollisionStatus::UnsupportedBoundary ||
      edge.configs[0].detected != capacity + 1 || edge.status[1] != GroundCollisionStatus::Success) {
    throw std::runtime_error("Wrong boundary error isolation");
  }
  std::cout << "Compared " << errors.witnesses << " synthetic GPU witnesses with COAL across "
            << count << " environments\n";
}

void replay(const FlatGroundModel& model, const std::filesystem::path& directory) {
  using Json = nlohmann::json;
  std::ifstream file(directory / "single_point_contact_history.jsonl");
  if (!file) throw std::runtime_error("Cannot read Sire contact trace");
  std::vector<Json> events;
  std::vector<double> poses, velocities;
  for (std::string line; std::getline(file, line);) {
    events.push_back(Json::parse(line));
    const auto& event = events.back();
    if (event.at("body_poses").size() != model.body_count ||
        event.at("body_velocities").size() != model.body_count) {
      throw std::runtime_error("Trace and scene body counts differ");
    }
    for (const auto& x : event.at("body_poses")) {
      const auto pose = x.get<std::array<double, 16>>();
      poses.insert(poses.end(), pose.begin(), pose.begin() + 12);
    }
    for (const auto& x : event.at("body_velocities")) {
      const auto velocity = x.get<std::array<double, 6>>();
      velocities.insert(velocities.end(), velocity.begin(), velocity.end());
    }
  }
  if (events.empty()) throw std::runtime_error("Empty Sire contact trace");
  const auto capacity = model.colliders.size();
  const auto result = runCollision(model, events.size(), capacity, poses, velocities);
  Errors errors;
  for (std::size_t env = 0; env < events.size(); ++env) {
    std::map<std::uint64_t, Json> expected;
    for (const auto& witness : events[env].at("detected")) {
      expected.emplace(witness.at("geometry_b").get<std::uint64_t>(), witness);
    }
    if (result.status[env] != GroundCollisionStatus::Success ||
        result.configs[env].detected != expected.size()) {
      throw std::runtime_error("Different Go2 collision count at event " + std::to_string(env));
    }
    for (std::size_t j = 0; j < result.configs[env].detected; ++j) {
      const auto index = env * capacity + j;
      const auto& collision = result.contacts[index];
      const auto& ref = expected.at(collision.geometry_b);
      if (collision.geometry_a != ref.at("geometry_a") || collision.body_a != ref.at("body_a") ||
          collision.body_b != ref.at("body_b") || collision.stiffness != ref.at("stiffness") ||
          collision.damping != ref.at("damping") || collision.friction != ref.at("friction")) {
        throw std::runtime_error("Different geometry, body or material in Go2 collision");
      }
      compare(collision, ref.at("point").get<std::array<double, 3>>(),
              ref.at("normal").get<std::array<double, 3>>(), ref.at("depth"), errors);
      const auto& framed = result.framed[index];
      const auto velocity = ref.at("relative_velocity").get<std::array<double, 3>>();
      const double e = std::abs(framed.relative_velocity[2] - velocity[2]);
      if (!(e <= 2e-4)) throw std::runtime_error("Different Go2 normal velocity after collision");
      errors.velocity = std::max(errors.velocity, e);
      expected.erase(collision.geometry_b);
    }
  }
  std::cout << "Compared " << errors.witnesses << " Go2 collision witnesses over " << events.size()
            << " events. Maximum errors in point, depth, normal and normal velocity: "
            << errors.point << ", " << errors.depth << ", " << errors.normal << ", " << errors.velocity << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  int devices = 0;
  if (cudaGetDeviceCount(&devices) != cudaSuccess || devices == 0) return 77;
  try {
    if (argc < 2 || argc > 3) throw std::runtime_error("Supply the Go2 scene and optionally a trace directory");
    synthetic();
    sire::simulator::Simulator simulator;
    aris::core::fromXmlFile(simulator, argv[1]);
    simulator.init();
    const auto model = loadFlatGroundModel(simulator.physicsEngine());
    if (model.colliders.size() != 25 || model.body_count != 18) {
      throw std::runtime_error("Unexpected Go2 collision model dimensions");
    }
    if (argc == 3) replay(model, argv[2]);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
