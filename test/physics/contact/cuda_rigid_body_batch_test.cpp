#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <aris/core/serialization.hpp>
#include <aris/dynamic/model.hpp>

#include "cuda_test_buffer.hpp"
#include "rigid_body_batch.hpp"
#include "sire/actuator/actuator.hpp"
#include "sire/simulator/simulator.hpp"

namespace {

using namespace sire::physics::contact::cuda;
using sire::test::DeviceBuffer;
using sire::test::checkCuda;

constexpr std::size_t batch_size = 1024;
constexpr std::size_t ticks = 14;
constexpr std::size_t early_ticks = 5;
constexpr double physics_dt = 0.001;
constexpr double control_dt = 0.02;
constexpr double time_tolerance = 2e-12;

class Stream {
 public:
  Stream() { checkCuda(cudaStreamCreateWithFlags(&value, cudaStreamNonBlocking)); }
  ~Stream() { cudaStreamDestroy(value); }
  Stream(const Stream&) = delete;
  Stream& operator=(const Stream&) = delete;

  cudaStream_t value{};
};

template <class T>
auto download(const T* source, std::size_t size) -> std::vector<T> {
  std::vector<T> output(size);
  checkCuda(cudaMemcpy(output.data(), source, size * sizeof(T), cudaMemcpyDeviceToHost));
  return output;
}

struct Snapshot {
  std::vector<double> poses, velocities, positions, rates, torques, contact_wrenches;
  std::unique_ptr<bool[]> ground_contacts;
  std::vector<RigidDynamicsStatus> statuses;
  std::vector<RigidEventClock> clocks;
};

auto readBatch(const RigidBodyBatch& batch, cudaStream_t stream) -> Snapshot {
  checkCuda(cudaStreamSynchronize(stream));
  const auto body_slots = batch.batchSize() * batch.bodyCount();
  const auto joint_slots = batch.batchSize() * batch.jointCount();
  const auto state = batch.state();
  auto contacts = std::make_unique<bool[]>(body_slots);
  checkCuda(cudaMemcpy(contacts.get(), state.ground_contacts, body_slots * sizeof(bool),
                       cudaMemcpyDeviceToHost));
  return {download(state.poses, 12 * body_slots),
          download(state.velocities, 6 * body_slots),
          download(state.positions, joint_slots),
          download(state.rates, joint_slots),
          download(state.motor_torques, joint_slots),
          download(state.contact_wrenches, 6 * body_slots),
          std::move(contacts),
          download(state.status, batch.batchSize()),
          download(state.clocks, batch.batchSize())};
}

void checkFinite(const std::vector<double>& values, const char* label) {
  const auto invalid = std::find_if(values.begin(), values.end(),
                                    [](double value) { return !std::isfinite(value); });
  if (invalid != values.end()) {
    throw std::runtime_error(std::string("Non-finite ") + label + " at index " +
                             std::to_string(invalid - values.begin()));
  }
}

void checkState(const Snapshot& state) {
  checkFinite(state.poses, "pose");
  checkFinite(state.velocities, "velocity");
  checkFinite(state.positions, "joint position");
  checkFinite(state.rates, "joint rate");
  checkFinite(state.torques, "motor torque");
  checkFinite(state.contact_wrenches, "contact wrench");
  if (std::any_of(state.torques.begin(), state.torques.end(), [](double value) { return value != 0.0; })) {
    throw std::runtime_error("Go2 test expected zero motor torque");
  }
}

void checkClock(const Snapshot& state, std::size_t env, double expected_time) {
  const auto& clock = state.clocks[env];
  if (state.statuses[env] != RigidDynamicsStatus::Success ||
      !(std::abs(clock.time - expected_time) <= time_tolerance) ||
      !(std::abs(clock.previous_control_time - expected_time) <= time_tolerance) ||
      (expected_time > 0.0 && (!clock.done || clock.event_count == 0))) {
    throw std::runtime_error("Go2 environment " + std::to_string(env) + " expected control time " +
                             std::to_string(expected_time) + ", observed " + std::to_string(clock.time) +
                             ", status " + std::to_string(static_cast<int>(state.statuses[env])));
  }
}

auto advanceCpu(sire::simulator::Simulator& cpu) -> std::size_t {
  auto& loop = cpu.simulationLoop();
  std::size_t events = 0;
  // Match the CPU policy runner's current-event handling and zero action.
  do {
    if (++events > 100000) throw std::runtime_error("CPU Go2 exceeded its event limit");
    for (auto& motion : cpu.model().motionPool()) {
      dynamic_cast<sire::actuator::ActuatorSISO&>(motion).setDesiredValue(0.0);
    }
    loop.handleContact();
  } while (!loop.headerIsCtrl());
  cpu.physicsEngine().enforceJointLimitSafety();
  return events;
}

struct TrajectoryError {
  double pose{0.0};
  double velocity{0.0};
  double root_translation{0.0};
};

void compareCpu(const RigidBodyBatch& batch, const Snapshot& state, aris::dynamic::Model& cpu,
                std::size_t tick, TrajectoryError& maximum) {
  // Compare representatives at both ends of the batch and at the reset boundary.
  constexpr std::array<std::size_t, 3> representatives{0, 511, batch_size - 1};
  TrajectoryError current;
  for (const auto env : representatives) {
    for (const auto& part : cpu.partPool()) {
      const auto body = env * batch.bodyCount() + part.id();
      const auto* cpu_pose = *part.pm();
      for (std::size_t i = 0; i < 12; ++i) {
        const double error = std::abs(state.poses[12 * body + i] - cpu_pose[i]);
        current.pose = std::max(current.pose, error);
        if (part.id() == batch.rootBody() && i % 4 == 3) {
          current.root_translation = std::max(current.root_translation, error);
        }
      }
      for (std::size_t i = 0; i < 6; ++i) {
        current.velocity = std::max(current.velocity,
                                    std::abs(state.velocities[6 * body + i] - part.vs()[i]));
      }
    }
  }
  if (tick <= early_ticks && (current.pose > 2e-8 || current.velocity > 2e-8)) {
    throw std::runtime_error("Go2 trajectory diverged before contact at tick " + std::to_string(tick) +
                             ", pose " + std::to_string(current.pose) +
                             ", velocity " + std::to_string(current.velocity));
  }
  maximum.pose = std::max(maximum.pose, current.pose);
  maximum.velocity = std::max(maximum.velocity, current.velocity);
  maximum.root_translation = std::max(maximum.root_translation, current.root_translation);
}

void compareSlice(const double* actual, const double* expected, std::size_t env,
                  std::size_t stride, const char* label) {
  const auto start = env * stride;
  if (!std::equal(actual + start, actual + start + stride, expected + start)) {
    throw std::runtime_error(std::string("Go2 masked reset changed ") + label + " in environment " +
                             std::to_string(env));
  }
}

void checkReset(const RigidBodyBatch& batch, const Snapshot& actual, const Snapshot& expected,
                std::size_t env) {
  const auto bodies = batch.bodyCount();
  const auto joints = batch.jointCount();
  compareSlice(actual.poses.data(), expected.poses.data(), env, 12 * bodies, "poses");
  compareSlice(actual.velocities.data(), expected.velocities.data(), env, 6 * bodies, "velocities");
  compareSlice(actual.positions.data(), expected.positions.data(), env, joints, "joint positions");
  compareSlice(actual.rates.data(), expected.rates.data(), env, joints, "joint rates");
  compareSlice(actual.torques.data(), expected.torques.data(), env, joints, "motor torques");
  compareSlice(actual.contact_wrenches.data(), expected.contact_wrenches.data(), env, 6 * bodies,
               "contact wrenches");
  const auto contact_offset = env * bodies;
  if (!std::equal(actual.ground_contacts.get() + contact_offset,
                  actual.ground_contacts.get() + contact_offset + bodies,
                  expected.ground_contacts.get() + contact_offset)) {
    throw std::runtime_error("Go2 masked reset changed ground-contact flags in environment " +
                             std::to_string(env));
  }
  const auto& clock = actual.clocks[env];
  const auto& expected_clock = expected.clocks[env];
  if (clock.time != expected_clock.time || clock.previous_control_time != expected_clock.previous_control_time ||
      clock.event_count != expected_clock.event_count || clock.done != expected_clock.done ||
      actual.statuses[env] != expected.statuses[env]) {
    throw std::runtime_error("Go2 masked reset changed clock or status unexpectedly in environment " +
                             std::to_string(env));
  }
}

void run(const char* scene) {
  sire::simulator::Simulator cpu;
  aris::core::fromXmlFile(cpu, scene);
  cpu.init();
  cpu.simulationLoop().setDeltaT(physics_dt);
  cpu.simulationLoop().setCtrlT(control_dt);
  std::vector<RigidJointControl> controls;
  for (std::size_t joint = 0; joint < cpu.model().motionPool().size(); ++joint) {
    controls.push_back({joint, 0.0, 0.0, 0.0, 1.0});
  }
  Stream stream;
  RigidBodyBatch batch(cpu.physicsEngine(), batch_size, physics_dt, control_dt, controls);
  if (batch.batchSize() != batch_size || batch.bodyCount() != 18 || batch.jointCount() != 12 ||
      batch.rootBody() != 1) {
    throw std::runtime_error("Unexpected Go2 model dimensions");
  }
  batch.reset(nullptr, nullptr, nullptr, nullptr, nullptr, stream.value);
  const auto initial = readBatch(batch, stream.value);
  checkState(initial);
  TrajectoryError early, later;
  compareCpu(batch, initial, cpu.model(), 0, early);
  for (std::size_t env = 0; env < batch_size; ++env) checkClock(initial, env, 0.0);

  std::array<bool, batch_size> saw_contact{};
  std::size_t cpu_events = 0, max_gpu_events = 0;
  double peak_contact_force = 0.0;
  Snapshot first, last;
  for (std::size_t tick = 1; tick <= ticks; ++tick) {
    batch.step(nullptr, stream.value);
    cpu_events += advanceCpu(cpu);
    const double target = tick * control_dt;
    if (!(std::abs(cpu.simulationLoop().simTime() - target) <= time_tolerance)) {
      throw std::runtime_error("CPU Go2 missed control boundary at tick " + std::to_string(tick));
    }
    auto current = readBatch(batch, stream.value);
    checkState(current);
    compareCpu(batch, current, cpu.model(), tick, tick <= early_ticks ? early : later);
    for (std::size_t env = 0; env < batch_size; ++env) {
      checkClock(current, env, target);
      max_gpu_events = std::max(max_gpu_events, current.clocks[env].event_count);
      for (std::size_t body = 0; body < batch.bodyCount(); ++body) {
        const auto index = env * batch.bodyCount() + body;
        if (!current.ground_contacts[index]) continue;
        const auto* force = current.contact_wrenches.data() + 6 * index;
        const double norm = std::hypot(force[0], force[1], force[2]);
        peak_contact_force = std::max(peak_contact_force, norm);
        if (norm > 1e-8) saw_contact[env] = true;
      }
    }
    if (tick == 1) first = std::move(current);
    if (tick == ticks) last = std::move(current);
  }
  if (!std::all_of(saw_contact.begin(), saw_contact.end(), [](bool value) { return value; })) {
    throw std::runtime_error("Some Go2 environments never produced a ground-contact force");
  }

  std::array<bool, batch_size> mask{};
  for (const auto env : {0, 17, 511, 1023}) mask[env] = true;
  DeviceBuffer<bool> device_mask(batch_size);
  checkCuda(cudaMemcpy(device_mask.data(), mask.data(), mask.size() * sizeof(bool),
                       cudaMemcpyHostToDevice));
  batch.reset(device_mask.data(), nullptr, nullptr, nullptr, nullptr, stream.value);
  const auto reset = readBatch(batch, stream.value);
  for (std::size_t env = 0; env < batch_size; ++env) {
    checkReset(batch, reset, mask[env] ? initial : last, env);
  }

  batch.step(nullptr, stream.value);
  const auto advanced = readBatch(batch, stream.value);
  checkState(advanced);
  for (std::size_t env = 0; env < batch_size; ++env) {
    checkClock(advanced, env, (mask[env] ? 1 : ticks + 1) * control_dt);
    if (mask[env]) checkReset(batch, advanced, first, env);
  }

  std::cout << batch_size << " Go2 environments advanced through " << ticks
            << " control ticks with ground contact. Peak sampled contact force " << peak_contact_force
            << " N. CPU events " << cpu_events << ", maximum GPU events per tick " << max_gpu_events
            << ".\nEarly maximum pose and velocity component errors " << early.pose << ", " << early.velocity
            << ". Later maximum pose, velocity, and base translation component differences "
            << later.pose << ", " << later.velocity << ", " << later.root_translation
            << ".\nMasked reset restored four environments while preserving the state and clocks of their peers.\n";
}

}  // namespace

int main(int argc, char** argv) {
  int devices = 0;
  if (cudaGetDeviceCount(&devices) != cudaSuccess || devices == 0) return 77;
  try {
    if (argc > 2) throw std::runtime_error("Supply at most one Go2 scene filename");
    run(argc == 2 ? argv[1] : "go2_rai_foot.xml");
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
