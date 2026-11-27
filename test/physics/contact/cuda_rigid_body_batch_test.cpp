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

void checkDomainParameters(sire::physics::PhysicsEngine& engine,
                           const std::vector<RigidJointControl>& controls, cudaStream_t stream) {
  constexpr std::size_t size = 7;
  RigidBodyBatch batch(engine, size, physics_dt, control_dt, controls, true, true);
  DeviceBuffer<double> friction(size), mass(size), base_velocities(6 * size), push(2 * size);
  DeviceBuffer<bool> mask(size), repeat_mask(size), restore_mask(size);
  const std::array<bool, size> selected{true, false, true, true, true, true, true};
  const std::array<bool, size> repeat{false, false, false, false, true, false, false};
  const std::array<bool, size> restore{false, false, false, false, false, false, true};
  checkCuda(cudaMemcpyAsync(mask.data(), selected.data(), sizeof(selected), cudaMemcpyHostToDevice, stream));
  checkCuda(cudaMemcpyAsync(repeat_mask.data(), repeat.data(), sizeof(repeat), cudaMemcpyHostToDevice, stream));
  checkCuda(cudaMemcpyAsync(restore_mask.data(), restore.data(), sizeof(restore), cudaMemcpyHostToDevice, stream));
  friction.upload({0.4, 1.0, 0.02, 1.0, 0.4, 0.4, 0.4});
  mass.upload({0.0, 3.0, 0.0, 0.0, 3.0, 3.0, 3.0});
  std::vector<double> velocity(6 * size, 0.0);
  for (std::size_t env = 0; env < size; ++env) velocity[6 * env] = 1.0;
  base_velocities.upload(velocity);
  batch.setDomainParameters(friction.data(), mass.data(), mask.data(), stream);
  batch.reset(nullptr, nullptr, base_velocities.data(), nullptr, nullptr, stream);
  batch.setDomainParameters(nullptr, mass.data(), repeat_mask.data(), stream);

  std::array<double, size> tangential{};
  double mass_response = 0.0;
  const auto velocity_stride = 6 * batch.bodyCount();
  for (std::size_t tick = 1; tick <= ticks; ++tick) {
    batch.step(nullptr, stream);
    const auto state = readBatch(batch, stream);
    checkState(state);
    for (std::size_t env = 0; env < size; ++env) {
      checkClock(state, env, tick * control_dt);
      for (std::size_t body = 0; body < batch.bodyCount(); ++body) {
        const auto* wrench = state.contact_wrenches.data() + env * velocity_stride + 6 * body;
        tangential[env] += std::hypot(wrench[0], wrench[1]);
      }
    }
    for (std::size_t component = 0; component < velocity_stride; ++component) {
      const auto& values = state.velocities;
      if (std::abs(values[component] - values[velocity_stride + component]) > 1e-10) {
        throw std::runtime_error("Masked domain update affected an unselected Go2 environment");
      }
      const auto changed = values[4 * velocity_stride + component];
      if (std::abs(changed - values[5 * velocity_stride + component]) > 1e-10) {
        throw std::runtime_error("Repeated base mass update compounded the previous offset");
      }
      mass_response = std::max(mass_response, std::abs(changed - values[component]));
    }
  }
  if (!(tangential[3] > tangential[2] + 1e-3)) {
    throw std::runtime_error("Higher randomized friction did not increase sliding contact force");
  }
  if (!(mass_response > 1e-4)) {
    throw std::runtime_error("Randomized base mass did not change the Go2 trajectory");
  }

  mass.upload(std::vector<double>(size, 0.0));
  batch.setDomainParameters(nullptr, mass.data(), restore_mask.data(), stream);
  batch.reset(nullptr, nullptr, base_velocities.data(), nullptr, nullptr, stream);
  for (std::size_t tick = 1; tick <= ticks; ++tick) {
    batch.step(nullptr, stream);
    const auto state = readBatch(batch, stream);
    checkClock(state, 6, tick * control_dt);
    for (std::size_t component = 0; component < velocity_stride; ++component) {
      if (std::abs(state.velocities[component] - state.velocities[6 * velocity_stride + component]) > 1e-10) {
        throw std::runtime_error("A zero offset did not restore the nominal Go2 base mass");
      }
    }
  }

  std::vector<double> xy(2 * size);
  for (std::size_t env = 0; env < size; ++env) {
    std::copy_n(std::array{0.0, 0.0, 0.3, 0.1, -0.2, 0.3}.data(), 6, velocity.data() + 6 * env);
    xy[2 * env] = 0.25 * env;
    xy[2 * env + 1] = -0.125 * env;
  }
  base_velocities.upload(velocity);
  push.upload(xy);
  batch.reset(nullptr, nullptr, base_velocities.data(), nullptr, nullptr, stream);
  const auto before = readBatch(batch, stream);
  batch.pushBaseVelocity(push.data(), stream);
  const auto after = readBatch(batch, stream);
  checkState(after);
  for (std::size_t env = 0; env < size; ++env) {
    checkClock(after, env, 0.0);
    const auto root_body = env * batch.bodyCount() + batch.rootBody();
    const auto root = 6 * root_body;
    const auto* pose = before.poses.data() + 12 * root_body;
    const auto* angular = before.velocities.data() + root + 3;
    const std::array expected{
        xy[2 * env] - (angular[1] * pose[11] - angular[2] * pose[7]),
        xy[2 * env + 1] - (angular[2] * pose[3] - angular[0] * pose[11]),
        -(angular[0] * pose[7] - angular[1] * pose[3]), angular[0], angular[1], angular[2]};
    for (std::size_t component = 0; component < expected.size(); ++component) {
      if (std::abs(after.velocities[root + component] - expected[component]) > 1e-12) {
        throw std::runtime_error("Go2 push did not set its base twist as requested");
      }
      const auto delta = expected[component] - before.velocities[root + component];
      for (std::size_t body = batch.rootBody() + 1; body < batch.bodyCount(); ++body) {
        const auto index = env * velocity_stride + 6 * body + component;
        if (std::abs(after.velocities[index] - before.velocities[index] - delta) > 1e-12) {
          throw std::runtime_error("Go2 push did not propagate through its articulated tree");
        }
      }
    }
  }
  std::cout << "Go2 domain parameters affected contact and dynamics, respected masks, and preserved coherent pushes.\n";
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
  checkDomainParameters(cpu.physicsEngine(), controls, stream.value);
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
  if (later.pose > 1e-7 || later.velocity > 1e-6) {
    throw std::runtime_error("Go2 CUDA trajectory differs from CPU after ground contact");
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
