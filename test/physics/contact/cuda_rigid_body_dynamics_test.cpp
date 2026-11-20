#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <aris/core/serialization.hpp>
#include <aris/dynamic/model.hpp>
#include <aris/ext/json.hpp>

#include "cuda_test_buffer.hpp"
#include "rigid_body_integration.hpp"
#include "sire/actuator/actuator.hpp"
#include "sire/integrator/semi_implicit_euler_integrator.hpp"
#include "sire/simulator/simulator.hpp"

namespace {

using namespace sire::physics::contact::cuda;
using sire::test::DeviceBuffer;
using sire::test::checkCuda;
using Json = nlohmann::json;

class Stream {
 public:
  Stream() { checkCuda(cudaStreamCreateWithFlags(&value, cudaStreamNonBlocking)); }
  ~Stream() { cudaStreamDestroy(value); }
  Stream(const Stream&) = delete;
  Stream& operator=(const Stream&) = delete;

  cudaStream_t value{};
};

struct DynamicsBatch {
  DynamicsBatch(const RigidBodyModel& model, std::size_t size)
      : batch(size),
        body_count(model.bodies.size()),
        joint_count(model.joint_count),
        dof(6 + joint_count),
        bodies(body_count),
        poses(batch * body_count * 12),
        velocities(batch * body_count * 6),
        accelerations(batch * body_count * 6),
        jacobians(batch * body_count * 6 * dof),
        mass_factor(batch * dof * dof),
        scratch(batch * body_count * (36 + 6 * dof + 6)),
        status(batch),
        tree{bodies.data(), body_count, joint_count, model.ground, {}} {
    std::copy_n(model.gravity, 6, tree.gravity);
    bodies.upload(model.bodies);
  }

  void solve(const double* torques, const double* wrenches = nullptr) {
    checkCuda(launchRigidBodyDynamics(batch, tree, {poses.data(), velocities.data(), torques, wrenches},
                                     {accelerations.data(), jacobians.data(), mass_factor.data(), status.data()},
                                     scratch.data(), stream.value));
  }

  void checkStatus() {
    checkCuda(cudaStreamSynchronize(stream.value));
    const auto statuses = status.download();
    for (std::size_t i = 0; i < batch; ++i) {
      if (statuses[i] != RigidDynamicsStatus::Success) {
        throw std::runtime_error("Dynamics failed in case " + std::to_string(i) + " with status " +
                                 std::to_string(static_cast<int>(statuses[i])));
      }
    }
  }

  const std::size_t batch, body_count, joint_count, dof;
  Stream stream;
  DeviceBuffer<RigidBody> bodies;
  DeviceBuffer<double> poses, velocities, accelerations, jacobians, mass_factor, scratch;
  DeviceBuffer<RigidDynamicsStatus> status;
  RigidBodyTree tree;
};

struct Error {
  double maximum{0.0};

  void compare(double actual, double expected, const char* label, std::size_t sample,
               std::size_t element, double absolute = 2e-8, double relative = 2e-8) {
    const double error = std::abs(actual - expected);
    if (!(error <= absolute + relative * std::abs(expected))) {
      std::ostringstream message;
      message << std::setprecision(17) << label << " in case " << sample << ", element " << element
              << ": GPU " << actual << ", CPU " << expected;
      throw std::runtime_error(message.str());
    }
    maximum = std::max(maximum, error);
  }
};

void expectAccelerations(DynamicsBatch& batch, const DeviceBuffer<double>& actual,
                         const std::vector<double>& expected, const char* label) {
  const auto observed = actual.download();
  const auto stride = 6 * batch.body_count;
  Error errors;
  for (std::size_t i = 0; i < observed.size(); ++i) {
    errors.compare(observed[i], expected[i], label, i / stride, i % stride);
  }
  std::cout << label << " across " << batch.batch << " cases, maximum error "
            << errors.maximum << '\n';
}

void appendAccelerations(aris::dynamic::Model& model, std::vector<double>& output) {
  if (model.forwardDynamics() != 0) throw std::runtime_error("ARIS forwardDynamics failed");
  for (const auto& part : model.partPool()) {
    output.insert(output.end(), part.as(), part.as() + 6);
  }
}

void appendState(aris::dynamic::Model& model, std::vector<double>& poses,
                 std::vector<double>& velocities) {
  for (const auto& part : model.partPool()) {
    poses.insert(poses.end(), *part.pm(), *part.pm() + 12);
    velocities.insert(velocities.end(), part.vs(), part.vs() + 6);
  }
}

void appendIntegratedState(aris::dynamic::Model& cpu,
                           sire::simulator::SemiImplicitEulerIntegrator& integrator,
                           double step, std::vector<double>& poses, std::vector<double>& velocities) {
  if (step > 0.0 && !integrator.doStep(step)) {
    throw std::runtime_error("Sire semi-implicit Euler integration failed");
  }
  appendState(cpu, poses, velocities);
}

void expectIntegratedState(DynamicsBatch& batch, const RigidBodyModel& model,
                           const std::vector<double>& steps,
                           const std::vector<double>& expected_poses,
                           const std::vector<double>& expected_velocities) {
  DeviceBuffer<std::size_t> order(model.order.size());
  DeviceBuffer<double> device_steps(steps.size()), corrections(batch.batch * batch.body_count * 6);
  order.upload(model.order);
  device_steps.upload(steps);
  checkCuda(launchIntegrateRigidBodyTree(batch.batch, batch.tree, order.data(), device_steps.data(),
                                         batch.accelerations.data(), batch.poses.data(),
                                         batch.velocities.data(), batch.status.data(),
                                         corrections.data(), batch.stream.value));
  batch.checkStatus();
  const auto poses = batch.poses.download();
  const auto velocities = batch.velocities.download();
  Error pose_error, velocity_error;
  for (std::size_t env = 0; env < batch.batch; ++env) {
    const auto pose_stride = 12 * batch.body_count;
    const auto velocity_stride = 6 * batch.body_count;
    const double tolerance = steps[env] == 0.0 ? 0.0 : 2e-8;
    for (std::size_t i = 0; i < pose_stride; ++i) {
      const auto index = env * pose_stride + i;
      pose_error.compare(poses[index], expected_poses[index], "Go2 integrated pose", env, i,
                         tolerance, tolerance);
    }
    for (std::size_t i = 0; i < velocity_stride; ++i) {
      const auto index = env * velocity_stride + i;
      velocity_error.compare(velocities[index], expected_velocities[index],
                             "Go2 integrated velocity", env, i, tolerance, tolerance);
    }
  }
  std::cout << "Go2 semi-implicit Euler integration across " << batch.batch
            << " cases, maximum pose and velocity errors " << pose_error.maximum << ", "
            << velocity_error.maximum << '\n';
}

auto activateOnlyMotors(aris::dynamic::Model& cpu)
    -> std::vector<sire::actuator::ActuatorSISO*> {
  for (auto& force : cpu.forcePool()) force.activate(false);
  std::vector<sire::actuator::ActuatorSISO*> motors;
  for (auto& motion : cpu.motionPool()) {
    auto* motor = dynamic_cast<sire::actuator::ActuatorSISO*>(&motion);
    if (!motor || !motor->fcePtr()) throw std::runtime_error("Unexpected Go2 actuator");
    motor->fcePtr()->activate(true);
    motors.push_back(motor);
  }
  return motors;
}

auto bodyWrenchForces(aris::dynamic::Model& cpu) -> std::vector<aris::dynamic::GeneralForce*> {
  std::vector<aris::dynamic::GeneralForce*> result(cpu.partPool().size());
  for (auto& element : cpu.forcePool()) {
    auto* force = dynamic_cast<aris::dynamic::GeneralForce*>(&element);
    if (!force) continue;
    const auto body = force->makI()->fatherPart().id();
    if (force->name() == "cf_" + std::to_string(body)) result.at(body) = force;
  }
  if (std::any_of(result.begin(), result.end(), [](auto* force) { return force == nullptr; })) {
    throw std::runtime_error("Missing Go2 body wrench inputs");
  }
  return result;
}

void setInitialState(aris::dynamic::Model& cpu, const RigidBodyModel& model,
                     const std::vector<std::array<double, 16>>& initial, std::size_t env) {
  const double phase = 0.073 * static_cast<double>(env + 1);
  const double offset[6] = {0.07 * std::sin(phase), 0.09 * std::cos(phase), 0.014,
                            0.16 * std::sin(phase), 0.10 * std::cos(phase), 0.03};
  const double root_velocity[6] = {0.13, -0.08 * std::sin(phase), 0.05 * std::cos(phase),
                                   0.60 * std::sin(phase), -0.45, 0.22 * std::cos(phase)};
  double transform[16];
  aris::dynamic::s_ps2pm(offset, transform);
  for (const auto body : model.order) {
    double pose[16];
    aris::dynamic::s_pm_dot_pm(transform, initial[body].data(), pose);
    cpu.partPool()[body].setPm(pose);
    if (body == model.root) {
      cpu.partPool()[body].setVs(root_velocity);
      continue;
    }
    const auto& link = model.bodies[body];
    const auto* parent_velocity = cpu.partPool()[link.parent].vs();
    if (link.joint < 0) {
      cpu.partPool()[body].setVs(parent_velocity);
      continue;
    }
    const double speed = 0.3 * std::cos(0.37 * phase * (link.joint + 1));
    const double local[6] = {0.0, 0.0, 0.0, 0.0, 0.0, speed};
    double velocity[6];
    aris::dynamic::s_tv(*cpu.motionPool()[link.joint].makI()->pm(), local, velocity);
    for (std::size_t i = 0; i < 6; ++i) velocity[i] += parent_velocity[i];
    cpu.partPool()[body].setVs(velocity);
  }
}

void testTorques(sire::simulator::Simulator& simulator, const RigidBodyModel& model) {
  constexpr std::size_t count = 1024;
  auto& cpu = simulator.model();
  sire::simulator::SemiImplicitEulerIntegrator integrator;
  integrator.init(&simulator.physicsEngine());
  const auto motors = activateOnlyMotors(cpu);
  const auto body_count = model.bodies.size();
  const auto stride = 6 * body_count;
  DynamicsBatch batch(model, count);
  std::vector<double> poses, velocities, integrated_poses, integrated_velocities;
  std::vector<double> torques(count * model.joint_count), wrenches(count * stride);
  std::vector<double> expected, expected_loaded, steps(count);
  expected.reserve(count * stride);
  expected_loaded.reserve(count * stride);

  // GeneralForce accepts a world wrench about the world origin.
  const auto body_forces = bodyWrenchForces(cpu);
  const std::array loaded_forces{body_forces[model.root], body_forces[model.order.back()]};
  std::vector<std::array<double, 16>> initial(body_count);
  for (const auto& part : cpu.partPool()) part.getPm(initial[part.id()].data());
  for (std::size_t env = 0; env < count; ++env) {
    setInitialState(cpu, model, initial, env);
    appendState(cpu, poses, velocities);
    for (std::size_t joint = 0; joint < motors.size(); ++joint) {
      auto& motor = *motors[joint];
      const double phase = 0.071 * (env + 1) * (joint + 1);
      motor.setDesiredValue(1.5 * std::sin(phase) + 0.4 * std::cos(0.31 * phase + joint));
      motor.forward();
      torques[env * model.joint_count + joint] = motor.fcePtr()->fce();
    }
    appendAccelerations(cpu, expected);
    for (auto* force : loaded_forces) {
      const auto body = force->makI()->fatherPart().id();
      auto* wrench = wrenches.data() + (env * body_count + body) * 6;
      for (std::size_t k = 0; k < 6; ++k) {
        wrench[k] = 0.8 * std::cos(0.13 * (env + 1) * (k + 1) + body);
      }
      force->setFce(wrench);
      force->activate(true);
    }
    appendAccelerations(cpu, expected_loaded);
    for (auto* force : loaded_forces) force->activate(false);
    steps[env] = env % 17 == 0 ? 0.0 : 0.00015 * (1 + env % 17);
    appendIntegratedState(cpu, integrator, steps[env], integrated_poses, integrated_velocities);
  }

  DeviceBuffer<double> device_torque(torques.size()), device_wrenches(wrenches.size());
  DeviceBuffer<double> loaded(wrenches.size()), generalized(count * batch.dof);
  batch.poses.upload(poses);
  batch.velocities.upload(velocities);
  device_torque.upload(torques);
  device_wrenches.upload(wrenches);
  batch.solve(device_torque.data());
  checkCuda(launchApplyRigidBodyWrenches(count, body_count, model.joint_count, device_wrenches.data(),
                                        batch.accelerations.data(), batch.jacobians.data(),
                                        batch.mass_factor.data(), batch.status.data(), loaded.data(),
                                        generalized.data(), batch.stream.value));
  batch.checkStatus();
  expectAccelerations(batch, batch.accelerations, expected, "Go2 torque-driven acceleration");
  expectAccelerations(batch, loaded, expected_loaded, "Go2 acceleration with applied world wrenches");
  expectIntegratedState(batch, model, steps, integrated_poses, integrated_velocities);
}

struct Event {
  std::vector<DetectedContact> detections;
  std::vector<std::size_t> active;
  std::vector<double> expected_inertia, expected_acceleration;
};

auto readSolverFrame(const std::filesystem::path& directory, std::size_t frame_id) -> Json {
  std::ostringstream filename;
  filename << "frame_" << std::setw(8) << std::setfill('0') << frame_id << ".json";
  std::ifstream file(directory / filename.str());
  if (!file) throw std::runtime_error("Cannot read solver frame " + filename.str());
  Json frame;
  file >> frame;
  return frame;
}

void replay(sire::simulator::Simulator& simulator, const RigidBodyModel& model,
            const std::filesystem::path& directory) {
  auto& cpu = simulator.model();
  auto& engine = simulator.physicsEngine();
  sire::simulator::SemiImplicitEulerIntegrator integrator;
  integrator.init(&engine);
  const auto motors = activateOnlyMotors(cpu);
  const auto body_forces = bodyWrenchForces(cpu);
  std::ifstream file(directory / "single_point_contact_history.jsonl");
  if (!file) throw std::runtime_error("Cannot read Go2 contact trace in " + directory.string());
  const auto body_count = model.bodies.size();
  std::size_t capacity = 0, active_contacts = 0;
  std::vector<double> poses, velocities, torques, wrenches, expected;
  std::vector<double> steps, integrated_poses, integrated_velocities;
  std::vector<Event> events;
  for (std::string line; std::getline(file, line);) {
    const auto row = Json::parse(line);
    const auto& body_poses = row.at("body_poses");
    const auto& body_velocities = row.at("body_velocities");
    if (body_poses.size() != body_count || body_velocities.size() != body_count) {
      throw std::runtime_error("Go2 trace and model body counts differ");
    }
    for (std::size_t body = 0; body < body_count; ++body) {
      const auto pose = body_poses.at(body).get<std::array<double, 16>>();
      const auto velocity = body_velocities.at(body).get<std::array<double, 6>>();
      poses.insert(poses.end(), pose.begin(), pose.begin() + 12);
      velocities.insert(velocities.end(), velocity.begin(), velocity.end());
      cpu.partPool()[body].setPm(pose.data());
      cpu.partPool()[body].setVs(velocity.data());
    }
    const auto* applied_wrenches =
        row.contains("body_applied_wrenches") ? &row.at("body_applied_wrenches") : nullptr;
    if (applied_wrenches && applied_wrenches->size() != body_count) {
      throw std::runtime_error("Wrong Go2 trace wrench dimensions");
    }
    for (auto* motor : motors) {
      motor->updP();
      motor->updV();
      motor->setDesiredValue(0.0);
      motor->forward();
    }
    if (!applied_wrenches) {
      // Older traces recorded the state and horizon but omitted the reaction
      // forces. Recompute them for the zero-action trace.
      engine.applyJointLimitForces(row.at("horizon").get<double>());
    }
    for (auto* motor : motors) torques.push_back(motor->fcePtr()->fce());
    for (std::size_t body = 0; body < body_count; ++body) {
      const auto wrench = applied_wrenches
                              ? applied_wrenches->at(body).get<std::array<double, 6>>()
                              : std::array<double, 6>{};
      wrenches.insert(wrenches.end(), wrench.begin(), wrench.end());
      body_forces[body]->setFce(wrench.data());
      body_forces[body]->activate(applied_wrenches != nullptr);
    }
    appendAccelerations(cpu, expected);

    auto& event = events.emplace_back();
    for (const auto& raw : row.at("detected")) {
      auto& detection = event.detections.emplace_back();
      detection.body_a = raw.at("body_a").get<std::size_t>();
      detection.body_b = raw.at("body_b").get<std::size_t>();
      if (detection.body_a >= body_count || detection.body_b >= body_count) {
        throw std::runtime_error("Wrong Go2 contact body index");
      }
      const auto pose = raw.at("frame").get<std::array<double, 16>>();
      std::copy_n(pose.begin(), 12, detection.frame);
    }
    event.active = row.at("active").get<std::vector<std::size_t>>();
    for (const auto index : event.active) {
      if (index >= event.detections.size()) throw std::runtime_error("Wrong Go2 active contact index");
    }
    double step = row.at("horizon").get<double>();
    if (!event.active.empty()) {
      const auto frame = readSolverFrame(directory, row.at("solver_frame").get<std::size_t>());
      step = frame.at("h").get<double>();
      const auto& reference = frame.at("body_contact_inputs");
      event.expected_inertia = reference.at("inverse_inertia").get<std::vector<double>>();
      event.expected_acceleration = reference.at("free_acceleration").get<std::vector<double>>();
      const auto size = 6 * event.active.size();
      if (event.expected_inertia.size() != size * size || event.expected_acceleration.size() != size) {
        throw std::runtime_error("Wrong Go2 contact solver frame dimensions");
      }
    }
    steps.push_back(step);
    appendIntegratedState(cpu, integrator, step, integrated_poses, integrated_velocities);
    engine.activateContactForce(false);
    capacity = std::max(capacity, event.detections.size());
    active_contacts += event.active.size();
  }
  if (active_contacts == 0) throw std::runtime_error("Go2 trace lacks active contacts");

  DynamicsBatch batch(model, events.size());
  const auto stride = 6 * capacity;
  const auto slots = batch.batch * capacity;
  std::vector<DetectedContact> detections(slots);
  std::vector<std::size_t> selected(slots);
  std::vector<AdmmProblem> problems(batch.batch);
  for (std::size_t env = 0; env < batch.batch; ++env) {
    const auto& event = events[env];
    std::copy(event.detections.begin(), event.detections.end(), detections.begin() + env * capacity);
    std::copy(event.active.begin(), event.active.end(), selected.begin() + env * capacity);
    problems[env] = {event.active.size(), 1000, 0.001, 1e-8, false, false};
  }

  DeviceBuffer<DetectedContact> device_detections(slots);
  DeviceBuffer<std::size_t> device_selected(slots);
  DeviceBuffer<AdmmProblem> device_problems(batch.batch);
  DeviceBuffer<double> device_torques(torques.size()), device_wrenches(wrenches.size());
  DeviceBuffer<double> inverse_inertia(batch.batch * stride * stride),
      free_acceleration(batch.batch * stride), factor(batch.batch * stride * batch.dof);
  batch.poses.upload(poses);
  batch.velocities.upload(velocities);
  device_detections.upload(detections);
  device_selected.upload(selected);
  device_problems.upload(problems);
  device_torques.upload(torques);
  device_wrenches.upload(wrenches);
  batch.solve(device_torques.data(), device_wrenches.data());
  checkCuda(launchRigidBodyContact(batch.batch, body_count, model.joint_count, capacity,
                                   batch.velocities.data(), batch.accelerations.data(),
                                   batch.jacobians.data(), batch.mass_factor.data(), batch.status.data(),
                                   device_detections.data(), device_selected.data(), device_problems.data(),
                                   inverse_inertia.data(), free_acceleration.data(), factor.data(),
                                   batch.stream.value));
  batch.checkStatus();
  expectAccelerations(batch, batch.accelerations, expected, "Go2 trace spatial acceleration");

  const auto observed_problems = device_problems.download();
  const auto inertia = inverse_inertia.download();
  const auto acceleration = free_acceleration.download();
  Error inertia_error, acceleration_error;
  for (std::size_t env = 0; env < batch.batch; ++env) {
    const auto& event = events[env];
    if (observed_problems[env].contacts != event.active.size() ||
        observed_problems[env].step != problems[env].step) {
      throw std::runtime_error("Go2 contact assembly failed in event " + std::to_string(env));
    }
    const auto size = 6 * event.active.size();
    for (std::size_t i = 0; i < size; ++i) {
      acceleration_error.compare(acceleration[env * stride + i], event.expected_acceleration[i],
                                 "Go2 free point acceleration", env, i);
      for (std::size_t j = 0; j < size; ++j) {
        inertia_error.compare(inertia[(env * stride + i) * stride + j],
                              event.expected_inertia[i * size + j], "Go2 contact inverse inertia",
                              env, i * size + j);
      }
    }
  }
  std::cout << "Compared " << active_contacts << " Go2 contacts across " << batch.batch
            << " trace events. Maximum free acceleration error " << acceleration_error.maximum
            << ", maximum inverse inertia error " << inertia_error.maximum << '\n';
  expectIntegratedState(batch, model, steps, integrated_poses, integrated_velocities);
}

}  // namespace

int main(int argc, char** argv) {
  int devices = 0;
  if (cudaGetDeviceCount(&devices) != cudaSuccess || devices == 0) return 77;
  try {
    if (argc > 3) throw std::runtime_error("Supply the Go2 scene and optionally a trace directory");
    sire::simulator::Simulator simulator;
    aris::core::fromXmlFile(simulator, argc > 1 ? argv[1] : "go2_rai_foot.xml");
    simulator.init();
    auto& cpu = simulator.model();
    const auto model = loadRigidBodyModel(cpu);
    if (model.bodies.size() != 18 || model.joint_count != 12) {
      throw std::runtime_error("Unexpected Go2 rigid dynamics model dimensions");
    }
    testTorques(simulator, model);
    if (argc == 3) replay(simulator, model, argv[2]);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
