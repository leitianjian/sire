#include "rigid_body_batch.hpp"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <string>

#include <aris/dynamic/model.hpp>

#include "flat_ground_model.hpp"
#include "rigid_body_integration.hpp"
#include "rigid_body_joint_state.hpp"
#include "rigid_body_limits.hpp"
#include "rigid_body_reset.hpp"
#include "sire/physics/contact/simple_admm_contact_solver.hpp"
#include "sire/physics/physics_engine.hpp"

namespace sire::physics::contact::cuda {
namespace {

void checkCuda(cudaError_t result, const char* operation) {
  if (result != cudaSuccess) {
    throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(result));
  }
}

auto product(std::initializer_list<std::size_t> extents) -> std::size_t {
  std::size_t size = 1;
  for (const auto extent : extents) {
    if (extent && size > std::numeric_limits<std::size_t>::max() / extent) {
      throw std::length_error("CUDA rigid-body batch exceeds the supported buffer size");
    }
    size *= extent;
  }
  return size;
}

template <class T>
class DeviceBuffer {
 public:
  DeviceBuffer() = default;
  ~DeviceBuffer() { if (data_) cudaFree(data_); }
  DeviceBuffer(const DeviceBuffer&) = delete;
  auto operator=(const DeviceBuffer&) -> DeviceBuffer& = delete;

  void allocate(std::initializer_list<std::size_t> extents) {
    size_ = product(extents);
    if (size_) {
      checkCuda(cudaMalloc(reinterpret_cast<void**>(&data_), bytes()), "Allocate CUDA rigid-body buffer");
    }
  }

  void upload(const std::vector<T>& source) {
    allocate({source.size()});
    if (size_) checkCuda(cudaMemcpy(data_, source.data(), bytes(), cudaMemcpyHostToDevice),
                         "Upload CUDA rigid-body metadata");
  }

  auto data() const -> T* { return data_; }
  auto size() const -> std::size_t { return size_; }
  auto bytes() const -> std::size_t { return product({size_, sizeof(T)}); }

 private:
  T* data_{};
  std::size_t size_{};
};

class Stream {
 public:
  Stream() {
    checkCuda(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking),
              "Create CUDA rigid-body setup stream");
  }
  ~Stream() { if (stream_) cudaStreamDestroy(stream_); }
  Stream(const Stream&) = delete;
  auto operator=(const Stream&) -> Stream& = delete;
  auto get() const -> cudaStream_t { return stream_; }

 private:
  cudaStream_t stream_{};
};

class Graph {
 public:
  ~Graph() {
    if (executable) cudaGraphExecDestroy(executable);
    if (graph) cudaGraphDestroy(graph);
  }
  Graph() = default;
  Graph(const Graph&) = delete;
  auto operator=(const Graph&) -> Graph& = delete;

  cudaGraph_t graph{};
  cudaGraphExec_t executable{};
};

void requireFinite(const std::vector<double>& values) {
  if (!std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); })) {
    throw std::invalid_argument("Non-finite initial state in CUDA rigid-body model");
  }
}

}  // namespace

struct RigidBodyBatch::Imp {
  Imp(PhysicsEngine& engine, std::size_t count, double physics, double control_step,
      const std::vector<RigidJointControl>& joint_control);

  void allocate();
  void buildGraph(cudaStream_t stream);
  void launchEvent(cudaStream_t stream);
  void readJoints(cudaStream_t stream, bool with_acceleration,
                  const ContactHistoryConfig* events = nullptr);
  void reset(const bool* mask, const double* base_poses, const double* base_velocities,
             const double* joint_positions, const double* joint_rates, cudaStream_t stream);
  void checkDevice() const;

  const std::size_t batch;
  const double physics_dt, control_dt;
  int device{};
  std::size_t root{}, capacity{}, dof{};
  RigidBodyTree tree{};
  GroundBox ground{};
  RigidBodyLimitSettings limit_settings{};

  DeviceBuffer<RigidBody> bodies;
  DeviceBuffer<std::size_t> order;
  DeviceBuffer<GroundCollider> colliders;
  DeviceBuffer<RigidBodyJointLimits> limits;
  DeviceBuffer<RigidJointControl> control;
  DeviceBuffer<double> initial_poses, initial_velocities, initial_positions, initial_rates;

  DeviceBuffer<ContactHistoryConfig> configs;
  DeviceBuffer<RigidDynamicsStatus> statuses;
  DeviceBuffer<RigidEventClock> clocks;
  DeviceBuffer<bool> rebuild, ground_contacts;
  DeviceBuffer<float> actions;
  DeviceBuffer<double> poses, velocities, positions, rates, joint_accelerations, motor_torques;
  DeviceBuffer<double> accelerations, jacobians, mass_factor, dynamics_scratch, generalized_scratch;
  DeviceBuffer<double> reactions, reaction_scratch, steps, integration_scratch, contact_wrenches;

  DeviceBuffer<WorldContact> world_contacts;
  DeviceBuffer<DetectedContact> detections;
  DeviceBuffer<GroundCollisionStatus> collision_status;
  DeviceBuffer<std::size_t> penetration_counts, force_counts, selected_indices;
  DeviceBuffer<ContactPenetrationCache> penetrations, temporary_penetrations;
  DeviceBuffer<ContactForceCache> cached_forces;
  DeviceBuffer<double> cached_dual, relative_velocity, force_guess, dual_guess;
  DeviceBuffer<ContactMaterialState> material;
  DeviceBuffer<AdmmProblem> problems, assembled_problems, accepted_problems;
  DeviceBuffer<double> body_inertia, body_acceleration, contact_factor;
  DeviceBuffer<double> friction, relative_inertia, relative_acceleration;
  DeviceBuffer<double> normal_matrix, normal_bias, initial_normal, normal_target;
  DeviceBuffer<NormalContactResult> normal;
  DeviceBuffer<AdmmResult> admm_result;
  DeviceBuffer<double> forces, dual, normal_scratch;
  DeviceBuffer<unsigned char> admm_scratch;

  Graph graph;
  auto history() const -> ContactHistory {
    return {penetration_counts.data(), penetrations.data(), force_counts.data(),
            cached_forces.data(), cached_dual.data()};
  }
};

RigidBodyBatch::Imp::Imp(PhysicsEngine& engine, std::size_t count, double physics,
                        double control_step, const std::vector<RigidJointControl>& joint_control)
    : batch(count), physics_dt(physics), control_dt(control_step) {
  if (!batch || batch > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
      !std::isfinite(physics_dt) || physics_dt <= 0.0 ||
      !std::isfinite(control_dt) || control_dt <= 0.0) {
    throw std::invalid_argument("Invalid CUDA rigid-body batch size or event intervals");
  }
  auto* model = engine.currentModel();
  if (!model) throw std::invalid_argument("Initialize the Sire engine before constructing a CUDA batch");
  const auto* solver = dynamic_cast<const simple_admm::ShiftedSpectralAdmmContactSolver*>(
      &engine.contactSolver());
  if (!solver || !solver->singlePointContactMode() || solver->contactTimeMethod() != "exponential" ||
      !engine.collisionDetectionFlag() || !engine.contactSolverFlag()) {
    throw std::invalid_argument("CUDA rigid-body batches require enabled exponential single-point shifted spectral ADMM");
  }

  const auto rigid_model = loadRigidBodyModel(*model);
  const auto ground_model = loadFlatGroundModel(engine);
  const auto limit_model = loadRigidBodyLimitsModel(engine);
  if (rigid_model.ground != ground_model.ground.body ||
      rigid_model.bodies.size() != ground_model.body_count ||
      rigid_model.joint_count != limit_model.joints.size() ||
      rigid_model.joint_count != joint_control.size() ||
      engine.contactForceIdx() != rigid_model.joint_count || ground_model.colliders.empty()) {
    throw std::invalid_argument("Incompatible CUDA rigid-body, ground, or actuator model layout");
  }
  std::vector<bool> selected(joint_control.size());
  for (const auto& joint : joint_control) {
    if (joint.action >= selected.size() || selected[joint.action] ||
        !std::isfinite(joint.stiffness) || joint.stiffness < 0.0 ||
        !std::isfinite(joint.damping) || joint.damping < 0.0 ||
        !std::isfinite(joint.default_position) || !std::isfinite(joint.action_scale)) {
      throw std::invalid_argument("Invalid CUDA actuator control parameters or action order");
    }
    selected[joint.action] = true;
  }
  checkCuda(cudaGetDevice(&device), "Get CUDA rigid-body batch device");
  root = rigid_model.root;
  capacity = ground_model.colliders.size();
  dof = 6 + rigid_model.joint_count;
  ground = ground_model.ground;
  limit_settings = limit_model.settings;
  bodies.upload(rigid_model.bodies);
  order.upload(rigid_model.order);
  colliders.upload(ground_model.colliders);
  limits.upload(limit_model.joints);
  control.upload(joint_control);
  tree = {bodies.data(), rigid_model.bodies.size(), rigid_model.joint_count, rigid_model.ground, {}};
  std::copy_n(rigid_model.gravity, 6, tree.gravity);

  std::vector<double> pose_template, velocity_template, position_template, rate_template;
  for (const auto& part : model->partPool()) {
    pose_template.insert(pose_template.end(), *part.pm(), *part.pm() + 12);
    velocity_template.insert(velocity_template.end(), part.vs(), part.vs() + 6);
  }
  for (const auto& motion : model->motionPool()) {
    position_template.push_back(motion.mp());
    rate_template.push_back(motion.mv());
  }
  requireFinite(pose_template);
  requireFinite(velocity_template);
  requireFinite(position_template);
  requireFinite(rate_template);
  initial_poses.upload(pose_template);
  initial_velocities.upload(velocity_template);
  initial_positions.upload(position_template);
  initial_rates.upload(rate_template);

  constexpr std::size_t admm_iterations = 1000;
  constexpr double admm_tolerance = 1e-8;
  const ContactHistoryConfig config{0, 0.0, admm_iterations, admm_tolerance,
      solver->initialPenetrationPolicy() == "retain_bounded", solver->warmStartEnabled(), false};
  configs.upload(std::vector<ContactHistoryConfig>(batch, config));
  allocate();

  checkCuda(cudaStreamSynchronize(nullptr), "Finish uploading CUDA scene metadata");
  Stream stream;
  buildGraph(stream.get());
  reset(nullptr, nullptr, nullptr, nullptr, nullptr, stream.get());
  checkCuda(cudaStreamSynchronize(stream.get()), "Initialize CUDA rigid-body state");
}

void RigidBodyBatch::Imp::allocate() {
  const auto body = tree.body_count;
  const auto joints = tree.joint_count;
  const auto body_rows = product({6, capacity});
  const auto relative_rows = product({3, capacity});
  const auto normal_rows = product({2, capacity});
  const auto normal_size = separationWorkspaceSize(batch, capacity);
  const auto admm_size = spectralAdmmWorkspaceSize(batch, capacity);
  const auto reaction_size = jointLimitReactionWorkspaceSize(batch, joints);
  if (!normal_size || !admm_size || (joints && !reaction_size)) {
    throw std::length_error("Unsupported CUDA rigid-body solver workspace size");
  }

  statuses.allocate({batch});
  clocks.allocate({batch});
  rebuild.allocate({batch});
  ground_contacts.allocate({batch, body});
  actions.allocate({batch, joints});
  poses.allocate({batch, body, 12});
  velocities.allocate({batch, body, 6});
  positions.allocate({batch, joints});
  rates.allocate({batch, joints});
  joint_accelerations.allocate({batch, joints});
  motor_torques.allocate({batch, joints});
  accelerations.allocate({batch, body, 6});
  jacobians.allocate({batch, body, 6, dof});
  mass_factor.allocate({batch, dof, dof});
  dynamics_scratch.allocate({batch, body, 42 + 6 * dof});
  generalized_scratch.allocate({batch, dof});
  reactions.allocate({batch, joints});
  reaction_scratch.allocate({reaction_size});
  steps.allocate({batch});
  integration_scratch.allocate({batch, body, 6});
  contact_wrenches.allocate({batch, body, 6});

  world_contacts.allocate({batch, capacity});
  detections.allocate({batch, capacity});
  collision_status.allocate({batch});
  penetration_counts.allocate({batch});
  force_counts.allocate({batch});
  selected_indices.allocate({batch, capacity});
  penetrations.allocate({batch, capacity});
  temporary_penetrations.allocate({batch, capacity});
  cached_forces.allocate({batch, capacity});
  cached_dual.allocate({batch, relative_rows});
  relative_velocity.allocate({batch, relative_rows});
  force_guess.allocate({batch, relative_rows});
  dual_guess.allocate({batch, relative_rows});
  material.allocate({batch, capacity});
  problems.allocate({batch});
  assembled_problems.allocate({batch});
  accepted_problems.allocate({batch});
  body_inertia.allocate({batch, body_rows, body_rows});
  body_acceleration.allocate({batch, body_rows});
  contact_factor.allocate({batch, body_rows, dof});
  friction.allocate({batch, capacity});
  relative_inertia.allocate({batch, relative_rows, relative_rows});
  relative_acceleration.allocate({batch, relative_rows});
  normal_matrix.allocate({batch, normal_rows, normal_rows});
  normal_bias.allocate({batch, normal_rows});
  initial_normal.allocate({batch, normal_rows});
  normal_target.allocate({batch, capacity});
  normal.allocate({batch});
  admm_result.allocate({batch});
  forces.allocate({batch, relative_rows});
  dual.allocate({batch, relative_rows});
  normal_scratch.allocate({normal_size});
  admm_scratch.allocate({admm_size});
}

void RigidBodyBatch::Imp::buildGraph(cudaStream_t stream) {
#if CUDART_VERSION >= 12040
  checkCuda(cudaGraphCreate(&graph.graph, 0), "Create CUDA rigid-body graph");
  cudaGraphConditionalHandle condition{};
  checkCuda(cudaGraphConditionalHandleCreate(&condition, graph.graph, 1, cudaGraphCondAssignDefault),
            "Create CUDA rigid-body graph loop condition");
  cudaGraphNodeParams params{};
  params.type = cudaGraphNodeTypeConditional;
  params.conditional.handle = condition;
  params.conditional.type = cudaGraphCondTypeWhile;
  params.conditional.size = 1;
  cudaGraphNode_t loop{};
  checkCuda(cudaGraphAddNode(&loop, graph.graph, nullptr, 0, &params), "Create CUDA rigid-body graph loop");
  checkCuda(cudaStreamBeginCaptureToGraph(stream, params.conditional.phGraph_out[0], nullptr,
                                        nullptr, 0, cudaStreamCaptureModeThreadLocal),
            "Begin capturing CUDA rigid-body event");
  cudaGraph_t captured{};
  try {
    launchEvent(stream);
    checkCuda(launchSetRigidEventGraphCondition(batch, clocks.data(), condition, stream),
              "Update CUDA rigid-body graph loop condition");
  } catch (...) {
    cudaStreamEndCapture(stream, &captured);
    throw;
  }
  checkCuda(cudaStreamEndCapture(stream, &captured), "Finish capturing CUDA rigid-body event");
  checkCuda(cudaGraphInstantiate(&graph.executable, graph.graph, 0), "Instantiate CUDA rigid-body graph");
#else
  static_cast<void>(stream);
  throw std::runtime_error("CUDA rigid-body batches require CUDA 12.4 or newer for conditional event graphs");
#endif
}

void RigidBodyBatch::Imp::readJoints(cudaStream_t stream, bool with_acceleration,
                                    const ContactHistoryConfig* events) {
  checkCuda(launchReadRigidJointState(batch, tree, poses.data(), velocities.data(),
      with_acceleration ? accelerations.data() : nullptr, positions.data(), rates.data(),
      with_acceleration ? joint_accelerations.data() : nullptr, stream,
      events, with_acceleration ? rebuild.data() : nullptr), "Read CUDA joint state");
}

void RigidBodyBatch::Imp::launchEvent(cudaStream_t stream) {
  const auto body = tree.body_count;
  const auto joints = tree.joint_count;
  checkCuda(launchPrepareRigidEvent(batch, clocks.data(), statuses.data(), physics_dt,
                                  control_dt, configs.data(), stream), "Prepare CUDA event");
  if (joints) {
    readJoints(stream, false, configs.data());
    checkCuda(launchPrepareRigidJointControls(batch, joints, limits.data(), limit_settings,
        configs.data(), actions.data(), control.data(), positions.data(), rates.data(),
        motor_torques.data(), rebuild.data(), statuses.data(), stream), "Prepare CUDA joint controls");
    checkCuda(launchRebuildRigidBodyState(batch, tree, order.data(), rebuild.data(), positions.data(),
        rates.data(), poses.data(), velocities.data(), stream), "Apply CUDA joint projection");
  }

  checkCuda(launchRigidBodyDynamics(batch, tree,
      {poses.data(), velocities.data(), motor_torques.data(), nullptr, configs.data()},
      {accelerations.data(), jacobians.data(), mass_factor.data(), statuses.data()},
      dynamics_scratch.data(), stream), "Solve CUDA free-body dynamics");
  if (joints && limit_settings.method == RigidBodyLimitMethod::ShiftedNcp) {
    readJoints(stream, true, configs.data());
    checkCuda(launchJointLimitReactions(batch, joints, limits.data(), limit_settings, physics_dt,
        configs.data(), positions.data(), rates.data(), joint_accelerations.data(), mass_factor.data(),
        statuses.data(), reactions.data(), reaction_scratch.data(), stream), "Solve CUDA joint limit reactions");
    checkCuda(launchApplyRigidJointReactions(batch, body, joints, reactions.data(), jacobians.data(),
        mass_factor.data(), statuses.data(), accelerations.data(), generalized_scratch.data(),
        configs.data(), stream), "Apply CUDA joint limit reactions");
  }

  checkCuda(launchFlatGroundCollision(batch, body, capacity, ground, colliders.data(), colliders.size(),
      poses.data(), configs.data(), world_contacts.data(), collision_status.data(), stream),
      "Detect CUDA ground contacts");
  checkCuda(launchFrameContacts(batch, capacity, body, configs.data(), world_contacts.data(),
      velocities.data(), detections.data(), stream), "Frame CUDA ground contacts");
  checkCuda(launchPrepareContactHistory(batch, capacity, configs.data(), detections.data(), history(),
      temporary_penetrations.data(), {problems.data(), selected_indices.data(), material.data(),
      relative_velocity.data(), force_guess.data(), dual_guess.data()}, stream), "Prepare CUDA contact history");
  checkCuda(launchRigidBodyContact(batch, body, joints, capacity, velocities.data(), accelerations.data(),
      jacobians.data(), mass_factor.data(), statuses.data(), detections.data(), selected_indices.data(),
      problems.data(), body_inertia.data(), body_acceleration.data(), contact_factor.data(), stream),
      "Project CUDA body contact dynamics");

  const AdmmOutputs solved{forces.data(), dual.data(), admm_result.data()};
  checkCuda(launchBodyContactStep(batch, capacity, {problems.data(), material.data(), body_inertia.data(),
      body_acceleration.data(), relative_velocity.data(), force_guess.data(), dual_guess.data()},
      {normal.data(), normal_target.data(), solved}, {assembled_problems.data(), friction.data(),
      relative_inertia.data(), relative_acceleration.data(), normal_matrix.data(), normal_bias.data(),
      initial_normal.data()}, {accepted_problems.data(), normal_scratch.data(), normal_scratch.size(),
      admm_scratch.data(), admm_scratch.size()}, stream), "Solve CUDA contact trajectory and forces");
  checkCuda(launchCommitContactHistory(batch, capacity, body, configs.data(), detections.data(),
      problems.data(), selected_indices.data(), solved, history(), contact_wrenches.data(), nullptr,
      stream, ground_contacts.data(), tree.ground), "Commit CUDA contact history");
  checkCuda(launchSelectRigidEventStep(batch, configs.data(), collision_status.data(), normal.data(),
      admm_result.data(), statuses.data(), steps.data(), stream), "Select CUDA event step");
  checkCuda(launchApplyRigidBodyWrenches(batch, body, joints, contact_wrenches.data(), accelerations.data(),
      jacobians.data(), mass_factor.data(), statuses.data(), accelerations.data(),
      generalized_scratch.data(), stream, configs.data()), "Apply CUDA contact wrenches");
  checkCuda(launchIntegrateRigidBodyTree(batch, tree, order.data(), steps.data(), accelerations.data(),
      poses.data(), velocities.data(), statuses.data(), integration_scratch.data(), stream),
      "Integrate CUDA rigid-body event");
  checkCuda(launchFinishRigidEvent(batch, configs.data(), steps.data(), physics_dt, control_dt,
      clocks.data(), statuses.data(), stream), "Finish CUDA rigid-body event");
}

void RigidBodyBatch::Imp::reset(
    const bool* mask, const double* base_poses, const double* base_velocities,
    const double* joint_positions, const double* joint_rates, cudaStream_t stream) {
  const RigidBodyResetSource source{initial_poses.data(), initial_velocities.data(),
      initial_positions.data(), initial_rates.data(), mask, base_poses, base_velocities,
      joint_positions, joint_rates};
  const RigidBodyResetState state{poses.data(), velocities.data(), positions.data(), rates.data(),
      motor_torques.data(), contact_wrenches.data(), ground_contacts.data(), statuses.data(),
      clocks.data(), configs.data()};
  checkCuda(launchResetRigidBodyBatch(batch, tree.body_count, tree.joint_count, root, source, state,
                                    history(), stream), "Reset CUDA rigid-body environments");
  checkCuda(launchRebuildRigidBodyState(batch, tree, order.data(), mask, positions.data(), rates.data(),
      poses.data(), velocities.data(), stream), "Rebuild reset CUDA rigid-body environments");
}

void RigidBodyBatch::Imp::checkDevice() const {
  int current{};
  checkCuda(cudaGetDevice(&current), "Get current CUDA device");
  if (current != device) throw std::invalid_argument("CUDA rigid-body batch belongs to another device");
}

RigidBodyBatch::RigidBodyBatch(PhysicsEngine& engine, std::size_t batch, double physics_dt,
                               double control_dt, const std::vector<RigidJointControl>& control)
    : imp_(std::make_unique<Imp>(engine, batch, physics_dt, control_dt, control)) {}

RigidBodyBatch::~RigidBodyBatch() {
  int previous{};
  if (cudaGetDevice(&previous) == cudaSuccess && previous != imp_->device &&
      cudaSetDevice(imp_->device) == cudaSuccess) {
    imp_.reset();
    cudaSetDevice(previous);
  }
}

auto RigidBodyBatch::batchSize() const -> std::size_t { return imp_->batch; }
auto RigidBodyBatch::bodyCount() const -> std::size_t { return imp_->tree.body_count; }
auto RigidBodyBatch::jointCount() const -> std::size_t { return imp_->tree.joint_count; }
auto RigidBodyBatch::rootBody() const -> std::size_t { return imp_->root; }

auto RigidBodyBatch::state() const -> RigidBodyBatchState {
  const auto& state = *imp_;
  return {state.poses.data(), state.velocities.data(), state.positions.data(), state.rates.data(),
          state.motor_torques.data(), state.contact_wrenches.data(), state.ground_contacts.data(),
          state.statuses.data(), state.clocks.data()};
}

void RigidBodyBatch::step(const float* actions, cudaStream_t stream) {
  auto& state = *imp_;
  state.checkDevice();
  if (state.actions.size()) {
    const auto result = actions
        ? cudaMemcpyAsync(state.actions.data(), actions, state.actions.bytes(), cudaMemcpyDeviceToDevice, stream)
        : cudaMemsetAsync(state.actions.data(), 0, state.actions.bytes(), stream);
    checkCuda(result, "Set CUDA rigid-body actions");
  }
  checkCuda(launchBeginRigidControl(state.batch, state.clocks.data(), state.statuses.data(), stream),
            "Begin CUDA rigid-body control");
  checkCuda(cudaGraphLaunch(state.graph.executable, stream), "Launch CUDA rigid-body control");
  if (state.tree.joint_count) {
    state.readJoints(stream, false);
    checkCuda(launchProjectRigidJointLimits(state.batch, state.tree.joint_count, state.limits.data(),
        state.limit_settings, state.positions.data(), state.rates.data(), state.rebuild.data(),
        state.statuses.data(), stream), "Project final CUDA joint state");
    checkCuda(launchRebuildRigidBodyState(state.batch, state.tree, state.order.data(), state.rebuild.data(),
        state.positions.data(), state.rates.data(), state.poses.data(), state.velocities.data(), stream),
        "Rebuild projected CUDA joint state");
  }
}

void RigidBodyBatch::reset(const bool* mask, const double* base_poses, const double* base_velocities,
                          const double* joint_positions, const double* joint_rates, cudaStream_t stream) {
  imp_->checkDevice();
  imp_->reset(mask, base_poses, base_velocities, joint_positions, joint_rates, stream);
}

}  // namespace sire::physics::contact::cuda
