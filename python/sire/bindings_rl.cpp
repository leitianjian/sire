#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <exception>
#include <functional>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <aris.hpp>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "sire/actuator/actuator.hpp"
#include "sire/physics/physics_engine.hpp"
#include "sire/simulator/simulator.hpp"
#include "sire/core/profiler.hpp"
#include "sire/simulator/event_manager.hpp"

namespace py = pybind11;
using namespace pybind11::literals;

namespace {

class ParallelExecutor {
 public:
  explicit ParallelExecutor(std::size_t thread_count)
      : thread_count_(std::max<std::size_t>(1, thread_count)) {
    workers_.reserve(thread_count_ - 1);
    for (std::size_t i = 1; i < thread_count_; ++i) {
      workers_.emplace_back([this]() { workerLoop(); });
    }
  }

  ParallelExecutor(const ParallelExecutor&) = delete;
  auto operator=(const ParallelExecutor&) -> ParallelExecutor& = delete;

  ~ParallelExecutor() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
      ++generation_;
    }
    work_cv_.notify_all();
    for (auto& worker : workers_) {
      if (worker.joinable()) worker.join();
    }
  }

  auto threadCount() const noexcept -> std::size_t { return thread_count_; }
  auto workerCount() const noexcept -> std::size_t { return workers_.size(); }
  auto dispatchCount() const noexcept -> std::size_t {
    return dispatch_count_.load(std::memory_order_relaxed);
  }

  auto run(std::size_t task_count,
           std::function<void(std::size_t)> task) -> void {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (active_) {
        throw std::logic_error("ParallelExecutor does not support nested run()");
      }
      active_ = true;
      task_ = std::move(task);
      task_count_ = task_count;
      next_task_.store(0, std::memory_order_relaxed);
      completed_workers_ = 0;
      first_exception_ = nullptr;
      ++generation_;
      dispatch_count_.fetch_add(1, std::memory_order_relaxed);
    }
    work_cv_.notify_all();

    // The calling thread is one of the requested threads.  This avoids
    // creating a permanently idle coordinator thread.
    consumeTasks();

    std::exception_ptr error;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      done_cv_.wait(lock,
                    [this]() { return completed_workers_ == workers_.size(); });
      error = first_exception_;
      task_ = {};
      active_ = false;
    }
    if (error) std::rethrow_exception(error);
  }

 private:
  auto consumeTasks() -> void {
    while (true) {
      const std::size_t task_index =
          next_task_.fetch_add(1, std::memory_order_relaxed);
      if (task_index >= task_count_) break;
      try {
        task_(task_index);
      } catch (...) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!first_exception_) first_exception_ = std::current_exception();
      }
    }
  }

  auto workerLoop() -> void {
    std::size_t observed_generation = 0;
    while (true) {
      {
        std::unique_lock<std::mutex> lock(mutex_);
        work_cv_.wait(lock, [this, &observed_generation]() {
          return stopping_ || generation_ != observed_generation;
        });
        if (stopping_) return;
        observed_generation = generation_;
      }

      consumeTasks();

      {
        std::lock_guard<std::mutex> lock(mutex_);
        ++completed_workers_;
        if (completed_workers_ == workers_.size()) done_cv_.notify_one();
      }
    }
  }

  const std::size_t thread_count_;
  std::vector<std::thread> workers_;
  std::mutex mutex_;
  std::condition_variable work_cv_;
  std::condition_variable done_cv_;
  bool stopping_{false};
  bool active_{false};
  std::size_t generation_{0};
  std::size_t completed_workers_{0};
  std::size_t task_count_{0};
  std::function<void(std::size_t)> task_;
  std::atomic<std::size_t> next_task_{0};
  std::atomic<std::size_t> dispatch_count_{0};
  std::exception_ptr first_exception_;
};

class SireRLBatchStepper {
 public:
  SireRLBatchStepper(
      py::sequence simulators,
      py::array_t<int, py::array::c_style | py::array::forcecast>
          motion_indices,
      py::array_t<int, py::array::c_style | py::array::forcecast>
          foot_part_ids,
      std::string control_type, double action_scale,
      py::array_t<double, py::array::c_style | py::array::forcecast> p_gains,
      py::array_t<double, py::array::c_style | py::array::forcecast> d_gains,
      py::array_t<double, py::array::c_style | py::array::forcecast> default_pos,
      py::array_t<double, py::array::c_style | py::array::forcecast>
          torque_limits,
      py::array_t<double, py::array::c_style | py::array::forcecast> dof_lower,
      py::array_t<double, py::array::c_style | py::array::forcecast> dof_upper,
      double simulation_dt, double control_dt, std::size_t thread_count)
      : simulator_owners_(simulators),
        num_envs_(static_cast<std::size_t>(py::len(simulators))),
        control_type_(std::move(control_type)),
        action_scale_(action_scale),
        simulation_dt_(simulation_dt),
        control_dt_(control_dt),
        executor_(resolveThreadCount(thread_count, num_envs_)) {
    if (num_envs_ == 0) {
      throw std::invalid_argument("SireRLBatchStepper requires at least one Simulator");
    }
    if (!(simulation_dt_ > 0.0)) {
      throw std::invalid_argument("simulation_dt must be positive");
    }
    if (!(control_dt_ > 0.0)) {
      throw std::invalid_argument("control_dt must be positive");
    }
    if (control_type_ != "P" && control_type_ != "V" &&
        control_type_ != "T") {
      throw std::invalid_argument("control_type must be P, V, or T");
    }

    motion_indices_ = copyVector(motion_indices, "motion_indices");
    foot_part_ids_ = copyVector(foot_part_ids, "foot_part_ids");
    p_gains_ = copyVector(p_gains, "p_gains");
    d_gains_ = copyVector(d_gains, "d_gains");
    default_pos_ = copyVector(default_pos, "default_pos");
    torque_limits_ = copyVector(torque_limits, "torque_limits");
    dof_lower_ = copyVector(dof_lower, "dof_lower");
    dof_upper_ = copyVector(dof_upper, "dof_upper");
    num_actions_ = motion_indices_.size();
    num_feet_ = foot_part_ids_.size();

    validateControlVector(p_gains_, "p_gains");
    validateControlVector(d_gains_, "d_gains");
    validateControlVector(default_pos_, "default_pos");
    validateControlVector(torque_limits_, "torque_limits");
    validateControlVector(dof_lower_, "dof_lower");
    validateControlVector(dof_upper_, "dof_upper");
    if (num_actions_ == 0) {
      throw std::invalid_argument("motion_indices must not be empty");
    }
    for (std::size_t dof_id = 0; dof_id < num_actions_; ++dof_id) {
      if (!std::isfinite(torque_limits_[dof_id]) ||
          !(torque_limits_[dof_id] > 0.0)) {
        throw std::invalid_argument("torque_limits must be finite and positive");
      }
      if (!std::isfinite(dof_lower_[dof_id]) ||
          !std::isfinite(dof_upper_[dof_id]) ||
          dof_lower_[dof_id] > dof_upper_[dof_id]) {
        throw std::invalid_argument(
            "dof_lower and dof_upper must be finite ordered bounds");
      }
    }

    simulators_.reserve(num_envs_);
    models_.reserve(num_envs_);
    loops_.reserve(num_envs_);
    engines_.reserve(num_envs_);
    for (std::size_t env_id = 0; env_id < num_envs_; ++env_id) {
      auto* simulator =
          simulators[static_cast<py::ssize_t>(env_id)]
              .cast<sire::simulator::Simulator*>();
      if (simulator == nullptr) {
        throw std::invalid_argument("simulators contains a null Simulator");
      }
      auto* model = &simulator->model();
      auto* loop = const_cast<sire::simulator::SimulationLoop*>(
          &simulator->simulationLoop());
      auto* engine = const_cast<sire::physics::PhysicsEngine*>(
          &simulator->physicsEngine());
      if (loop->eventManager().getHandlerIdByEventId(0) != 12 ||
          loop->eventManager().getHandlerIdByEventId(1) != 13 ||
          loop->eventManager().getHandlerIdByEventId(2) != 14) {
        throw std::invalid_argument(
            "SireRLBatchStepper requires initial5/step5/ctrl5 handlers");
      }
      if (env_id == 0) {
        num_bodies_ = model->partPool().size();
      } else if (model->partPool().size() != num_bodies_) {
        throw std::invalid_argument(
            "all Simulator models must have the same number of bodies");
      }
      for (std::size_t dof_id = 0; dof_id < num_actions_; ++dof_id) {
        const int motion_index = motion_indices_[dof_id];
        if (motion_index < 0 ||
            static_cast<std::size_t>(motion_index) >= model->motionPool().size()) {
          throw std::out_of_range("motion index is outside the model motionPool");
        }
        auto* actuator = dynamic_cast<sire::actuator::ActuatorSISO*>(
            &model->motionPool().at(static_cast<std::size_t>(motion_index)));
        if (actuator == nullptr) {
          throw std::invalid_argument(
              "every selected motion must be an ActuatorSISO");
        }
        // The actuator is the single source of truth for physical effort and
        // joint ranges. It applies these guards before every dynamics solve.
        actuator->setMinForce(-torque_limits_[dof_id]);
        actuator->setMaxForce(torque_limits_[dof_id]);
        actuator->setMinPosition(dof_lower_[dof_id]);
        actuator->setMaxPosition(dof_upper_[dof_id]);
      }
      for (int part_id : foot_part_ids_) {
        if (part_id < 0 || static_cast<std::size_t>(part_id) >= num_bodies_) {
          throw std::out_of_range("foot part id is outside the model partPool");
        }
      }
      if (std::find(simulators_.begin(), simulators_.end(), simulator) !=
          simulators_.end()) {
        throw std::invalid_argument("each environment must own a distinct Simulator");
      }
      loop->recorder().setHistoryEnabled(false);
      simulators_.push_back(simulator);
      models_.push_back(model);
      loops_.push_back(loop);
      engines_.push_back(engine);
    }

    previous_dof_vel_.assign(num_envs_ * num_actions_, 0.0);
    errors_.resize(num_envs_);
    recovered_errors_.resize(num_envs_);
    pre_step_snapshots_.resize(num_envs_);
    snapshot_scratch_.resize(num_envs_);
    for (std::size_t env_id = 0; env_id < num_envs_; ++env_id) {
      for (auto* snapshot : {&pre_step_snapshots_[env_id],
                             &snapshot_scratch_[env_id]}) {
        snapshot->mp.resize(num_actions_);
        snapshot->mv.resize(num_actions_);
        snapshot->torque.resize(num_actions_);
      }
    }
    allocateOutputs();
  }

  SireRLBatchStepper(const SireRLBatchStepper&) = delete;
  auto operator=(const SireRLBatchStepper&) -> SireRLBatchStepper& = delete;

  auto step(py::array actions) -> py::tuple {
    py::array_t<float, py::array::c_style | py::array::forcecast>
        contiguous_actions(actions);
    const auto info = contiguous_actions.request();
    if (info.ndim != 2 ||
        static_cast<std::size_t>(info.shape[0]) != num_envs_ ||
        static_cast<std::size_t>(info.shape[1]) != num_actions_) {
      std::ostringstream message;
      message << "actions must have shape [" << num_envs_ << ", "
              << num_actions_ << "]";
      throw std::invalid_argument(message.str());
    }
    const auto* action_data = static_cast<const float*>(info.ptr);
    // Invalid policy output is a global training error. Validate it before any
    // environment advances so a retry cannot double-step healthy environments.
    for (std::size_t i = 0; i < num_envs_ * num_actions_; ++i) {
      if (!std::isfinite(action_data[i])) {
        const std::size_t env_id = i / num_actions_;
        const std::size_t dof_id = i % num_actions_;
        std::ostringstream reason;
        reason << "action contains NaN or Inf at dof_id=" << dof_id;
        throw std::runtime_error(formatError(
            env_id, reason.str(), action_data + env_id * num_actions_));
      }
    }
    std::fill(errors_.begin(), errors_.end(), std::string{});
    std::fill(recovered_errors_.begin(), recovered_errors_.end(), std::string{});

    {
      py::gil_scoped_release release;
      executor_.run(num_envs_, [this, action_data](std::size_t env_id) {
        try {
          stepOne(env_id, action_data + env_id * num_actions_);
        } catch (const std::exception& error) {
          const std::string failure = formatError(
              env_id, error.what(), action_data + env_id * num_actions_);
          try {
            recoverEnvironment(env_id);
            recovered_errors_[env_id] = failure;
          } catch (const std::exception& recovery_error) {
            errors_[env_id] = failure + " recovery_failed=" +
                              std::string(recovery_error.what());
          } catch (...) {
            errors_[env_id] = failure + " recovery_failed=unknown exception";
          }
        } catch (...) {
          const std::string failure = formatError(
              env_id, "unknown C++ exception",
              action_data + env_id * num_actions_);
          try {
            recoverEnvironment(env_id);
            recovered_errors_[env_id] = failure;
          } catch (...) {
            errors_[env_id] = failure + " recovery_failed=unknown exception";
          }
        }
      });
    }

    throwErrors("step");
    total_recovered_failures_ += recoveredEnvIds().size();
    return outputs();
  }

  auto reset(py::array env_ids) -> void {
    py::array_t<std::int64_t, py::array::c_style | py::array::forcecast>
        contiguous_ids(env_ids);
    const auto info = contiguous_ids.request();
    if (info.ndim != 1) {
      throw std::invalid_argument("env_ids must be a one-dimensional array");
    }
    const auto* id_data = static_cast<const std::int64_t*>(info.ptr);
    std::vector<std::size_t> ids(static_cast<std::size_t>(info.shape[0]));
    for (std::size_t i = 0; i < ids.size(); ++i) {
      if (id_data[i] < 0 || static_cast<std::size_t>(id_data[i]) >= num_envs_) {
        throw std::out_of_range("env_ids contains an invalid environment id");
      }
      ids[i] = static_cast<std::size_t>(id_data[i]);
      errors_[ids[i]].clear();
    }
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());

    {
      py::gil_scoped_release release;
      executor_.run(ids.size(), [this, &ids](std::size_t task_id) {
        const std::size_t env_id = ids[task_id];
        try {
          simulators_[env_id]->simReset();
          std::fill_n(previous_dof_vel_.data() + env_id * num_actions_,
                      num_actions_, 0.0);
        } catch (const std::exception& error) {
          errors_[env_id] = formatError(env_id, error.what(), nullptr);
        } catch (...) {
          errors_[env_id] =
              formatError(env_id, "unknown C++ exception", nullptr);
        }
      });
    }
    throwErrors("reset");
  }

  // Call between batch steps, with the GIL held. Only env 0 is recorded.
  auto setHistoryRecording(bool enabled) -> void {
    auto* loop = loops_.front();
    if (loop->recorder().historyEnabled() == enabled) return;
    loop->recorder().setHistoryEnabled(enabled);
    loop->recorder().addRecord(loop->simTime());
  }

  auto resetRecorders() -> void {
    std::fill(errors_.begin(), errors_.end(), std::string{});
    {
      py::gil_scoped_release release;
      executor_.run(num_envs_, [this](std::size_t env_id) {
        try {
          resetRecorderForContinuation(env_id);
        } catch (const std::exception& error) {
          errors_[env_id] = formatError(env_id, error.what(), nullptr);
        } catch (...) {
          errors_[env_id] =
              formatError(env_id, "unknown C++ exception", nullptr);
        }
      });
    }
    throwErrors("resetRecorders");
  }

  auto outputs() const -> py::tuple {
    return py::make_tuple(root_states_, dof_pos_, dof_vel_, torques_,
                          contact_forces_, feet_pos_, body_ground_contact_,
                          foot_ground_contact_, dt_actual_);
  }

  auto threadCount() const noexcept -> std::size_t {
    return executor_.threadCount();
  }
  auto workerCount() const noexcept -> std::size_t {
    return executor_.workerCount();
  }
  auto dispatchCount() const noexcept -> std::size_t {
    return executor_.dispatchCount();
  }
  auto numEnvs() const noexcept -> std::size_t { return num_envs_; }
  auto recoveredEnvIds() const -> std::vector<std::size_t> {
    std::vector<std::size_t> ids;
    for (std::size_t i = 0; i < recovered_errors_.size(); ++i) {
      if (!recovered_errors_[i].empty()) ids.push_back(i);
    }
    return ids;
  }
  auto recoveredErrors() const -> std::vector<std::string> {
    std::vector<std::string> result;
    for (const auto& error : recovered_errors_) {
      if (!error.empty()) result.push_back(error);
    }
    return result;
  }
  auto totalRecoveredFailures() const noexcept -> std::size_t {
    return total_recovered_failures_;
  }

 private:
  struct StepSnapshot {
    bool valid{false};
    double sim_time{0.0};
    std::array<double, 7> pq{};
    std::array<double, 6> vs{};
    std::vector<double> mp;
    std::vector<double> mv;
    std::vector<double> torque;
  };
  template <typename T>
  static auto copyVector(
      const py::array_t<T, py::array::c_style | py::array::forcecast>& array,
      const char* name) -> std::vector<T> {
    const auto info = array.request();
    if (info.ndim != 1) {
      throw std::invalid_argument(std::string(name) +
                                  " must be a one-dimensional array");
    }
    const auto* data = static_cast<const T*>(info.ptr);
    return std::vector<T>(data, data + info.shape[0]);
  }

  static auto resolveThreadCount(std::size_t requested,
                                 std::size_t num_envs) -> std::size_t {
    if (requested == 0) {
      requested = std::max<unsigned int>(1, std::thread::hardware_concurrency());
    }
    return std::max<std::size_t>(1, std::min(requested, num_envs));
  }

  auto validateControlVector(const std::vector<double>& values,
                             const char* name) const -> void {
    if (values.size() != num_actions_) {
      throw std::invalid_argument(std::string(name) +
                                  " length must match motion_indices");
    }
  }

  auto allocateOutputs() -> void {
    const auto e = static_cast<py::ssize_t>(num_envs_);
    const auto a = static_cast<py::ssize_t>(num_actions_);
    const auto b = static_cast<py::ssize_t>(num_bodies_);
    const auto f = static_cast<py::ssize_t>(num_feet_);
    root_states_ = py::array_t<float>({e, py::ssize_t(13)});
    dof_pos_ = py::array_t<float>({e, a});
    dof_vel_ = py::array_t<float>({e, a});
    torques_ = py::array_t<float>({e, a});
    contact_forces_ = py::array_t<float>({e, b, py::ssize_t(3)});
    feet_pos_ = py::array_t<float>({e, f, py::ssize_t(3)});
    body_ground_contact_ = py::array_t<bool>({e, b});
    foot_ground_contact_ = py::array_t<bool>({e, f});
    dt_actual_ = py::array_t<double>({e});

    root_states_data_ = root_states_.mutable_data();
    dof_pos_data_ = dof_pos_.mutable_data();
    dof_vel_data_ = dof_vel_.mutable_data();
    torques_data_ = torques_.mutable_data();
    contact_forces_data_ = contact_forces_.mutable_data();
    feet_pos_data_ = feet_pos_.mutable_data();
    body_ground_contact_data_ = body_ground_contact_.mutable_data();
    foot_ground_contact_data_ = foot_ground_contact_.mutable_data();
    dt_actual_data_ = dt_actual_.mutable_data();
  }

  auto updateActuatorTorque(std::size_t env_id, const float* actions) -> void {
    auto& motion_pool = models_[env_id]->motionPool();
    for (std::size_t dof_id = 0; dof_id < num_actions_; ++dof_id) {
      if (!std::isfinite(actions[dof_id])) {
        throw std::runtime_error("action contains NaN or Inf");
      }
      const auto motion_id =
          static_cast<std::size_t>(motion_indices_[dof_id]);
      auto& motion = motion_pool.at(motion_id);
      const double scaled_action =
          static_cast<double>(actions[dof_id]) * action_scale_;
      double torque = 0.0;
      if (control_type_ == "P") {
        torque = p_gains_[dof_id] *
                     (scaled_action + default_pos_[dof_id] - motion.mp()) -
                 d_gains_[dof_id] * motion.mv();
      } else if (control_type_ == "V") {
        const double previous_velocity =
            previous_dof_vel_[env_id * num_actions_ + dof_id];
        torque = p_gains_[dof_id] * (scaled_action - motion.mv()) -
                 d_gains_[dof_id] *
                     (motion.mv() - previous_velocity) / control_dt_;
      } else {
        torque = scaled_action;
      }
      auto* actuator =
          dynamic_cast<sire::actuator::ActuatorSISO*>(&motion);
      if (actuator == nullptr) {
        throw std::runtime_error("selected motion is no longer an ActuatorSISO");
      }
      actuator->setDesiredValue(torque);
      torques_data_[env_id * num_actions_ + dof_id] =
          static_cast<float>(actuator->limitedDesiredValue());
    }
  }

  auto stepOne(std::size_t env_id, const float* actions) -> void {
    auto* loop = loops_[env_id];
    const double start_time = loop->simTime();
    std::size_t event_count = 0;
    // One control-boundary snapshot is sufficient to diagnose and recover a
    // failed rollout. Taking it for every 1 kHz event adds avoidable overhead
    // to all healthy environments; the integrator itself guards every event.
    capturePreStepSnapshot(env_id);
    // Handler5 integrates forward from the current event. Leave the next
    // control event pending so the next action owns that control interval.
    do {
      if (++event_count > 100000) {
        throw std::runtime_error("control interval exceeded 100000 events");
      }
      updateActuatorTorque(env_id, actions);
      loop->handleContact();
    } while (!loop->headerIsCtrl());
    dt_actual_data_[env_id] = loop->simTime() - start_time;
    readState(env_id);

  }

  auto capturePreStepSnapshot(std::size_t env_id) -> void {
    auto& snapshot = snapshot_scratch_[env_id];
    auto* model = models_[env_id];
    snapshot.sim_time = loops_[env_id]->simTime();
    model->partPool().at(1).getPq(snapshot.pq.data());
    model->partPool().at(1).getVs(snapshot.vs.data());
    auto& motion_pool = model->motionPool();
    for (std::size_t dof_id = 0; dof_id < num_actions_; ++dof_id) {
      auto& motion = motion_pool.at(
          static_cast<std::size_t>(motion_indices_[dof_id]));
      snapshot.mp[dof_id] = motion.mp();
      snapshot.mv[dof_id] = motion.mv();
      auto* actuator = dynamic_cast<sire::actuator::ActuatorSISO*>(&motion);
      snapshot.torque[dof_id] =
          actuator == nullptr ? 0.0 : actuator->appliedValue();
    }
    // Catch an already-diverging state before another contact solve/integration
    // turns it into Inf or NaN. These are numerical-failure bounds, not model
    // limits and are deliberately much wider than normal GO2 motion.
    for (double value : snapshot.pq) {
      if (!std::isfinite(value))
        throw std::runtime_error("pre-step base pose contains NaN or Inf");
    }
    for (double value : snapshot.vs) {
      if (!std::isfinite(value) || std::abs(value) > 1e4)
        throw std::runtime_error("pre-step base twist exceeded safety bound");
    }
    for (std::size_t i = 0; i < num_actions_; ++i) {
      if (!std::isfinite(snapshot.mp[i]) || !std::isfinite(snapshot.mv[i]) ||
          std::abs(snapshot.mv[i]) > 1e4) {
        throw std::runtime_error("pre-step joint state exceeded safety bound");
      }
    }
    snapshot.valid = true;
    std::swap(pre_step_snapshots_[env_id], snapshot_scratch_[env_id]);
  }

  auto recoverEnvironment(std::size_t env_id) -> void {
    simulators_[env_id]->simReset();
    std::fill_n(previous_dof_vel_.data() + env_id * num_actions_,
                num_actions_, 0.0);

    float* root = root_states_data_ + env_id * 13;
    std::fill_n(root, std::size_t(13), 0.0F);
    root[6] = 1.0F;
    std::fill_n(dof_pos_data_ + env_id * num_actions_, num_actions_, 0.0F);
    std::fill_n(dof_vel_data_ + env_id * num_actions_, num_actions_, 0.0F);
    std::fill_n(torques_data_ + env_id * num_actions_, num_actions_, 0.0F);
    std::fill_n(contact_forces_data_ + env_id * num_bodies_ * std::size_t(3),
                num_bodies_ * std::size_t(3), 0.0F);
    std::fill_n(body_ground_contact_data_ + env_id * num_bodies_, num_bodies_,
                false);
    std::fill_n(foot_ground_contact_data_ + env_id * num_feet_, num_feet_,
                false);
    std::fill_n(feet_pos_data_ + env_id * num_feet_ * std::size_t(3),
                num_feet_ * std::size_t(3), 0.0F);
    dt_actual_data_[env_id] = 0.0;
  }

  auto resetRecorderForContinuation(std::size_t env_id) -> void {
    auto* loop = loops_[env_id];
    loop->resetRecorder();
    // The contact solvers write into recorder().records.back() before they
    // append the next frame.  A bare reset leaves the vector empty and makes
    // the next physics step invoke undefined behaviour.
    loop->recorder().addRecord(loop->simTime());
  }

  auto readState(std::size_t env_id) -> void {
    auto* model = models_[env_id];
    engines_[env_id]->enforceJointLimitSafety();
    auto& base = model->partPool().at(1);
    double pq[7]{0.0};
    double vs[6]{0.0};
    double vp[3]{0.0};
    base.getPq(pq);
    base.getVs(vs);
    aris::dynamic::s_vs2vp(vs, pq, vp);

    if (std::abs(pq[0]) > 100.0 || std::abs(pq[1]) > 100.0 ||
        pq[2] < -5.0 || pq[2] > 50.0 || std::abs(vp[0]) > 100.0 ||
        std::abs(vp[1]) > 100.0 || std::abs(vp[2]) > 100.0 ||
        std::abs(vs[3]) > 100.0 || std::abs(vs[4]) > 100.0 ||
        std::abs(vs[5]) > 100.0) {
      throw std::runtime_error("physics state exceeded configured safety bounds");
    }
    for (double value : pq) {
      if (!std::isfinite(value))
        throw std::runtime_error("base pose contains NaN or Inf");
    }
    for (double value : vs) {
      if (!std::isfinite(value))
        throw std::runtime_error("base velocity contains NaN or Inf");
    }

    float* root = root_states_data_ + env_id * 13;
    for (std::size_t i = 0; i < 7; ++i) root[i] = static_cast<float>(pq[i]);
    for (std::size_t i = 0; i < 3; ++i) root[7 + i] = static_cast<float>(vp[i]);
    for (std::size_t i = 0; i < 3; ++i)
      root[10 + i] = static_cast<float>(vs[3 + i]);

    auto& motion_pool = model->motionPool();
    for (std::size_t dof_id = 0; dof_id < num_actions_; ++dof_id) {
      auto& motion = motion_pool.at(
          static_cast<std::size_t>(motion_indices_[dof_id]));
      auto* actuator = dynamic_cast<sire::actuator::ActuatorSISO*>(&motion);
      if (actuator == nullptr) {
        throw std::runtime_error("selected motion is no longer an ActuatorSISO");
      }
      double position = motion.mp();
      double velocity = motion.mv();
      if (!std::isfinite(position) || !std::isfinite(velocity)) {
        throw std::runtime_error("joint state contains NaN or Inf");
      }
      const std::size_t offset = env_id * num_actions_ + dof_id;
      dof_pos_data_[offset] = static_cast<float>(position);
      dof_vel_data_[offset] = static_cast<float>(velocity);
      torques_data_[offset] = static_cast<float>(actuator->appliedValue());
      previous_dof_vel_[offset] = velocity;
    }

    float* feet = feet_pos_data_ + env_id * num_feet_ * 3;
    for (std::size_t foot_id = 0; foot_id < num_feet_; ++foot_id) {
      double pm[16]{0.0};
      model->partPool()
          .at(static_cast<std::size_t>(foot_part_ids_[foot_id]))
          .getPm(pm);
      feet[foot_id * 3] = static_cast<float>(pm[3]);
      feet[foot_id * 3 + 1] = static_cast<float>(pm[7]);
      feet[foot_id * 3 + 2] = static_cast<float>(pm[11]);
    }

    float* contact =
        contact_forces_data_ + env_id * num_bodies_ * std::size_t(3);
    bool* body_contact =
        body_ground_contact_data_ + env_id * num_bodies_;
    bool* foot_contact = foot_ground_contact_data_ + env_id * num_feet_;
    std::fill_n(contact, num_bodies_ * std::size_t(3), 0.0F);
    std::fill_n(body_contact, num_bodies_, false);
    std::fill_n(foot_contact, num_feet_, false);

    const auto& latest_contact_results =
        loops_[env_id]->recorder().latestContactPairResults();
    {
      // Latest physical-substep forces in N, not an impulse or an average
      // over the control interval. Do not scale by either timestep.
      for (const auto& result : latest_contact_results) {
        const auto* geom_a =
            engines_[env_id]->queryGeometryPoolById(result.geomIdA);
        const auto* geom_b =
            engines_[env_id]->queryGeometryPoolById(result.geomIdB);
        const std::size_t part_a =
            geom_a == nullptr ? 0 : static_cast<std::size_t>(geom_a->partId());
        const std::size_t part_b =
            geom_b == nullptr ? 0 : static_cast<std::size_t>(geom_b->partId());
        if (part_a >= num_bodies_ || part_b >= num_bodies_) {
          throw std::runtime_error("contact result contains an invalid part id");
        }
        for (std::size_t axis = 0; axis < 3; ++axis) {
          const float force =
              static_cast<float>(result.force_W[axis]);
          contact[part_a * 3 + axis] -= force;
          contact[part_b * 3 + axis] += force;
        }
        if (part_a == 0 && part_b != 0) body_contact[part_b] = true;
        if (part_b == 0 && part_a != 0) body_contact[part_a] = true;
      }
    }
    for (std::size_t foot_id = 0; foot_id < num_feet_; ++foot_id) {
      foot_contact[foot_id] =
          body_contact[static_cast<std::size_t>(foot_part_ids_[foot_id])];
    }
  }

  auto formatError(std::size_t env_id, const std::string& reason,
                   const float* actions) -> std::string {
    std::ostringstream message;
    message << std::setprecision(17) << "env_id=" << env_id
            << " reason=" << reason;
    try {
      auto* loop = loops_.at(env_id);
      auto* model = models_.at(env_id);
      double pq[7]{0.0};
      double vs[6]{0.0};
      model->partPool().at(1).getPq(pq);
      model->partPool().at(1).getVs(vs);
      message << " sim_time=" << loop->simTime() << " pq=[";
      for (std::size_t i = 0; i < 7; ++i)
        message << (i == 0 ? "" : ",") << pq[i];
      message << "] vs=[";
      for (std::size_t i = 0; i < 6; ++i)
        message << (i == 0 ? "" : ",") << vs[i];
      message << "] mp=[";
      for (std::size_t i = 0; i < num_actions_; ++i) {
        const auto& motion = model->motionPool().at(
            static_cast<std::size_t>(motion_indices_[i]));
        message << (i == 0 ? "" : ",") << motion.mp();
      }
      message << "] mv=[";
      for (std::size_t i = 0; i < num_actions_; ++i) {
        const auto& motion = model->motionPool().at(
            static_cast<std::size_t>(motion_indices_[i]));
        message << (i == 0 ? "" : ",") << motion.mv();
      }
      message << "]";
      const auto& engine = *engines_.at(env_id);
      message << " joint_limit={active="
              << engine.jointLimitLastActiveCount()
              << ",iterations=" << engine.jointLimitLastIterations()
              << ",residual=" << engine.jointLimitLastResidual()
              << ",max_reaction=" << engine.jointLimitLastMaxReaction()
              << ",saturated=" << engine.jointLimitLastSaturatedCount()
              << "}";
      const auto& contacts = loop->recorder().latestContactPairResults();
      double max_contact_force = 0.0;
      for (const auto& contact : contacts) {
        const double norm = std::sqrt(
            contact.force_W[0] * contact.force_W[0] +
            contact.force_W[1] * contact.force_W[1] +
            contact.force_W[2] * contact.force_W[2]);
        max_contact_force = std::max(max_contact_force, norm);
      }
      message << " contacts={count=" << contacts.size()
              << ",max_force=" << max_contact_force << "}";
    } catch (const std::exception& snapshot_error) {
      message << " state_snapshot_error=" << snapshot_error.what();
    } catch (...) {
      message << " state_snapshot_error=unknown";
    }
    if (actions != nullptr) {
      message << " actions=[";
      for (std::size_t i = 0; i < num_actions_; ++i)
        message << (i == 0 ? "" : ",") << actions[i];
      message << "]";
    }
    const auto& previous = pre_step_snapshots_.at(env_id);
    if (previous.valid) {
      message << " previous={sim_time=" << previous.sim_time << ",pq=[";
      for (std::size_t i = 0; i < previous.pq.size(); ++i)
        message << (i == 0 ? "" : ",") << previous.pq[i];
      message << "],vs=[";
      for (std::size_t i = 0; i < previous.vs.size(); ++i)
        message << (i == 0 ? "" : ",") << previous.vs[i];
      message << "],mp=[";
      for (std::size_t i = 0; i < previous.mp.size(); ++i)
        message << (i == 0 ? "" : ",") << previous.mp[i];
      message << "],mv=[";
      for (std::size_t i = 0; i < previous.mv.size(); ++i)
        message << (i == 0 ? "" : ",") << previous.mv[i];
      message << "],torque=[";
      for (std::size_t i = 0; i < previous.torque.size(); ++i)
        message << (i == 0 ? "" : ",") << previous.torque[i];
      message << "]}";
    }
    return message.str();
  }

  auto throwErrors(const char* operation) const -> void {
    std::ostringstream message;
    bool has_error = false;
    for (const auto& error : errors_) {
      if (!error.empty()) {
        if (!has_error) {
          message << "SireRLBatchStepper " << operation << " failed";
          has_error = true;
        }
        message << "\n" << error;
      }
    }
    if (has_error) throw std::runtime_error(message.str());
  }

  py::sequence simulator_owners_;
  std::vector<sire::simulator::Simulator*> simulators_;
  std::vector<aris::dynamic::Model*> models_;
  std::vector<sire::simulator::SimulationLoop*> loops_;
  std::vector<sire::physics::PhysicsEngine*> engines_;
  std::size_t num_envs_{0};
  std::size_t num_actions_{0};
  std::size_t num_bodies_{0};
  std::size_t num_feet_{0};
  std::vector<int> motion_indices_;
  std::vector<int> foot_part_ids_;
  std::string control_type_;
  double action_scale_{1.0};
  double simulation_dt_{0.001};
  double control_dt_{0.01};
  std::vector<double> p_gains_;
  std::vector<double> d_gains_;
  std::vector<double> default_pos_;
  std::vector<double> torque_limits_;
  std::vector<double> dof_lower_;
  std::vector<double> dof_upper_;
  std::vector<double> previous_dof_vel_;
  std::vector<std::string> errors_;
  std::vector<std::string> recovered_errors_;
  std::vector<StepSnapshot> pre_step_snapshots_;
  std::vector<StepSnapshot> snapshot_scratch_;
  std::size_t total_recovered_failures_{0};
  ParallelExecutor executor_;

  py::array_t<float> root_states_;
  py::array_t<float> dof_pos_;
  py::array_t<float> dof_vel_;
  py::array_t<float> torques_;
  py::array_t<float> contact_forces_;
  py::array_t<float> feet_pos_;
  py::array_t<bool> body_ground_contact_;
  py::array_t<bool> foot_ground_contact_;
  py::array_t<double> dt_actual_;
  float* root_states_data_{nullptr};
  float* dof_pos_data_{nullptr};
  float* dof_vel_data_{nullptr};
  float* torques_data_{nullptr};
  float* contact_forces_data_{nullptr};
  float* feet_pos_data_{nullptr};
  bool* body_ground_contact_data_{nullptr};
  bool* foot_ground_contact_data_{nullptr};
  double* dt_actual_data_{nullptr};
};

}  // namespace

void init_rl(py::module& m) {
#if defined(SIRE_PROFILE_TRACY)
  m.attr("tracyEnabled") = true;
#else
  m.attr("tracyEnabled") = false;
#endif
  py::class_<SireRLBatchStepper>(m, "SireRLBatchStepper")
      .def(py::init<
               py::sequence,
               py::array_t<int, py::array::c_style | py::array::forcecast>,
               py::array_t<int, py::array::c_style | py::array::forcecast>,
               std::string, double,
               py::array_t<double,
                           py::array::c_style | py::array::forcecast>,
               py::array_t<double,
                           py::array::c_style | py::array::forcecast>,
               py::array_t<double,
                           py::array::c_style | py::array::forcecast>,
               py::array_t<double,
                           py::array::c_style | py::array::forcecast>,
               py::array_t<double,
                           py::array::c_style | py::array::forcecast>,
               py::array_t<double,
                           py::array::c_style | py::array::forcecast>,
               double, double, std::size_t>(),
           "simulators"_a, "motion_indices"_a, "foot_part_ids"_a,
           "control_type"_a, "action_scale"_a, "p_gains"_a,
           "d_gains"_a, "default_pos"_a, "torque_limits"_a,
           "dof_lower"_a, "dof_upper"_a, "simulation_dt"_a,
           "control_dt"_a,
           "sire_batch_threads"_a = 0)
      .def("step", &SireRLBatchStepper::step, "actions"_a)
      .def("reset", &SireRLBatchStepper::reset, "env_ids"_a)
      .def("resetRecorders", &SireRLBatchStepper::resetRecorders)
      .def("setHistoryRecording", &SireRLBatchStepper::setHistoryRecording,
           "enabled"_a)
      .def("outputs", &SireRLBatchStepper::outputs)
      .def_property_readonly("threadCount", &SireRLBatchStepper::threadCount)
      .def_property_readonly("workerCount", &SireRLBatchStepper::workerCount)
      .def_property_readonly("dispatchCount", &SireRLBatchStepper::dispatchCount)
      .def_property_readonly("numEnvs", &SireRLBatchStepper::numEnvs)
      .def_property_readonly("recoveredEnvIds",
                             &SireRLBatchStepper::recoveredEnvIds)
      .def_property_readonly("recoveredErrors",
                             &SireRLBatchStepper::recoveredErrors)
      .def_property_readonly("totalRecoveredFailures",
                             &SireRLBatchStepper::totalRecoveredFailures);
}
