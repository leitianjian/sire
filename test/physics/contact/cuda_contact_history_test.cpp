#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <aris/dynamic/kinematics.hpp>
#include <aris/dynamic/pose.hpp>
#include <aris/ext/json.hpp>

#include "contact_history.hpp"
#include "cuda_test_buffer.hpp"
#include "sire/core/force_screw.hpp"
#include "sire/physics/contact/simple_admm_contact_solver.hpp"

namespace {

using namespace sire::physics::contact::cuda;
using sire::physics::contact::simple_admm::cptContactForceShiftedSpectralAdmmWithWarmStart;
using sire::test::DeviceBuffer;
using sire::test::checkCuda;

struct Event {
  ContactHistoryConfig config{0, 0.001, 1000, 1e-8, false, true, false};
  std::vector<DetectedContact> detected;
  std::vector<WorldContact> world;
  std::vector<double> body_velocities;
  std::vector<std::size_t> active;
  std::vector<double> corrected, initial_force, initial_dual, force, dual;
  std::vector<double> body_inertia, body_acceleration;
  std::vector<double> friction, inverse, velocity, acceleration;
  double step{0.001}, residual{0};
};

struct Observations {
  std::vector<DetectedContact> detections;
  std::vector<AdmmProblem> problems;
  std::vector<AdmmResult> status;
  std::vector<NormalContactResult> normal;
  std::vector<std::size_t> indices;
  std::vector<ContactMaterialState> material;
  std::vector<double> guesses, dual_guesses, targets, force, dual, wrenches;
};

class Stream {
 public:
  Stream() { checkCuda(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking)); }
  ~Stream() { cudaStreamDestroy(stream_); }
  Stream(const Stream&) = delete;
  Stream& operator=(const Stream&) = delete;
  auto get() const -> cudaStream_t { return stream_; }

 private:
  cudaStream_t stream_{};
};

class Replay {
 public:
  Replay(std::size_t batch, std::size_t capacity, std::size_t body_count)
      : batch_(batch), capacity_(capacity), body_count_(body_count), slots_(batch * capacity),
        configs_(batch), detected_(slots_), world_(slots_), body_velocities_(6 * batch * body_count),
        penetration_counts_(batch), force_counts_(batch),
        baselines_(slots_), temporary_baselines_(slots_), previous_(slots_), previous_dual_(3 * slots_),
        problems_(batch), indices_(slots_), material_(slots_), velocity_(3 * slots_),
        guesses_(3 * slots_), dual_guesses_(3 * slots_), status_(batch), force_(3 * slots_),
        dual_(3 * slots_), wrenches_(6 * batch * body_count), normal_(batch), targets_(slots_),
        accepted_(batch), normal_work_(separationWorkspaceSize(batch, capacity)),
        admm_work_(spectralAdmmWorkspaceSize(batch, capacity)), inertia_(36 * slots_ * capacity),
        body_acc_(6 * slots_), assembled_(batch), friction_(slots_), relative_inertia_(9 * slots_ * capacity),
        relative_acc_(3 * slots_), matrix_(4 * slots_ * capacity), bias_(2 * slots_), initial_(2 * slots_) {
    const std::vector<std::size_t> empty(batch, 0);
    penetration_counts_.upload(empty);
    force_counts_.upload(empty);
  }

  auto step(const std::vector<Event>& events, bool live) -> Observations {
    if (events.size() != batch_) throw std::runtime_error("Wrong event batch size");
    std::vector<ContactHistoryConfig> configs;
    std::vector<DetectedContact> detected(slots_);
    for (std::size_t env = 0; env < batch_; ++env) {
      const auto& event = events[env];
      if (event.detected.size() > capacity_) throw std::runtime_error("Contact replay exceeds capacity");
      configs.push_back(event.config);
      configs.back().detected = event.detected.size();
      std::copy(event.detected.begin(), event.detected.end(), detected.begin() + env * capacity_);
    }
    configs_.upload(configs);
    if (live) {
      uploadBodyInputs(events);
      frameContacts(events);
    } else {
      detected_.upload(detected);
      uploadSolvedForces(events);
    }

    const ContactHistory history{penetration_counts_.data(), baselines_.data(), force_counts_.data(),
                                  previous_.data(), previous_dual_.data()};
    const AdmmOutputs solved{force_.data(), dual_.data(), status_.data()};
    checkCuda(launchPrepareContactHistory(batch_, capacity_, configs_.data(), detected_.data(), history,
        temporary_baselines_.data(), {problems_.data(), indices_.data(), material_.data(),
        velocity_.data(), guesses_.data(), dual_guesses_.data()}, stream_.get()));
    if (live) {
      checkCuda(launchBodyContactStep(batch_, capacity_,
          {problems_.data(), material_.data(), inertia_.data(), body_acc_.data(), velocity_.data(),
           guesses_.data(), dual_guesses_.data(), indices_.data()}, {normal_.data(), targets_.data(), solved},
          {assembled_.data(), friction_.data(), relative_inertia_.data(), relative_acc_.data(),
           matrix_.data(), bias_.data(), initial_.data()},
          {accepted_.data(), normal_work_.data(), separationWorkspaceSize(batch_, capacity_),
           admm_work_.data(), spectralAdmmWorkspaceSize(batch_, capacity_)}, stream_.get()));
    }
    checkCuda(launchCommitContactHistory(batch_, capacity_, body_count_, configs_.data(), detected_.data(),
        problems_.data(), indices_.data(), solved, history, wrenches_.data(), nullptr, stream_.get()));
    checkCuda(cudaStreamSynchronize(stream_.get()));
    return {detected_.download(), problems_.download(), status_.download(),
            live ? normal_.download() : std::vector<NormalContactResult>{},
            indices_.download(), material_.download(), guesses_.download(), dual_guesses_.download(),
            live ? targets_.download() : std::vector<double>{}, force_.download(), dual_.download(),
            wrenches_.download()};
  }

 private:
  void frameContacts(const std::vector<Event>& events) {
    std::vector<WorldContact> world(slots_);
    const auto stride = 6 * body_count_;
    std::vector<double> velocities(batch_ * stride);
    for (std::size_t env = 0; env < batch_; ++env) {
      const auto& event = events[env];
      if (event.world.size() != event.detected.size() || event.body_velocities.size() != stride) {
        throw std::runtime_error("Wrong world contact trace dimensions");
      }
      std::copy(event.world.begin(), event.world.end(), world.begin() + env * capacity_);
      std::copy(event.body_velocities.begin(), event.body_velocities.end(), velocities.begin() + env * stride);
    }
    world_.upload(world);
    body_velocities_.upload(velocities);
    checkCuda(launchFrameContacts(batch_, capacity_, body_count_, configs_.data(), world_.data(),
        body_velocities_.data(), detected_.data(), stream_.get()));
  }

  void uploadSolvedForces(const std::vector<Event>& events) {
    std::vector<AdmmResult> status(batch_);
    std::vector<double> force(3 * slots_), dual(3 * slots_);
    for (std::size_t env = 0; env < batch_; ++env) {
      const auto& event = events[env];
      const auto size = 3 * event.active.size();
      if (event.force.size() != size || event.dual.size() != size) throw std::runtime_error("Wrong prescribed solve size");
      const auto offset = 3 * env * capacity_;
      std::copy(event.force.begin(), event.force.end(), force.begin() + offset);
      std::copy(event.dual.begin(), event.dual.end(), dual.begin() + offset);
      status[env].status = event.residual > event.config.tolerance ? AdmmStatus::IterationLimit : AdmmStatus::Converged;
    }
    force_.upload(force);
    dual_.upload(dual);
    status_.upload(status);
  }

  void uploadBodyInputs(const std::vector<Event>& events) {
    const auto stride = 6 * capacity_;
    const double pad = std::numeric_limits<double>::quiet_NaN();
    std::vector<double> matrix(batch_ * stride * stride, pad), acceleration(batch_ * stride, pad);
    for (std::size_t env = 0; env < batch_; ++env) {
      const auto& event = events[env];
      const auto n = 6 * event.active.size();
      if (event.body_inertia.size() != n * n || event.body_acceleration.size() != n) {
        throw std::runtime_error("Wrong body-resolved trace dimensions");
      }
      for (std::size_t row = 0; row < n; ++row) {
        const auto raw_row = 6 * event.active[row / 6] + row % 6;
        acceleration[env * stride + raw_row] = event.body_acceleration[row];
        for (std::size_t col = 0; col < n; ++col) {
          const auto raw_col = 6 * event.active[col / 6] + col % 6;
          matrix[(env * stride + raw_row) * stride + raw_col] = event.body_inertia[row * n + col];
        }
      }
    }
    inertia_.upload(matrix);
    body_acc_.upload(acceleration);
  }

  const std::size_t batch_, capacity_, body_count_, slots_;
  Stream stream_;
  DeviceBuffer<ContactHistoryConfig> configs_;
  DeviceBuffer<DetectedContact> detected_;
  DeviceBuffer<WorldContact> world_;
  DeviceBuffer<double> body_velocities_;
  DeviceBuffer<std::size_t> penetration_counts_, force_counts_;
  DeviceBuffer<ContactPenetrationCache> baselines_, temporary_baselines_;
  DeviceBuffer<ContactForceCache> previous_;
  DeviceBuffer<double> previous_dual_;
  DeviceBuffer<AdmmProblem> problems_;
  DeviceBuffer<std::size_t> indices_;
  DeviceBuffer<ContactMaterialState> material_;
  DeviceBuffer<double> velocity_, guesses_, dual_guesses_;
  DeviceBuffer<AdmmResult> status_;
  DeviceBuffer<double> force_, dual_, wrenches_;
  DeviceBuffer<NormalContactResult> normal_;
  DeviceBuffer<double> targets_;
  DeviceBuffer<AdmmProblem> accepted_;
  DeviceBuffer<double> normal_work_;
  DeviceBuffer<unsigned char> admm_work_;
  DeviceBuffer<double> inertia_, body_acc_;
  DeviceBuffer<AdmmProblem> assembled_;
  DeviceBuffer<double> friction_, relative_inertia_, relative_acc_, matrix_, bias_, initial_;
};

void compare(double actual, double expected, const char* field, double absolute = 1e-12,
             double relative = 0) {
  if (std::isfinite(actual) && std::abs(actual - expected) <= absolute + relative * std::abs(expected)) {
    return;
  }
  std::ostringstream message;
  message << std::setprecision(17) << field << " is " << actual << ", expected " << expected;
  throw std::runtime_error(message.str());
}

auto pose(const DetectedContact& contact) -> std::array<double, 16> {
  std::array<double, 16> result{};
  std::copy_n(contact.frame, 12, result.begin());
  result[15] = 1;
  return result;
}

auto worldWrench(const DetectedContact& contact, const double* force) -> std::array<double, 6> {
  const auto pm = pose(contact);
  std::array<double, 6> result;
  sire::core::screw::s_fpm2fs(force, pm.data(), result.data());
  return result;
}

void checkPrepared(const Event& event, const Observations& obs, std::size_t env, std::size_t capacity,
                   const std::vector<double>& guess, const std::vector<double>& dual) {
  const auto& problem = obs.problems[env];
  if (problem.contacts != event.active.size() || problem.has_force_guess != !guess.empty() ||
      problem.has_dual_guess != !dual.empty()) {
    std::ostringstream message;
    message << "Incorrect prepared problem for environment " << env << " (contacts "
            << problem.contacts << "/" << event.active.size() << ", force guess "
            << problem.has_force_guess << "/" << !guess.empty() << ", dual guess "
            << problem.has_dual_guess << "/" << !dual.empty() << ")";
    throw std::runtime_error(message.str());
  }
  compare(problem.step, event.config.horizon, "prepared horizon");
  for (std::size_t i = 0; i < problem.contacts; ++i) {
    const auto index = env * capacity + i;
    if (obs.indices[index] != event.active[i]) throw std::runtime_error("Wrong active contact order");
    compare(obs.material[index].modified_depth, event.corrected[event.active[i]], "modified depth");
    for (std::size_t k = 0; k < 3; ++k) {
      compare(obs.guesses[3 * index + k], guess.empty() ? 0 : guess[3 * i + k], "force guess", 3e-12, 1e-13);
      if (!dual.empty()) compare(obs.dual_guesses[3 * index + k], dual[3 * i + k], "dual guess");
    }
  }
}

void checkWrenches(const Event& event, const Observations& obs, std::size_t env,
                  std::size_t body_count, const double* force) {
  std::vector<double> expected(6 * body_count);
  for (std::size_t i = 0; i < event.active.size(); ++i) {
    const auto& contact = event.detected[event.active[i]];
    const auto wrench = worldWrench(contact, force + 3 * i);
    for (std::size_t k = 0; k < wrench.size(); ++k) {
      expected[6 * contact.body_a + k] -= wrench[k];
      expected[6 * contact.body_b + k] += wrench[k];
    }
  }
  for (std::size_t k = 0; k < expected.size(); ++k) {
    compare(obs.wrenches[6 * body_count * env + k], expected[k], "world wrench", 3e-12, 1e-13);
  }
}

struct HistoryTrace {
  std::size_t body_count{0}, capacity{0};
  std::vector<Event> events;
};

auto readHistory(const std::filesystem::path& dir) -> HistoryTrace {
  using Json = nlohmann::json;
  std::ifstream file(dir / "single_point_contact_history.jsonl");
  if (!file) throw std::runtime_error("Cannot open history in " + dir.string());
  HistoryTrace trace;
  for (std::string line; std::getline(file, line);) {
    const auto row = Json::parse(line);
    const auto body_count = row.at("body_count").get<std::size_t>();
    if (trace.events.empty()) trace.body_count = body_count;
    if (body_count != trace.body_count) throw std::runtime_error("Body count changed during replay");
    Event event;
    event.config.horizon = row.at("horizon").get<double>();
    event.config.retain_bounded_depth = row.at("retain_bounded_depth").get<bool>();
    event.config.warm_start = row.at("warm_start").get<bool>();
    event.active = row.at("active").get<std::vector<std::size_t>>();
    for (const auto& spatial_velocity : row.at("body_velocities")) {
      const auto velocity = spatial_velocity.get<std::array<double, 6>>();
      event.body_velocities.insert(event.body_velocities.end(), velocity.begin(), velocity.end());
    }
    for (const auto& d : row.at("detected")) {
      DetectedContact contact{};
      contact.geometry_a = d.at("geometry_a").get<std::uint64_t>();
      contact.geometry_b = d.at("geometry_b").get<std::uint64_t>();
      contact.body_a = d.at("body_a").get<std::size_t>();
      contact.body_b = d.at("body_b").get<std::size_t>();
      if (contact.body_a >= body_count || contact.body_b >= body_count) {
        throw std::runtime_error("Body index exceeds trace size");
      }
      const auto pm = d.at("frame").get<std::array<double, 16>>();
      std::copy_n(pm.begin(), 12, contact.frame);
      const auto velocity = d.at("relative_velocity").get<std::array<double, 3>>();
      std::copy(velocity.begin(), velocity.end(), contact.relative_velocity);
      contact.depth = d.at("depth").get<double>();
      contact.stiffness = d.at("stiffness").get<double>();
      contact.damping = d.at("damping").get<double>();
      contact.friction = d.at("friction").get<double>();
      event.corrected.push_back(d.at("modified_depth").get<double>());
      event.detected.push_back(contact);
      WorldContact world{contact.geometry_a, contact.geometry_b, contact.body_a, contact.body_b, {},
                         {}, contact.depth, contact.stiffness, contact.damping, contact.friction};
      const auto point = d.at("point").get<std::array<double, 3>>();
      const auto normal = d.at("normal").get<std::array<double, 3>>();
      std::copy(point.begin(), point.end(), world.point);
      std::copy(normal.begin(), normal.end(), world.normal);
      event.world.push_back(world);
    }
    for (const auto index : event.active) {
      if (index >= event.detected.size()) throw std::runtime_error("Invalid active index in trace");
    }
    const auto& frame_id = row.at("solver_frame");
    event.step = event.config.horizon;
    if (!frame_id.is_null()) {
      std::ostringstream name;
      name << "frame_" << std::setw(8) << std::setfill('0') << frame_id.get<std::size_t>() << ".json";
      std::ifstream frame_file(dir / name.str());
      if (!frame_file) throw std::runtime_error("Missing " + name.str());
      Json frame;
      frame_file >> frame;
      const auto& admm = frame.at("shifted_spectral_admm");
      if (!admm.at("initial_force").is_null()) {
        event.initial_force = admm.at("initial_force").get<std::vector<double>>();
      }
      if (!admm.at("initial_dual").is_null()) {
        event.initial_dual = admm.at("initial_dual").get<std::vector<double>>();
      }
      event.config.max_iterations = frame.at("solver_max_iters").get<std::size_t>();
      event.config.tolerance = frame.at("solver_max_error").get<double>();
      event.step = frame.at("h").get<double>();
      event.residual = frame.at("solver_error").get<double>();
      event.force = frame.at("cResult").get<std::vector<double>>();
      event.dual = admm.at("final_dual").get<std::vector<double>>();
      event.friction = frame.at("fri_coef").get<std::vector<double>>();
      event.inverse = frame.at("invM").get<std::vector<double>>();
      event.velocity = frame.at("v0").get<std::vector<double>>();
      event.acceleration = frame.at("b").get<std::vector<double>>();
      const auto& body = frame.at("body_contact_inputs");
      event.body_inertia = body.at("inverse_inertia").get<std::vector<double>>();
      event.body_acceleration = body.at("free_acceleration").get<std::vector<double>>();
    }
    trace.capacity = std::max(trace.capacity, event.detected.size());
    trace.events.push_back(std::move(event));
  }
  if (!trace.capacity || !trace.body_count) throw std::runtime_error("History is missing contact events");
  return trace;
}

auto expectedForceGuess(const Event& event, const std::vector<ContactForceCache>& previous)
    -> std::vector<double> {
  const auto count = event.active.size();
  std::vector<double> expected(3 * count);
  bool matched = false;
  if (!event.config.warm_start || event.config.reset) return {};
  for (std::size_t i = 0; i < count; ++i) {
    const auto& contact = event.detected[event.active[i]];
    for (const auto& force : previous) {
      const bool same = force.geometry_a == contact.geometry_a && force.geometry_b == contact.geometry_b;
      const bool reverse = force.geometry_a == contact.geometry_b && force.geometry_b == contact.geometry_a;
      if (!same && !reverse) continue;
      const auto pm = pose(contact);
      std::array<double, 3> world;
      std::transform(force.world_force, force.world_force + 3, world.begin(),
                     [same](double f) { return same ? f : -f; });
      aris::dynamic::s_inv_pm_dot_v3(pm.data(), world.data(), expected.data() + 3 * i);
      matched = true;
      break;
    }
  }
  if (!matched) expected.clear();
  return expected;
}

void checkLive(Event& event, const Observations& obs, std::vector<ContactForceCache>& previous,
               std::vector<double>& previous_dual, std::size_t capacity, std::size_t body_count) {
  const auto count = event.active.size();
  for (std::size_t i = 0; i < event.detected.size(); ++i) {
    const auto& reference = event.detected[i];
    const auto& detected = obs.detections[i];
    for (std::size_t k = 0; k < 12; ++k) {
      compare(detected.frame[k], reference.frame[k], "contact frame");
    }
    for (std::size_t k = 0; k < 3; ++k) {
      compare(detected.relative_velocity[k], reference.relative_velocity[k], "relative contact velocity");
    }
  }
  const auto guess = expectedForceGuess(event, previous);
  const std::vector<double> empty;
  const auto& dual = event.config.warm_start && !event.config.reset && count == previous.size()
      ? previous_dual : empty;
  checkPrepared(event, obs, 0, capacity, guess, dual);
  const auto& normal = obs.normal[0];
  if (normal.status != TrajectoryStatus::Success) throw std::runtime_error("Live normal solve failed");
  compare(normal.accepted_step, event.step, "accepted step", 1e-8);
  const auto& result = obs.status[0];
  if (result.status != AdmmStatus::Converged && result.status != AdmmStatus::IterationLimit) {
    throw std::runtime_error("Live shifted ADMM failed");
  }
  checkWrenches(event, obs, 0, body_count, obs.force.data());
  if (count != 0) {
    std::vector<double> force(3 * count), final_dual;
    auto targets = std::vector<double>(obs.targets.begin(), obs.targets.begin() + count);
    const double residual = cptContactForceShiftedSpectralAdmmWithWarmStart(
        count, event.friction, event.inverse, event.velocity, targets, event.acceleration,
        normal.accepted_step, force, guess.empty() ? nullptr : &guess, dual.empty() ? nullptr : &dual,
        &final_dual, event.config.max_iterations, event.config.tolerance);
    compare(std::max({result.primal_residual, result.dual_residual, result.complementarity_residual}),
            residual, "live residual", 1e-9, 0.05);
    for (std::size_t i = 0; i < 3 * count; ++i) {
      compare(obs.force[i], force[i], "live contact force", 1e-6, 2e-8);
      compare(obs.dual[i], final_dual[i], "live contact dual", 1e-8, 1e-7);
    }
  }
  previous.clear();
  previous_dual.clear();
  if (event.config.warm_start) {
    for (std::size_t i = 0; i < count; ++i) {
      const auto& contact = event.detected[event.active[i]];
      const auto wrench = worldWrench(contact, obs.force.data() + 3 * i);
      previous.push_back({contact.geometry_a, contact.geometry_b, {wrench[0], wrench[1], wrench[2]}});
    }
    previous_dual.assign(obs.dual.begin(), obs.dual.begin() + 3 * count);
  }
}

void replayGo2(const std::filesystem::path& directory) {
  auto trace = readHistory(directory);
  Replay prescribed(1, trace.capacity, trace.body_count), live(1, trace.capacity, trace.body_count);
  std::vector<ContactForceCache> previous;
  std::vector<double> previous_dual;
  std::size_t active = 0;
  for (std::size_t i = 0; i < trace.events.size(); ++i) {
    auto& event = trace.events[i];
    try {
      const auto replayed = prescribed.step({event}, false);
      checkPrepared(event, replayed, 0, trace.capacity, event.initial_force, event.initial_dual);
      checkWrenches(event, replayed, 0, trace.body_count, event.force.data());
      checkLive(event, live.step({event}, true), previous, previous_dual, trace.capacity, trace.body_count);
    } catch (const std::exception& error) {
      throw std::runtime_error("Go2 event " + std::to_string(i) + ": " + error.what());
    }
    if (!event.active.empty()) ++active;
  }
  std::cout << "Validated sequential warm starts and solves for " << trace.events.size()
            << " Go2 events (" << active << " with active contacts)\n";
}

void frameBasisCases() {
  const std::vector<std::array<double, 3>> normals{
      {0, 0, 1}, {0, 0, -1}, {0, -1, 0}, {1, 0, 0},
      {1, 1, 1}, {-1, 1, 1}, {0.7, -0.4, 1.3}, {0, 0, 0}};
  const std::vector<double> velocity{0.4, 0.6, 0.3, -0.7, 0.5, 1.3,
                                      -0.7, -0.8, 0.4, 0.5, 0.1, 0.2};
  const auto count = normals.size();
  std::vector<WorldContact> contacts;
  for (const auto& normal : normals) {
    contacts.push_back({0, contacts.size() + 1, 0, 1, {-0.3, 0.37, 0.24},
                        {normal[0], normal[1], normal[2]}, 1e-4, 2e8, 5e3, 0.4});
  }
  DeviceBuffer<ContactHistoryConfig> config(1);
  config.upload({{count, 0.001, 1000, 1e-8, false, true, false}});
  DeviceBuffer<WorldContact> world(count);
  world.upload(contacts);
  DeviceBuffer<double> body_velocity(velocity.size());
  body_velocity.upload(velocity);
  DeviceBuffer<DetectedContact> detections(count);
  Stream stream;
  checkCuda(launchFrameContacts(1, count, 2, config.data(), world.data(), body_velocity.data(),
                                detections.data(), stream.get()));
  checkCuda(cudaStreamSynchronize(stream.get()));
  const auto output = detections.download();
  for (std::size_t i = 0; i < count; ++i) {
    const auto& contact = contacts[i];
    double frame[16], point_a[3], point_b[3], local[3];
    aris::dynamic::s_sov_axes2pm(contact.point, contact.normal, contact.normal, frame, "zx");
    aris::dynamic::s_vs2vp(velocity.data(), contact.point, point_a);
    aris::dynamic::s_vs2vp(velocity.data() + 6, contact.point, point_b);
    aris::dynamic::s_vs(3, point_a, point_b);
    aris::dynamic::s_inv_pm_dot_v3(frame, point_b, local);
    for (std::size_t k = 0; k < 12; ++k) compare(output[i].frame[k], frame[k], "ARIS contact frame");
    for (std::size_t k = 0; k < 3; ++k) {
      compare(output[i].relative_velocity[k], local[k], "ARIS local contact velocity");
    }
  }
}

void syntheticHistory() {
  constexpr std::size_t batch = 1024, capacity = 2, bodies = 3;
  const DetectedContact contact{1, 2, 0, 1, {1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0},
                                1e-3, 2e8, 5e3, 0.4, {0, 0, -1}};
  auto reversed = contact;
  std::swap(reversed.geometry_a, reversed.geometry_b);
  std::swap(reversed.body_a, reversed.body_b);
  reversed.frame[0] = -1;
  reversed.frame[10] = -1;
  auto standby = contact;
  standby.geometry_b = 3;
  standby.body_b = 2;
  standby.relative_velocity[2] = 1;

  Replay replay(batch, capacity, bodies);
  Event event;
  event.detected = {contact};
  event.active = {0};
  event.corrected = {0};
  event.force = {2, 3, 4};
  event.dual = {1, 2, 3};
  const auto run = [&](const std::vector<Event>& events) {
    const auto obs = replay.step(events, false);
    for (std::size_t env = 0; env < batch; ++env) {
      const auto& input = events[env];
      checkPrepared(input, obs, env, capacity, input.initial_force, input.initial_dual);
      checkWrenches(input, obs, env, bodies, input.force.data());
    }
  };
  run(std::vector<Event>(batch, event));

  event.detected = {standby, reversed};
  event.active = {1};
  event.corrected = {0, 0};
  event.initial_force = {2, -3, 4};
  event.initial_dual = event.dual;
  run(std::vector<Event>(batch, event));

  auto lower = contact;
  lower.depth = 5e-4;
  Event inactive;
  inactive.detected = {lower};
  inactive.corrected = {-5e-4};
  run(std::vector<Event>(batch, inactive));

  auto restored = contact;
  restored.depth = 6e-4;
  event.detected = {restored};
  event.active = {0};
  event.corrected = {1e-4};
  event.initial_force.clear();
  event.initial_dual.clear();
  run(std::vector<Event>(batch, event));

  event.detected[0].depth = 7e-4;
  event.corrected = {2e-4};
  event.initial_force = event.force;
  event.initial_dual = event.dual;
  std::vector<Event> reset(batch, event);
  for (std::size_t env = 0; env < batch; env += 2) {
    reset[env].config.reset = true;
    reset[env].config.retain_bounded_depth = env % 4 == 0;
    reset[env].corrected = {reset[env].config.retain_bounded_depth ? 1e-4 : 0};
    reset[env].initial_force.clear();
    reset[env].initial_dual.clear();
  }
  run(reset);
  std::cout << "Validated independent contact history across " << batch << " CUDA environments\n";
}

}  // namespace

int main(int argc, char** argv) {
  int devices = 0;
  if (cudaGetDeviceCount(&devices) != cudaSuccess || devices == 0) return 77;
  try {
    if (argc > 2) throw std::runtime_error("Supply at most one trajectory directory");
    frameBasisCases();
    syntheticHistory();
    if (argc == 2) replayGo2(argv[1]);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
