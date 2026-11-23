#include "contact_history.hpp"

#include <cmath>
#include <limits>

namespace sire::physics::contact::cuda {
namespace {

constexpr unsigned threads_per_block = 128;

__device__ auto cross(const double* a, const double* b, double* c) -> void {
  c[0] = a[1] * b[2] - a[2] * b[1];
  c[1] = a[2] * b[0] - a[0] * b[2];
  c[2] = a[0] * b[1] - a[1] * b[0];
}

__device__ auto norm(const double* value) -> double {
  return ::sqrt(__dadd_rn(__dadd_rn(__dmul_rn(value[0], value[0]),
                                   __dmul_rn(value[1], value[1])),
                          __dmul_rn(value[2], value[2])));
}

__device__ auto makeFrame(const WorldContact& contact, double* frame) -> bool {
  double tx[3]{1.0, 0.0, 0.0}, ty[3]{0.0, 1.0, 0.0}, tz[3]{0.0, 0.0, 1.0};
  const double normal_size = norm(contact.normal);
  if (!::isfinite(normal_size)) return false;
  if (normal_size > 0.0) {
    const double scale = 1.0 / normal_size;
    for (int i = 0; i < 3; ++i) tz[i] = contact.normal[i] * scale;
    // ARIS s_sov_axes2pm("zx") receives the normal for both axes. Its
    // collinear fallback crosses z with each world axis and keeps the first
    // largest candidate as x, then uses z cross x as y.
    double longest = 0.0;
    for (int axis = 0; axis < 3; ++axis) {
      double world_axis[3]{};
      double candidate[3];
      world_axis[axis] = 1.0;
      cross(tz, world_axis, candidate);
      const double length = norm(candidate);
      if (length > longest) {
        longest = length;
        const double inv_length = 1.0 / length;
        for (int i = 0; i < 3; ++i) tx[i] = candidate[i] * inv_length;
      }
    }
    cross(tz, tx, ty);
  }
  for (int i = 0; i < 3; ++i) {
    frame[4 * i] = tx[i];
    frame[4 * i + 1] = ty[i];
    frame[4 * i + 2] = tz[i];
    frame[4 * i + 3] = contact.point[i];
  }
  return true;
}

__global__ void frameBatch(std::size_t batch, std::size_t capacity, std::size_t body_count,
                           const ContactHistoryConfig* configs, const WorldContact* contacts,
                           const double* body_velocities, DetectedContact* detections) {
  const auto env = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (env >= batch || configs[env].suspended || configs[env].detected > capacity) return;
  const auto* velocity = body_velocities + 6 * env * body_count;
  const auto offset = env * capacity;
  for (std::size_t i = 0; i < configs[env].detected; ++i) {
    const auto& contact = contacts[offset + i];
    auto& detection = detections[offset + i];
    detection = {contact.geometry_a, contact.geometry_b, contact.body_a, contact.body_b, {},
                 contact.depth, contact.stiffness, contact.damping, contact.friction, {}};
    if (contact.body_a >= body_count || contact.body_b >= body_count ||
        !makeFrame(contact, detection.frame)) {
      detection.depth = ::nan("");
      continue;
    }
    const auto* va = velocity + 6 * contact.body_a;
    const auto* vb = velocity + 6 * contact.body_b;
    double point_a[3], point_b[3];
    cross(va + 3, contact.point, point_a);
    cross(vb + 3, contact.point, point_b);
    for (int k = 0; k < 3; ++k) point_b[k] = (point_b[k] + vb[k]) - (point_a[k] + va[k]);
    for (int axis = 0; axis < 3; ++axis) {
      double local = 0.0;
      for (int k = 0; k < 3; ++k) local += detection.frame[4 * k + axis] * point_b[k];
      detection.relative_velocity[axis] = local;
    }
  }
}

__device__ auto samePair(std::uint64_t a, std::uint64_t b,
                         std::uint64_t previous_a, std::uint64_t previous_b) -> bool {
  return (a == previous_a && b == previous_b) || (a == previous_b && b == previous_a);
}

__device__ auto validDetections(const DetectedContact* contacts, std::size_t count) -> bool {
  for (std::size_t i = 0; i < count; ++i) {
    const auto& contact = contacts[i];
    if (!::isfinite(contact.depth)) return false;
    for (double value : contact.frame) if (!::isfinite(value)) return false;
    for (double value : contact.relative_velocity) if (!::isfinite(value)) return false;
    for (std::size_t j = 0; j < i; ++j) {
      if (samePair(contact.geometry_a, contact.geometry_b,
                   contacts[j].geometry_a, contacts[j].geometry_b)) return false;
    }
  }
  return true;
}

__global__ void prepareBatch(std::size_t batch, std::size_t capacity,
                             const ContactHistoryConfig* configs, const DetectedContact* contacts,
                             ContactHistory history, ContactPenetrationCache* scratch,
                             PreparedContacts output) {
  const auto env = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (env >= batch) return;
  const auto& config = configs[env];
  const auto offset = env * capacity;
  auto& problem = output.problems[env];
  problem = {0, config.max_iterations, 0.0, config.tolerance, false, false};
  if (config.suspended) return;
  const auto* detected = contacts + offset;
  const auto baseline_count = config.reset ? 0 : history.penetration_counts[env];
  const auto force_count = config.warm_start && !config.reset ? history.force_counts[env] : 0;
  if (config.detected > capacity || baseline_count > capacity || force_count > capacity ||
      !::isfinite(config.horizon) || config.horizon <= 0 || config.max_iterations == 0 ||
      !::isfinite(config.tolerance) || config.tolerance < 0 ||
      !validDetections(detected, config.detected)) return;

  auto* baselines = history.penetrations + offset;
  auto* next_baselines = scratch + offset;
  const auto* previous_forces = history.forces + offset;
  for (std::size_t i = 0; i < config.detected; ++i) {
    const auto& contact = detected[i];
    double baseline = config.retain_bounded_depth ? ::fmax(0.0, contact.depth - 1e-4) : contact.depth;
    for (std::size_t j = 0; j < baseline_count; ++j) {
      const auto& previous = baselines[j];
      if (samePair(contact.geometry_a, contact.geometry_b, previous.geometry_a, previous.geometry_b)) {
        baseline = previous.baseline;
        break;
      }
    }
    const double modified_depth = contact.depth - baseline;
    next_baselines[i] = {contact.geometry_a, contact.geometry_b,
                         modified_depth < 0.0 ? contact.depth : baseline};
    if (modified_depth < 0.0 || (modified_depth < 1e-8 && contact.relative_velocity[2] >= 0.0)) continue;

    const auto active = offset + problem.contacts++;
    output.detected_indices[active] = i;
    output.material[active] = {modified_depth, contact.stiffness, contact.damping, contact.friction};
    for (std::size_t k = 0; k < 3; ++k) {
      output.velocity[3 * active + k] = contact.relative_velocity[k];
      output.force_guess[3 * active + k] = 0.0;
    }
    for (std::size_t j = 0; j < force_count; ++j) {
      const auto& previous = previous_forces[j];
      if (!samePair(contact.geometry_a, contact.geometry_b, previous.geometry_a, previous.geometry_b)) continue;
      const double sign = contact.geometry_a == previous.geometry_a ? 1.0 : -1.0;
      // Single-point mode guarantees one prior force per geometry pair.
      for (std::size_t k = 0; k < 3; ++k) {
        double local = 0.0;
        for (std::size_t axis = 0; axis < 3; ++axis) {
          local += contact.frame[4 * axis + k] * (sign * previous.world_force[axis]);
        }
        output.force_guess[3 * active + k] = local;
      }
      problem.has_force_guess = true;
      break;
    }
  }
  problem.has_dual_guess = force_count != 0 && force_count == problem.contacts;
  if (problem.has_dual_guess) {
    // CPU Sire reuses duals by index when the solve dimension is unchanged.
    for (std::size_t j = 0; j < 3 * problem.contacts; ++j) {
      output.dual_guess[3 * offset + j] = history.dual[3 * offset + j];
    }
  }
  for (std::size_t i = 0; i < config.detected; ++i) baselines[i] = next_baselines[i];
  history.penetration_counts[env] = config.detected;
  if (config.reset || !config.warm_start || problem.contacts == 0) history.force_counts[env] = 0;
  problem.step = config.horizon;
}

__device__ auto contactWrench(const DetectedContact& contact, const double* local, double* wrench)
    -> void {
  for (std::size_t row = 0; row < 3; ++row) {
    const auto* axis = contact.frame + 4 * row;
    wrench[row] = axis[0] * local[0] + axis[1] * local[1] + axis[2] * local[2];
  }
  const double point[3]{contact.frame[3], contact.frame[7], contact.frame[11]};
  cross(point, wrench, wrench + 3);
}

__global__ void commitBatch(std::size_t batch, std::size_t capacity, std::size_t body_count,
                            const ContactHistoryConfig* configs, const DetectedContact* contacts,
                            const AdmmProblem* problems, const std::size_t* indices,
                            AdmmOutputs solved, ContactHistory history, double* body_wrenches,
                            double* world_forces, bool* ground_contacts, std::size_t ground_body) {
  const auto env = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (env >= batch) return;
  const auto& config = configs[env];
  if (config.suspended) return;
  const auto offset = env * capacity;
  const auto count = problems[env].contacts;
  auto& result = solved.results[env];
  auto* net = body_wrenches + env * 6 * body_count;
  auto* touching = ground_contacts ? ground_contacts + env * body_count : nullptr;
  for (std::size_t i = 0; i < 6 * body_count; ++i) net[i] = 0.0;
  if (touching) for (std::size_t i = 0; i < body_count; ++i) touching[i] = false;
  history.force_counts[env] = 0;
  if (count == 0 || (result.status != AdmmStatus::Converged && result.status != AdmmStatus::IterationLimit)) return;
  if (count > capacity || config.detected > capacity) {
    result.status = AdmmStatus::InvalidInput;
    return;
  }

  for (std::size_t i = 0; i < count; ++i) {
    const auto source = indices[offset + i];
    if (source >= config.detected) {
      result.status = AdmmStatus::InvalidInput;
      return;
    }
    const auto& contact = contacts[offset + source];
    if (contact.body_a >= body_count || contact.body_b >= body_count) {
      result.status = AdmmStatus::InvalidInput;
      return;
    }
  }
  for (std::size_t i = 0; i < count; ++i) {
    const auto active = offset + i;
    const auto& contact = contacts[offset + indices[active]];
    double wrench[6];
    contactWrench(contact, solved.forces + 3 * active, wrench);
    for (std::size_t k = 0; k < 6; ++k) {
      net[6 * contact.body_a + k] -= wrench[k];
      net[6 * contact.body_b + k] += wrench[k];
    }
    if (touching) {
      if (contact.body_a == ground_body && contact.body_b != ground_body) touching[contact.body_b] = true;
      if (contact.body_b == ground_body && contact.body_a != ground_body) touching[contact.body_a] = true;
    }
    if (world_forces) {
      for (std::size_t k = 0; k < 3; ++k) world_forces[3 * active + k] = wrench[k];
    }
    if (config.warm_start) {
      history.forces[active] = {contact.geometry_a, contact.geometry_b, {wrench[0], wrench[1], wrench[2]}};
      for (std::size_t k = 0; k < 3; ++k) history.dual[3 * active + k] = solved.dual[3 * active + k];
    }
  }
  for (std::size_t i = 0; i < 6 * body_count; ++i) {
    if (!::isfinite(net[i])) {
      result.status = AdmmStatus::NumericalFailure;
      for (std::size_t j = 0; j < 6 * body_count; ++j) net[j] = 0.0;
      if (touching) for (std::size_t j = 0; j < body_count; ++j) touching[j] = false;
      return;
    }
  }
  if (config.warm_start) history.force_counts[env] = count;
}

auto validLayout(std::size_t batch, std::size_t capacity) -> bool {
  constexpr auto max_bytes = std::numeric_limits<std::size_t>::max();
  return capacity != 0 && capacity <= max_bytes / sizeof(DetectedContact) / batch &&
         (batch - 1) / threads_per_block + 1 <= std::numeric_limits<int>::max();
}

auto validBodyLayout(std::size_t batch, std::size_t body_count) -> bool {
  constexpr auto max_bodies = std::numeric_limits<std::size_t>::max() / (6 * sizeof(double));
  return body_count != 0 && body_count <= max_bodies / batch;
}

}  // namespace

auto launchFrameContacts(
    std::size_t batch, std::size_t capacity, std::size_t body_count,
    const ContactHistoryConfig* configs, const WorldContact* contacts,
    const double* body_velocities, DetectedContact* detections,
    cudaStream_t stream) -> cudaError_t {
  if (batch == 0) return cudaSuccess;
  if (!validLayout(batch, capacity) || !validBodyLayout(batch, body_count) ||
      !configs || !contacts || !body_velocities || !detections) return cudaErrorInvalidValue;
  const auto blocks = (batch - 1) / threads_per_block + 1;
  frameBatch<<<static_cast<unsigned>(blocks), threads_per_block, 0, stream>>>(
      batch, capacity, body_count, configs, contacts, body_velocities, detections);
  return cudaGetLastError();
}

auto launchPrepareContactHistory(
    std::size_t batch, std::size_t capacity, const ContactHistoryConfig* configs,
    const DetectedContact* contacts, ContactHistory history,
    ContactPenetrationCache* temporary_baselines, PreparedContacts output,
    cudaStream_t stream) -> cudaError_t {
  if (batch == 0) return cudaSuccess;
  if (!validLayout(batch, capacity) || !configs || !contacts || !history.penetration_counts ||
      !history.penetrations || !history.force_counts || !history.forces || !history.dual ||
      !temporary_baselines || !output.problems || !output.detected_indices || !output.material ||
      !output.velocity || !output.force_guess || !output.dual_guess) return cudaErrorInvalidValue;
  const auto blocks = (batch - 1) / threads_per_block + 1;
  prepareBatch<<<static_cast<unsigned>(blocks), threads_per_block, 0, stream>>>(
      batch, capacity, configs, contacts, history, temporary_baselines, output);
  return cudaGetLastError();
}

auto launchCommitContactHistory(
    std::size_t batch, std::size_t capacity, std::size_t body_count,
    const ContactHistoryConfig* configs, const DetectedContact* contacts,
    const AdmmProblem* problems, const std::size_t* detected_indices,
    AdmmOutputs solved, ContactHistory history, double* body_wrenches,
    double* world_forces, cudaStream_t stream,
    bool* ground_contacts, std::size_t ground_body) -> cudaError_t {
  if (batch == 0) return cudaSuccess;
  if (!validLayout(batch, capacity) || !validBodyLayout(batch, body_count) ||
      (ground_contacts && ground_body >= body_count) ||
      !configs || !contacts || !problems || !detected_indices || !solved.forces || !solved.dual ||
      !solved.results || !history.force_counts || !history.forces || !history.dual || !body_wrenches) {
    return cudaErrorInvalidValue;
  }
  const auto blocks = (batch - 1) / threads_per_block + 1;
  commitBatch<<<static_cast<unsigned>(blocks), threads_per_block, 0, stream>>>(
      batch, capacity, body_count, configs, contacts, problems, detected_indices,
      solved, history, body_wrenches, world_forces, ground_contacts, ground_body);
  return cudaGetLastError();
}

}  // namespace sire::physics::contact::cuda
