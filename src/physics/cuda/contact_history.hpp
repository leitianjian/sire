#pragma once

#include <cstdint>

#include "shifted_contact_step.hpp"

namespace sire::physics::contact::cuda {

struct WorldContact {
  std::uint64_t geometry_a, geometry_b;
  std::size_t body_a, body_b;
  double point[3], normal[3];  // World point and A-to-B normal from narrow phase.
  double depth, stiffness, damping, friction;
};

struct DetectedContact {
  std::uint64_t geometry_a, geometry_b;
  std::size_t body_a, body_b;
  double frame[12];  // First three rows of the row-major contact pose.
  double depth, stiffness, damping, friction;
  double relative_velocity[3];  // Contact-frame velocity of B relative to A.
};

struct ContactHistoryConfig {
  std::size_t detected;
  double horizon;
  std::size_t max_iterations;
  double tolerance;
  bool retain_bounded_depth;
  bool warm_start;
  bool reset;
  bool suspended{false};
};

struct ContactPenetrationCache {
  std::uint64_t geometry_a, geometry_b;
  double baseline;
};

struct ContactForceCache {
  std::uint64_t geometry_a, geometry_b;
  double world_force[3];
};

struct ContactHistory {
  std::size_t* penetration_counts;
  ContactPenetrationCache* penetrations;
  std::size_t* force_counts;
  ContactForceCache* forces;
  double* dual;
};

struct PreparedContacts {
  AdmmProblem* problems;
  std::size_t* detected_indices;
  ContactMaterialState* material;
  double* velocity;
  double* force_guess;
  double* dual_guess;
};

/// Construct Sire's contact pose and relative point velocity from the narrow
/// phase output and the body state. World contacts and output detections are
/// [batch, capacity]. Spatial velocities are [batch, body_count, 6] in world
/// coordinates about the world origin [linear, angular]. Call before preparing
/// the contact history on the same stream. Invalid detections propagate to
/// the preparation result through the output.
auto launchFrameContacts(
    std::size_t batch, std::size_t capacity, std::size_t body_count,
    const ContactHistoryConfig* configs, const WorldContact* contacts,
    const double* body_velocities, DetectedContact* detections,
    cudaStream_t stream) -> cudaError_t;

/// Update penetration baselines, select active contacts stably and prepare
/// warm starts. Configs and count arrays are [batch]. Contacts, caches,
/// selected indices, temporary baselines and material are [batch, capacity].
/// Velocity, guesses and history.dual are [batch, 3*capacity]. Arrays are
/// disjoint and device resident. Zero both history count arrays before the
/// first call, or pass reset for each fresh environment. Invalid detections
/// publish an invalid problem and leave the prior history available for reset.
auto launchPrepareContactHistory(
    std::size_t batch, std::size_t capacity, const ContactHistoryConfig* configs,
    const DetectedContact* contacts, ContactHistory history,
    ContactPenetrationCache* temporary_baselines, PreparedContacts output,
    cudaStream_t stream) -> cudaError_t;

/// Complete the event after ADMM and store warm starts for the next event.
/// body_wrenches [batch, body_count, 6] are cleared per call, in world
/// coordinates about the world origin [force, moment]. world_forces, if
/// provided, is [batch, 3*capacity] in active contact order, force on B.
/// An iteration-limit result still carries a usable iterate, as on CPU.
/// Call on the same stream as preparation and the solver.
auto launchCommitContactHistory(
    std::size_t batch, std::size_t capacity, std::size_t body_count,
    const ContactHistoryConfig* configs, const DetectedContact* contacts,
    const AdmmProblem* problems, const std::size_t* detected_indices,
    AdmmOutputs solved, ContactHistory history, double* body_wrenches,
    double* world_forces, cudaStream_t stream,
    bool* ground_contacts = nullptr, std::size_t ground_body = 0) -> cudaError_t;

}  // namespace sire::physics::contact::cuda
