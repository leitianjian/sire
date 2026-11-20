#pragma once

#include "contact_history.hpp"
#include "rigid_body_model.hpp"

namespace sire::physics::contact::cuda {

enum class RigidDynamicsStatus { Success, InvalidInput, NonPositiveMass, NumericalFailure };

struct RigidBodyTree {
  const RigidBody* bodies;
  std::size_t body_count, joint_count, ground;
  double gravity[6];
};

struct RigidDynamicsInputs {
  const double* poses;
  const double* velocities;
  const double* torques;  // Optional; motor order from the model.
  const double* wrenches;  // Optional; net world wrench per body.
  const ContactHistoryConfig* configs{nullptr};  // Optional event suspension mask.
};

struct RigidDynamicsOutputs {
  double* accelerations;
  double* jacobians;
  double* mass_factor;
  RigidDynamicsStatus* status;
};

/// Sire's constrained dynamics for a floating-base tree of fixed/revolute
/// joints. Use the validated body records from loadRigidBodyModel. Poses are
/// [batch, body, 12]. Velocities, wrenches and accelerations are [batch, body,
/// 6] in world coordinates about the world origin. Torques are [batch,
/// joint_count]. Jacobians are [batch, body, 6, dof], with dof=6+joint_count
/// (base world twist, then motor order). mass_factor is the row-major lower
/// Cholesky factor [batch, dof, dof]. Ground rows are zero.
///
/// Scratch is [batch, body, 36+6*dof+6] doubles. Buffers are disjoint and
/// device-resident. An optional applied wrench at fixed pose can be solved
/// using the same factorization with launchApplyRigidBodyWrenches.
auto launchRigidBodyDynamics(
    std::size_t batch, RigidBodyTree model, RigidDynamicsInputs inputs,
    RigidDynamicsOutputs outputs, double* scratch, cudaStream_t stream) -> cudaError_t;

/// Add the response to applied world wrenches to the free accelerations
/// without repeating inertia assembly. Buffers use the layouts above.
/// generalized_scratch is [batch, dof]. Supports acceleration output
/// aliasing the free accelerations. Environments with a failed solve are
/// preserved so callers can apply their reset policy.
auto launchApplyRigidBodyWrenches(
    std::size_t batch, std::size_t body_count, std::size_t joint_count,
    const double* wrenches, const double* free_accelerations, const double* jacobians,
    const double* mass_factor, RigidDynamicsStatus* status, double* accelerations,
    double* generalized_scratch, cudaStream_t stream,
    const ContactHistoryConfig* configs = nullptr) -> cudaError_t;

/// Add signed generalized joint reactions to the accelerations in place.
/// Reuses the dynamics factor and the same [batch, dof] scratch.
auto launchApplyRigidJointReactions(
    std::size_t batch, std::size_t body_count, std::size_t joint_count,
    const double* reactions, const double* jacobians, const double* mass_factor,
    RigidDynamicsStatus* status, double* accelerations, double* generalized_scratch,
    const ContactHistoryConfig* configs, cudaStream_t stream) -> cudaError_t;

/// Publish body A/B point acceleration and J M^-1 J^T for compact active
/// contacts. selected_indices maps that active prefix to raw detections.
/// problems, detections, and selected_indices use the contact-history layout.
/// The output is directly accepted by launchBodyContactStep. contact_factor
/// is scratch [batch, 6*capacity, dof]. All arrays are device-resident.
auto launchRigidBodyContact(
    std::size_t batch, std::size_t body_count, std::size_t joint_count, std::size_t capacity,
    const double* velocities, const double* accelerations, const double* jacobians,
    const double* mass_factor, const RigidDynamicsStatus* dynamics_status,
    const DetectedContact* detections, const std::size_t* selected_indices, AdmmProblem* problems,
    double* inverse_inertia, double* free_acceleration, double* contact_factor,
    cudaStream_t stream) -> cudaError_t;

}  // namespace sire::physics::contact::cuda
