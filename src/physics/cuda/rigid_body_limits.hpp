#pragma once

#include "rigid_body_dynamics.hpp"
#include "rigid_body_limits_model.hpp"

namespace sire::physics::contact::cuda {

/// Scratch size in doubles. Returns zero for an empty or unrepresentable shape.
auto jointLimitReactionWorkspaceSize(std::size_t batch, std::size_t joint_count) -> std::size_t;

/// Solve shifted NCP mechanical-stop reactions for each environment. The actual
/// step is configs[env].horizon. Positions, rates, detached_joint_acceleration,
/// and reactions are [batch, joint_count] in motor order. Mass factors use the
/// layout from launchRigidBodyDynamics. Reactions are signed generalized forces
/// added after the motor torque has been clamped.
///
/// All buffers are disjoint and device-resident. Existing failed environments
/// emit zero reactions. Disabled and Projection also emit zero reactions and
/// only require the output buffer.
auto launchJointLimitReactions(
    std::size_t batch, std::size_t joint_count, const RigidBodyJointLimits* limits,
    RigidBodyLimitSettings settings, double nominal_dt, const ContactHistoryConfig* configs,
    const double* positions, const double* rates, const double* detached_joint_acceleration,
    const double* mass_factor, RigidDynamicsStatus* status, double* reactions, double* scratch,
    cudaStream_t stream) -> cudaError_t;

}  // namespace sire::physics::contact::cuda
