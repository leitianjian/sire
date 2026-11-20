#pragma once

#include "rigid_body_dynamics.hpp"
#include "rigid_body_limits_model.hpp"

namespace sire::physics::contact::cuda {

struct RigidJointControl {
  std::size_t action;
  double stiffness, damping, default_position, action_scale;
};

/// Prepare motor effort from the pre-projection state, then apply the configured
/// hard joint limits. Limits and control are in motion order. Actions are
/// [batch, joint_count] in policy order, with nullptr selecting zero actions.
/// Positions, rates, and torques are [batch, joint_count] in motion order.
/// Configs, rebuild, and status are [batch]. Buffers reside on the device.
auto launchPrepareRigidJointControls(
    std::size_t batch, std::size_t joint_count, const RigidBodyJointLimits* limits,
    RigidBodyLimitSettings settings, const ContactHistoryConfig* configs,
    const float* actions, const RigidJointControl* control, double* positions,
    double* rates, double* motor_torques, bool* rebuild, RigidDynamicsStatus* status,
    cudaStream_t stream) -> cudaError_t;

/// Apply hard limits at the control boundary before publishing joint state.
/// Uses the same layouts and updates all successful environments.
auto launchProjectRigidJointLimits(
    std::size_t batch, std::size_t joint_count, const RigidBodyJointLimits* limits,
    RigidBodyLimitSettings settings, double* positions, double* rates, bool* rebuild,
    RigidDynamicsStatus* status, cudaStream_t stream) -> cudaError_t;

}  // namespace sire::physics::contact::cuda
