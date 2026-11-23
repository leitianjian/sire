#pragma once

#include "rigid_body_dynamics.hpp"

namespace sire::physics::contact::cuda {

/// Semi-implicit Euler step followed by ARIS tree position and velocity projection.
/// order is [body_count-1] moving part IDs in parent-before-child order.
/// steps and status are [batch], poses are [batch, body, 12], and accelerations,
/// velocities, and corrections are [batch, body, 6]. All buffers are on the device.
/// A zero step or existing failure skips the environment. Reset failed environments
/// before continuing because a projection failure may leave partial updates.
auto launchIntegrateRigidBodyTree(
    std::size_t batch, RigidBodyTree model, const std::size_t* order,
    const double* steps, const double* accelerations, double* poses,
    double* velocities, RigidDynamicsStatus* status, double* corrections,
    cudaStream_t stream) -> cudaError_t;

}  // namespace sire::physics::contact::cuda
