#pragma once

#include "rigid_body_dynamics.hpp"

namespace sire::physics::contact::cuda {

/// Read revolute joint state in motor order as [batch, joint_count]. Poses
/// are [batch, body_count, 12]; velocities and accelerations are world spatial
/// vectors [batch, body_count, 6]. Pass both acceleration buffers or neither.
/// Use a validated floating tree with the ARIS revolute range centered at zero.
/// An optional event array skips suspended environments. When projected is
/// provided, refresh positions and rates only for environments rebuilt after
/// the previous read. Both optional arrays have batch entries.
auto launchReadRigidJointState(
    std::size_t batch, RigidBodyTree model, const double* poses, const double* velocities,
    const double* accelerations, double* positions, double* rates, double* joint_accelerations,
    cudaStream_t stream, const ContactHistoryConfig* events = nullptr,
    const bool* projected = nullptr) -> cudaError_t;

/// Rebuild descendants in the validated model's parent-first order. order has
/// body_count - 1 entries, starting with the root and excluding ground. A true
/// mask entry selects an environment. Pass null to rebuild every environment.
/// The input root and ground poses and velocities are preserved. Input joint
/// position and rate buffers use [batch, joint_count], in motor order.
auto launchRebuildRigidBodyState(
    std::size_t batch, RigidBodyTree model, const std::size_t* order, const bool* mask,
    const double* positions, const double* rates, double* poses, double* velocities,
    cudaStream_t stream) -> cudaError_t;

}  // namespace sire::physics::contact::cuda
