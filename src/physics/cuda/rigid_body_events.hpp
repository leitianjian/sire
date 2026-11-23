#pragma once

#include "flat_ground_collision.hpp"
#include "rigid_body_dynamics.hpp"

namespace sire::physics::contact::cuda {

struct RigidEventClock {
  double time{};
  double previous_control_time{};
  std::size_t event_count{};
  bool done{};
};

/// All arrays are device-resident with one entry per environment. Zero the
/// clocks when resetting simulation time. A failed environment remains done
/// until its status has been restored by the caller.
auto launchBeginRigidControl(std::size_t batch, RigidEventClock* clocks,
                            RigidDynamicsStatus* statuses, cudaStream_t stream) -> cudaError_t;

/// Set only the horizon and suspended flag in the existing contact configs.
auto launchPrepareRigidEvent(std::size_t batch, const RigidEventClock* clocks,
                            const RigidDynamicsStatus* statuses, double physics_dt,
                            double control_dt, ContactHistoryConfig* configs,
                            cudaStream_t stream) -> cudaError_t;

/// Publish zero steps for suspended or failed environments. Keep the earlier
/// failure status when it already reports a dynamics failure.
auto launchSelectRigidEventStep(
    std::size_t batch, const ContactHistoryConfig* configs,
    const GroundCollisionStatus* collision, const NormalContactResult* normal,
    const AdmmResult* admm, RigidDynamicsStatus* statuses, double* steps,
    cudaStream_t stream) -> cudaError_t;

/// Advance time after successful integration and complete the environment at
/// its control boundary. Time is unchanged on failure.
auto launchFinishRigidEvent(
    std::size_t batch, const ContactHistoryConfig* configs, const double* steps,
    double physics_dt, double control_dt, RigidEventClock* clocks,
    RigidDynamicsStatus* statuses, cudaStream_t stream) -> cudaError_t;

#if CUDART_VERSION >= 12030
/// Set the graph loop condition if any environment has work remaining. This
/// kernel must be launched in the graph that owns the conditional handle.
auto launchSetRigidEventGraphCondition(std::size_t batch, const RigidEventClock* clocks,
                                      cudaGraphConditionalHandle handle,
                                      cudaStream_t stream) -> cudaError_t;
#endif

}  // namespace sire::physics::contact::cuda
