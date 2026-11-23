#pragma once

#include <cstddef>

#include <cuda_runtime_api.h>

namespace sire::physics::contact::cuda {

enum class TrajectoryStatus { Success, InvalidInput, NumericalFailure };

/// Scratch size in doubles, or zero for an empty batch or invalid dimensions.
auto trajectoryWorkspaceSize(std::size_t batch_size, std::size_t contact_count)
    -> std::size_t;

/// Advance a batch of affine normal-contact trajectories at requested times.
/// Group problems by contact count N. The packed, device-resident arrays have
/// shapes [batch, 2N, 2N] for row-major A, [batch, 2N] for b, x0 and state, and
/// [batch] for time and status. The state order is depths followed by depth
/// rates. All arrays and scratch must be disjoint and live through execution.
/// Launch is asynchronous on stream. A failed problem leaves its state intact
/// and reports the reason through status. Other problems still execute.
auto launchContactTrajectories(
    std::size_t batch_size, std::size_t contact_count, const double* matrix,
    const double* bias, const double* initial_state, const double* time,
    double* state, TrajectoryStatus* status, double* workspace,
    std::size_t workspace_size, cudaStream_t stream) -> cudaError_t;

struct SeparationSearch {
  double horizon;
  double spectral_rate;
  double spectral_frequency;
};

auto separationWorkspaceSize(std::size_t batch_size, std::size_t contact_count)
    -> std::size_t;

/// Execute the exponential separation scan and refinement on the given
/// stream. A, b and x0 use the same packing as launchContactTrajectories.
/// Per-problem search parameters supply the horizon and spectral limits. The
/// result is the separating end of the final bracket, or -1 if none was found.
/// Failed problems leave separation_time unchanged and set their status.
auto launchContactSeparations(
    std::size_t batch_size, std::size_t contact_count, const double* matrix,
    const double* bias, const double* initial_state,
    const SeparationSearch* search, double* separation_time,
    TrajectoryStatus* status, double* workspace, std::size_t workspace_size,
    cudaStream_t stream) -> cudaError_t;

}  // namespace sire::physics::contact::cuda
