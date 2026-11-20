#pragma once

#include <cstddef>

#include <cuda_runtime_api.h>

namespace sire::physics::contact::cuda {

constexpr unsigned spectral_admm_problems_per_block = 2;

struct AdmmProblem {
  std::size_t contacts;
  std::size_t max_iterations;
  double step;
  double tolerance;
  bool has_force_guess;
  bool has_dual_guess;
};

enum class AdmmStatus { Converged, IterationLimit, InvalidInput, NumericalFailure };

struct AdmmResult {
  AdmmStatus status;
  std::size_t iterations;
  double primal_residual;
  double dual_residual;
  double complementarity_residual;
};

struct AdmmInputs {
  const AdmmProblem* problems;
  const double* friction;
  const double* inverse_mass;
  const double* velocity;
  const double* normal_target;
  const double* acceleration;
  const double* force_guess;
  const double* dual_guess;
};

struct AdmmOutputs {
  double* forces;
  double* dual;
  AdmmResult* results;
};

/// Scratch size in bytes. Use device memory aligned for double.
auto spectralAdmmWorkspaceSize(std::size_t batch_size, std::size_t capacity)
    -> std::size_t;

/// Solve Sire's shifted spectral ADMM with independent Coulomb cones.
/// Batch fields are device-resident. The friction and target arrays use
/// [batch, capacity], the row-major inverse mass [batch, 3*capacity, 3*capacity],
/// and each vector of contact coordinates [batch, 3*capacity]. Only the active
/// prefix is used. Contact forces and force guesses are in newtons. The dual
/// iterate is in the solver's impulse-space representation.
/// Converged and IterationLimit publish iterates. Invalid and failed solves
/// publish status only. A null dual output discards solved duals. Launch is
/// asynchronous on stream and inputs may be immediately reused on that stream
/// after the launch. Input, output and scratch buffers must be disjoint.
auto launchShiftedSpectralAdmm(std::size_t batch_size, std::size_t capacity,
                               AdmmInputs inputs, AdmmOutputs outputs,
                               void* workspace, std::size_t workspace_size,
                               cudaStream_t stream) -> cudaError_t;

}  // namespace sire::physics::contact::cuda
