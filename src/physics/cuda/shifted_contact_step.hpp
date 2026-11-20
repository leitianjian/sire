#pragma once

#include "contact_trajectory.hpp"
#include "spectral_admm.hpp"

namespace sire::physics::contact::cuda {

struct ContactMaterialState;

struct ShiftedContactInputs {
  // problems[i].step is the suggested interval. The standalone ADMM target is
  // derived from the trajectory at the interval accepted on the device.
  AdmmInputs admm;
  const double* normal_matrix;
  const double* normal_bias;
  const double* initial_normal_state;
  const ContactMaterialState* material{nullptr};  // Set by the body contact assembly path.
};

struct NormalContactResult {
  TrajectoryStatus status;
  double separation_time;
  double accepted_step;
  double spectral_rate;
  double spectral_frequency;
};

struct ShiftedContactOutputs {
  NormalContactResult* normal;
  double* normal_target;
  AdmmOutputs admm;
};

struct ShiftedContactScratch {
  AdmmProblem* accepted_problems;
  double* normal;
  std::size_t normal_size;  // doubles, from separationWorkspaceSize
  void* admm;
  std::size_t admm_size;  // bytes, from spectralAdmmWorkspaceSize
};

/// Evaluate a batched single-point contact step on stream. The normal ODE
/// uses padded row-major [batch, 2*capacity, 2*capacity] matrices and
/// [batch, 2*capacity] vectors (depths followed by their rates in the active
/// prefix). normal_target is [batch, capacity]. Scratch includes a
/// batch-sized buffer of accepted ADMM problems. All buffers are device
/// resident and disjoint. Per-problem failures are reported in the results.
auto launchShiftedContactStep(std::size_t batch_size, std::size_t capacity,
                              ShiftedContactInputs inputs,
                              ShiftedContactOutputs outputs,
                              ShiftedContactScratch scratch,
                              cudaStream_t stream) -> cudaError_t;

struct ContactMaterialState {
  double modified_depth;
  double stiffness;
  double damping;
  double friction;
};

struct BodyContactInputs {
  const AdmmProblem* problems;
  const ContactMaterialState* material;
  const double* inverse_inertia;
  const double* free_acceleration;
  const double* relative_velocity;
  const double* force_guess;
  const double* dual_guess;
  // Optional [batch, capacity] contact indices into the body-resolved arrays.
  // Material and relative velocity always use the compact active ordering.
  const std::size_t* body_indices{nullptr};
};

struct BodyContactAssembly {
  AdmmProblem* problems;
  double* friction;
  double* relative_inverse_inertia;
  double* relative_acceleration;
  double* normal_matrix;
  double* normal_bias;
  double* initial_normal_state;
};

/// Assemble and solve a single-point contact step from body-resolved inertia
/// and acceleration in each contact frame. The first three coordinates of
/// each six-coordinate block are body A, then body B. The row-major inertia
/// has shape [batch, 6*capacity, 6*capacity], free_acceleration [batch, 6*capacity],
/// material [batch, capacity], and the relative velocity and warm-start
/// vectors use the ADMM packing. Relative velocity is B minus A. All arrays
/// are device-resident and disjoint. Assembly writes scratch in the existing
/// normal/ADMM layouts. Failed assembly prevents that problem's solve.
auto launchBodyContactStep(std::size_t batch_size, std::size_t capacity,
                           BodyContactInputs inputs, ShiftedContactOutputs outputs,
                           BodyContactAssembly assembly, ShiftedContactScratch scratch,
                           cudaStream_t stream) -> cudaError_t;

}  // namespace sire::physics::contact::cuda
