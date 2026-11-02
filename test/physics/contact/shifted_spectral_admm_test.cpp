#include <cmath>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "sire/physics/contact/simple_admm_contact_solver.hpp"

namespace {
using namespace sire::physics::contact::simple_admm;

struct ContactCase {
  std::vector<double> mu{0.5};
  std::vector<double> inverse_mass{-1, 0, 0, 0, -1, 0, 0, 0, -1};
  std::vector<double> velocity{2, 0, -1};
  std::vector<double> target{-0.5};
  std::vector<double> external{0, 0, 0};
  std::vector<double> force{0, 0, 0};
  double h = 0.01;

  auto solve(bool shifted = true) -> double {
    const auto solver = shifted ? cptContactForceShiftedSpectralAdmm
                                : cptContactForceSpectralAdmm;
    return solver(1, mu, inverse_mass, velocity, target, external, h, force,
                  2000, 1e-10);
  }
};

class WarmStartHarness : public SimpleAdmmContactSolver {
 public:
  using SimpleAdmmContactSolver::commitContactForceSolution;
  using SimpleAdmmContactSolver::prepareContactForceInitialGuess;
  using SimpleAdmmContactSolver::solveContactForceQP;
};

class PatchWarmStartHarness : public ShiftedSpectralAdmmContactSolver {
 public:
  using ShiftedSpectralAdmmContactSolver::commitContactForceSolution;
  using ShiftedSpectralAdmmContactSolver::prepareContactForceInitialGuess;
  using ShiftedSpectralAdmmContactSolver::solveContactForceQP;
  using ShiftedSpectralAdmmContactSolver::updatePatchInitialDepths;
};

auto patchPair(std::uint64_t contact_id, double depth)
    -> sire::physics::common::PenetrationAsPointPair {
  sire::physics::common::PenetrationAsPointPair pair;
  pair.contact_id = contact_id;
  pair.patch_id = 17;
  pair.depth = depth;
  return pair;
}

TEST(ShiftedSpectralAdmm, PatchHasIndependentNormalsAndCentroidFriction) {
  PatchWarmStartHarness solver;
  solver.setContactModelMode("height_field_patch");
  std::vector<sire::physics::common::PenetrationAsPointPair> pairs(2);
  for (int i = 0; i < 2; ++i) {
    pairs[i].id_A = 1;
    pairs[i].id_B = 2;
    pairs[i].contact_id = i + 1;
    pairs[i].patch_id = 7;
    pairs[i].p_WC = {0.1 * i, 0, 0};
  }
  const std::array<double, 16> identity{1, 0, 0, 0, 0, 1, 0, 0,
                                        0, 0, 1, 0, 0, 0, 0, 1};
  std::vector<std::array<double, 16>> frames(2, identity);
  std::vector<sire::Size> indices{0, 1};
  std::vector<double> force(6, 0), friction{0.5, 0.5},
      inverse_mass(36, 0), velocity{1, 0, -1, 1, 0, -1},
      target{0, 0}, external(6, 0);
  for (int i = 0; i < 6; ++i) inverse_mass[6 * i + i] = -1;
  solver.prepareContactForceInitialGuess(pairs, frames, indices, 0.01, force);
  const double error = solver.solveContactForceQP(
      2, friction, inverse_mass, velocity, target, external, 0.01, force,
      2000, 1e-9);
  EXPECT_TRUE(std::isfinite(error));
  EXPECT_GT(force[2], 0);
  EXPECT_GT(force[5], 0);
  EXPECT_NEAR(force[0], force[3], 1e-9);
  EXPECT_NEAR(force[1], force[4], 1e-9);
  EXPECT_LE(std::hypot(force[0] + force[3], force[1] + force[4]),
            friction[0] * (force[2] + force[5]) + 1e-8);

  // Equal tangential forces at supports x=0 and x=0.1 have zero moment about
  // the patch centroid x=0.05.
  const double tangent_moment_about_centroid =
      (-0.05) * force[1] + 0.05 * force[4];
  EXPECT_NEAR(tangent_moment_about_centroid, 0.0, 1e-10);

  // Reordering support metadata must not change the centroid distribution.
  std::swap(pairs[0], pairs[1]);
  solver.prepareContactForceInitialGuess(pairs, frames, indices, 0.01, force);
  solver.solveContactForceQP(2, friction, inverse_mass, velocity, target,
                             external, 0.01, force, 2000, 1e-9);
  EXPECT_NEAR(force[0], force[3], 1e-9);
  EXPECT_NEAR(force[1], force[4], 1e-9);
  EXPECT_GT(std::hypot(force[0] + force[3], force[1] + force[4]), 1e-6);
}

TEST(ShiftedSpectralAdmm, PrismModeUsesPatchDepthsAndSharedFrictionPipeline) {
  PatchWarmStartHarness solver;
  solver.setContactModelMode("height_field_prism");
  EXPECT_EQ(solver.heightFieldPrismMaxContacts(), 50);
  solver.setHeightFieldPrismMaxContacts(4096);
  EXPECT_EQ(solver.heightFieldPrismMaxContacts(), 4096);
  EXPECT_THROW(solver.setHeightFieldPrismMaxContacts(0), std::invalid_argument);
  EXPECT_THROW(solver.setHeightFieldPrismMaxContacts(-1), std::invalid_argument);
  solver.setInitialPenetrationPolicy("eliminate");
  EXPECT_TRUE(solver.patchContactMode());
  EXPECT_FALSE(solver.singlePointContactMode());
  auto pairs = std::vector<sire::physics::common::PenetrationAsPointPair>{
      patchPair(31, 0.001), patchPair(32, 0.002)};
  solver.updatePatchInitialDepths(pairs);
  EXPECT_NEAR(pairs[0].modifiedDepth, 0, 1e-12);
  EXPECT_NEAR(pairs[1].modifiedDepth, 0, 1e-12);
  pairs[0].depth += 0.0001;
  pairs[1].depth += 0.0002;
  solver.updatePatchInitialDepths(pairs);
  EXPECT_NEAR(pairs[0].modifiedDepth, 0.0001, 1e-12);
  EXPECT_NEAR(pairs[1].modifiedDepth, 0.0002, 1e-12);
}

TEST(ShiftedSpectralAdmm, ZeroFrictionPatchHasNoTangentialForce) {
  PatchWarmStartHarness solver;
  solver.setContactModelMode("height_field_patch");
  std::vector<sire::physics::common::PenetrationAsPointPair> pairs(2);
  for (int i = 0; i < 2; ++i) {
    pairs[i].contact_id = i + 1;
    pairs[i].patch_id = 9;
    pairs[i].p_WC = {0.1 * i, 0, 0};
  }
  const std::array<double, 16> identity{1, 0, 0, 0, 0, 1, 0, 0,
                                        0, 0, 1, 0, 0, 0, 0, 1};
  std::vector<std::array<double, 16>> frames(2, identity);
  std::vector<sire::Size> indices{0, 1};
  std::vector<double> force(6, 0), friction{0, 0}, inverse_mass(36, 0),
      velocity{3, -2, -1, 3, -2, -1}, target{0, 0}, external(6, 0);
  for (int i = 0; i < 6; ++i) inverse_mass[6 * i + i] = -1;
  solver.prepareContactForceInitialGuess(pairs, frames, indices, 0.01, force);
  ASSERT_LE(solver.solveContactForceQP(
                2, friction, inverse_mass, velocity, target, external, 0.01,
                force, 2000, 1e-9),
            1e-9);
  EXPECT_NEAR(force[0], 0, 1e-10);
  EXPECT_NEAR(force[1], 0, 1e-10);
  EXPECT_NEAR(force[3], 0, 1e-10);
  EXPECT_NEAR(force[4], 0, 1e-10);
  EXPECT_GT(force[2] + force[5], 0);
}

TEST(ShiftedSpectralAdmm, DefaultsToSinglePointWithoutChangingBaselineDefault) {
  ShiftedSpectralAdmmContactSolver shifted;
  SimpleAdmmContactSolver baseline;
  EXPECT_EQ(shifted.contactModelMode(), "single_point");
  EXPECT_EQ(shifted.initialPenetrationPolicy(), "eliminate");
  EXPECT_EQ(baseline.contactModelMode(), "height_field");
  EXPECT_EQ(baseline.initialPenetrationPolicy(), "retain_bounded");

  baseline.setContactModelMode("height_field_patch");
  EXPECT_TRUE(baseline.patchContactMode());
  sire::physics::contact::ps_vs_solver_v5::PsVsSolverV5 v5;
  EXPECT_THROW(v5.setContactModelMode("height_field_patch"),
               std::invalid_argument);

  baseline.setInitialPenetrationPolicy("eliminate");
  EXPECT_EQ(baseline.initialPenetrationPolicy(), "eliminate");
  EXPECT_THROW(baseline.setInitialPenetrationPolicy("invalid"),
               std::invalid_argument);
}

TEST(ShiftedSpectralAdmm, PatchEliminatesInitialDepthPerPersistentContact) {
  PatchWarmStartHarness solver;
  solver.setContactModelMode("height_field_patch");
  ASSERT_EQ(solver.initialPenetrationPolicy(), "eliminate");

  std::vector<sire::physics::common::PenetrationAsPointPair> pairs{
      patchPair(101, 0.01), patchPair(102, 0.02)};
  solver.updatePatchInitialDepths(pairs);
  EXPECT_DOUBLE_EQ(pairs[0].modifiedDepth, 0.0);
  EXPECT_DOUBLE_EQ(pairs[1].modifiedDepth, 0.0);

  // Persistent supports retain independent baselines. A shallower support is
  // filtered once and ratchets its baseline down without affecting its peer.
  pairs = {patchPair(101, 0.0102), patchPair(102, 0.0195)};
  solver.updatePatchInitialDepths(pairs);
  EXPECT_NEAR(pairs[0].modifiedDepth, 0.0002, 1e-15);
  EXPECT_NEAR(pairs[1].modifiedDepth, -0.0005, 1e-15);
  pairs = {patchPair(101, 0.0102), patchPair(102, 0.0195)};
  solver.updatePatchInitialDepths(pairs);
  EXPECT_NEAR(pairs[0].modifiedDepth, 0.0002, 1e-15);
  EXPECT_DOUBLE_EQ(pairs[1].modifiedDepth, 0.0);

  // Once an ID disappears it is a new impact if it later returns.
  pairs = {patchPair(102, 0.0195)};
  solver.updatePatchInitialDepths(pairs);
  pairs = {patchPair(101, 0.03), patchPair(102, 0.0195)};
  solver.updatePatchInitialDepths(pairs);
  EXPECT_DOUBLE_EQ(pairs[0].modifiedDepth, 0.0);
}

TEST(ShiftedSpectralAdmm, PatchRetainsOnlyBoundedInitialCompression) {
  PatchWarmStartHarness solver;
  solver.setContactModelMode("height_field_patch");
  solver.setInitialPenetrationPolicy("retain_bounded");
  std::vector<sire::physics::common::PenetrationAsPointPair> pairs{
      patchPair(201, 0.01), patchPair(202, 5e-5)};
  solver.updatePatchInitialDepths(pairs);
  EXPECT_NEAR(pairs[0].modifiedDepth, 1e-4, 1e-15);
  EXPECT_NEAR(pairs[1].modifiedDepth, 5e-5, 1e-15);
}

TEST(ShiftedSpectralAdmm, ZeroTargetMatchesUnshiftedIteration) {
  ContactCase shifted;
  shifted.target[0] = 0;
  shifted.external = {0.5, 0, -0.25};
  ContactCase baseline = shifted;
  // The paper baseline must continue to ignore even a nonzero target.
  baseline.target[0] = 7;
  EXPECT_DOUBLE_EQ(shifted.solve(), baseline.solve(false));
  EXPECT_EQ(shifted.force, baseline.force);
}

TEST(ShiftedSpectralAdmm, UsesSimpleReferenceTau) {
  ContactCase problem;
  problem.mu[0] = 0;
  problem.velocity = {0, 0, -1};
  problem.target[0] = 0;

  constexpr double eta = 1e-6;
  constexpr double tau = 0.5;
  const double rho = std::sqrt(eta) * std::pow(1.0 / eta, 0.2);
  const double expected_impulse = 1.0 / (1.0 + eta + tau * rho);
  // A converged solve is independent of tau. Re-run a single iteration to
  // expose the augmented-Lagrangian scaling used by the reference code.
  problem.force.assign(3, 0.0);
  cptContactForceSpectralAdmm(
      1, problem.mu, problem.inverse_mass, problem.velocity, problem.target,
      problem.external, problem.h, problem.force, 1, 0.0);
  EXPECT_NEAR(problem.h * problem.force[2], expected_impulse, 1e-12);
}

TEST(ShiftedSpectralAdmm, WarmStartsPersistentContactByDefault) {
  WarmStartHarness solver;
  EXPECT_TRUE(solver.warmStartEnabled());

  std::vector<sire::physics::common::PenetrationAsPointPair> pairs(1);
  pairs[0].id_A = 11;
  pairs[0].id_B = 17;
  pairs[0].p_WC[0] = 0.1;
  pairs[0].p_WC[1] = -0.2;
  pairs[0].p_WC[2] = 0.3;
  std::vector<std::array<double, 16>> frames(1);
  frames[0] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  std::vector<sire::Size> indices{0};

  ContactCase problem;
  ASSERT_LE(solver.solveContactForceQP(1, problem.mu, problem.inverse_mass,
                                       problem.velocity, problem.target,
                                       problem.external, problem.h,
                                       problem.force, 2000, 1e-10),
            1e-9);
  const auto converged_force = problem.force;
  solver.commitContactForceSolution(pairs, frames, indices, problem.h,
                                    problem.force);

  std::vector<double> guess(3, 0.0);
  solver.prepareContactForceInitialGuess(pairs, frames, indices, problem.h,
                                         guess);
  for (int axis = 0; axis < 3; ++axis) {
    EXPECT_NEAR(guess[axis], converged_force[axis], 1e-12);
  }

  ASSERT_LE(solver.solveContactForceQP(
                1, problem.mu, problem.inverse_mass, problem.velocity,
                problem.target, problem.external, problem.h, guess, 1, 1e-10),
            1e-9);
  for (int axis = 0; axis < 3; ++axis) {
    EXPECT_NEAR(guess[axis], converged_force[axis], 1e-8);
  }

  solver.setWarmStartEnabled(false);
  solver.prepareContactForceInitialGuess(pairs, frames, indices, problem.h,
                                         guess);
  EXPECT_EQ(guess, std::vector<double>({0.0, 0.0, 0.0}));
}

TEST(ShiftedSpectralAdmm, EqualsBaselineWithShiftedFreeNormalVelocity) {
  ContactCase shifted;
  shifted.external = {0.5, 0, -0.25};
  ContactCase baseline = shifted;
  baseline.velocity[2] += baseline.target[0];
  const auto originalVelocity = shifted.velocity;
  const auto originalTarget = shifted.target;
  ASSERT_LE(shifted.solve(), 1e-9);
  ASSERT_LE(baseline.solve(false), 1e-9);
  for (int axis = 0; axis < 3; ++axis) {
    EXPECT_NEAR(shifted.force[axis], baseline.force[axis], 1e-7);
  }
  EXPECT_EQ(shifted.velocity, originalVelocity);
  EXPECT_EQ(shifted.target, originalTarget);
}

TEST(ShiftedSpectralAdmm, NormalTargetSignAndImpulseUnits) {
  for (double target : {-1.0, 1.0}) {
    for (double h : {0.01, 0.02}) {
      ContactCase problem;
      problem.mu[0] = 0;
      problem.velocity = {0, 0, -2};
      problem.target[0] = target;
      problem.h = h;
      ASSERT_LE(problem.solve(), 1e-9);
      // Unit inverse mass: lambda_N = 2-target, c_N = -target.
      EXPECT_NEAR(h * problem.force[2], 2 - target, 1e-8);
      EXPECT_NEAR(-2 + h * problem.force[2], -target, 1e-8);
    }
  }
}

TEST(ShiftedSpectralAdmm, SlidingSatisfiesShiftedNormalAndCoulombBoundary) {
  ContactCase problem;
  ASSERT_LE(problem.solve(), 1e-9);
  // G=I, g_T=2, g_N=-1.5, mu=0.5: lambda_N=1.5, lambda_T=-0.75.
  EXPECT_NEAR(problem.h * problem.force[2], 1.5, 1e-8);
  EXPECT_NEAR(problem.h * problem.force[0], -0.75, 1e-8);
  EXPECT_NEAR(problem.force[1], 0, 1e-8);
  EXPECT_NEAR(
      problem.velocity[2] + problem.h * problem.force[2] + problem.target[0], 0,
      1e-8);
}

TEST(ShiftedSpectralAdmm, SeparatingShiftedVelocityDoesNotRequireAttraction) {
  ContactCase problem;
  problem.mu[0] = 0;
  problem.velocity = {0, 0, 1};
  problem.target[0] = 0.5;
  ASSERT_LE(problem.solve(), 1e-9);
  EXPECT_NEAR(problem.force[2], 0, 1e-8);
}

TEST(ShiftedSpectralAdmm, MissingTargetDoesNotSilentlySelectBaseline) {
  ContactCase problem;
  problem.target.clear();
  EXPECT_TRUE(std::isinf(problem.solve()));
}
}  // namespace
