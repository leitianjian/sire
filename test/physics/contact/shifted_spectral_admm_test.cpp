#include <cmath>
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

TEST(ShiftedSpectralAdmm, DefaultsToSinglePointWithoutChangingBaselineDefault) {
  ShiftedSpectralAdmmContactSolver shifted;
  SimpleAdmmContactSolver baseline;
  EXPECT_EQ(shifted.contactModelMode(), "single_point");
  EXPECT_EQ(baseline.contactModelMode(), "height_field");
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
