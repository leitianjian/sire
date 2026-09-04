#include <cmath>
#include <stdexcept>

#include <gtest/gtest.h>

#include "sire/physics/contact/ps_vs_solver3.hpp"
#include "sire/physics/contact/ps_vs_solver_v5.hpp"

using sire::physics::contact::ps_vs_solver3::findSinglePointContactEndTime;

TEST(SinglePointContactPolynomial, AffineSeparationAndPersistentContact) {
  const double A[]{0, 1, 0, 0}, b[]{0, -2}, x0[]{1, 0};
  EXPECT_NEAR(findSinglePointContactEndTime(1, 1.5, A, b, x0, "polynomial"), 1, 1e-9);
  const double noForce[]{0, 0};
  EXPECT_DOUBLE_EQ(findSinglePointContactEndTime(1, 1.5, A, noForce, x0, "polynomial"), -1);
}

TEST(SinglePointContactPolynomial, StiffFallbackMatchesBaseline) {
  const double A[]{0, 1e4, -2e4, -5e3}, b[]{0, 0}, x0[]{0, 1};
  const double reference = findSinglePointContactEndTime(1, 1e-3, A, b, x0);
  EXPECT_NEAR(findSinglePointContactEndTime(1, 1e-3, A, b, x0, "polynomial"), reference, 2e-12);
}

TEST(SinglePointContactPolynomial, FlatZeroCannotHideCoupledSeparation) {
  const double A[]{0,0,1,0, 0,0,0,1, 0,0,0,0, 0,0,0,0};
  const double b[]{0,0,0,0}, x0[]{0,0.503,0,-1};
  EXPECT_NEAR(findSinglePointContactEndTime(2, 1, A, b, x0, "polynomial"), 0.503, 2e-12);
}

TEST(SinglePointContactPolynomial, ConfigurationValidation) {
  sire::physics::contact::ps_vs_solver_v5::PsVsSolverV5 solver;
  EXPECT_EQ(solver.contactTimeMethod(), "exponential");
  solver.setContactTimeMethod("polynomial");
  EXPECT_EQ(solver.contactTimeMethod(), "polynomial");
  EXPECT_THROW(solver.setContactTimeMethod("typo"), std::invalid_argument);
  EXPECT_EQ(solver.contactTimeMethod(), "polynomial");
  EXPECT_THROW(findSinglePointContactEndTime(0, 1, nullptr, nullptr, nullptr, "typo"), std::invalid_argument);
}

TEST(SinglePointContactPolynomial, StiffUnusedModeDoesNotHideLateSeparation) {
  // The second contact remains at constant depth, but its stiff velocity
  // mode forces tiny initial samples. Polynomial retries must not affect
  // detection near the final shortened interval after fixed propagation.
  const double A[]{0,0,1,0, 0,0,0,1, 0,0,0,0, 0,0,0,-1e6};
  const double b[]{0,0,0,0}, x0[]{0.000999,1,-1,0};
  const double reference = findSinglePointContactEndTime(2, 0.001, A, b, x0);
  EXPECT_NEAR(reference, 0.000999, 2e-12);
  EXPECT_NEAR(findSinglePointContactEndTime(2, 0.001, A, b, x0, "polynomial"),
              reference, 2e-12);
}

TEST(SinglePointContactPolynomial, NewImpactAndRealModesRetainFirstCrossing) {
  const double oscillator[]{0,1,-100,0}, zero[]{0,0}, impact[]{0,1};
  EXPECT_DOUBLE_EQ(findSinglePointContactEndTime(1, 0.1, oscillator, zero, impact, "polynomial"), -1);
  EXPECT_NEAR(findSinglePointContactEndTime(1, 0.5, oscillator, zero, impact, "polynomial"), std::acos(-1.0)/10, 1e-9);
  const double damped[]{0,1,-2,-3}, initial[]{1,-3};
  EXPECT_NEAR(findSinglePointContactEndTime(1, 1, damped, zero, initial, "polynomial"), std::log(2.0), 1e-9);
}

TEST(SinglePointContactPolynomial, MultipleCrossingsAndShortContact) {
  const double A[]{0,0,1,0, 0,0,0,1, 0,0,0,0, 0,0,0,0};
  const double b[]{0,0,0,0}, x0[]{0.503,5.1,-1,-10};
  EXPECT_NEAR(findSinglePointContactEndTime(2, 1, A, b, x0, "polynomial"), 0.503, 2e-12);
  const double smallA[]{0,1,0,0}, smallB[]{0,0}, smallX[]{1e-9,-1};
  const double time=findSinglePointContactEndTime(1, 1e-8, smallA, smallB, smallX, "polynomial");
  EXPECT_GT(time, 0);
  EXPECT_NEAR(time, 1e-9, 1e-15);
}

TEST(SinglePointContactTime, NewImpactDoesNotSelectInitialZero) {
  const double A[]{0, 1, -100, 0}, b[]{0, 0}, x0[]{0, 1};
  EXPECT_DOUBLE_EQ(findSinglePointContactEndTime(1, 0.1, A, b, x0), -1);
  EXPECT_NEAR(findSinglePointContactEndTime(1, 0.5, A, b, x0),
              std::acos(-1.0) / 10, 1e-9);
}

TEST(SinglePointContactTime, OverdampedSeparationWithRealEigenvalues) {
  // d(t) = -exp(-t) + 2 exp(-2t), first crossing at log(2).
  const double A[]{0, 1, -2, -3}, b[]{0, 0}, x0[]{1, -3};
  EXPECT_NEAR(findSinglePointContactEndTime(1, 1, A, b, x0),
              std::log(2.0), 1e-9);
}

TEST(SinglePointContactTime, StiffScaledDepthAndVelocity) {
  // The depth coordinate is scaled: depth' = 1e4*v, not v.
  // Underdamped motion starting at zero depth first separates at pi/omega.
  const double A[]{0, 1e4, -2e4, -5e3}, b[]{0, 0}, x0[]{0, 1};
  const double omega = std::sqrt(2e8 - 2.5e3 * 2.5e3);
  EXPECT_NEAR(findSinglePointContactEndTime(1, 1e-3, A, b, x0),
              std::acos(-1.0) / omega, 2e-12);
}

TEST(SinglePointContactTime, AffineForceAndSingularMatrix) {
  // d(t) = 1 - t^2. All eigenvalues are zero.
  const double A[]{0, 1, 0, 0}, b[]{0, -2}, x0[]{1, 0};
  EXPECT_NEAR(findSinglePointContactEndTime(1, 1.5, A, b, x0), 1, 1e-9);
}

TEST(SinglePointContactTime, SeparationInShortenedFinalScanInterval) {
  // d(t) = -exp(-t) + exp(0.99)*exp(-2t). The root lies in the
  // shortened final interval after the scan has reached its fixed step.
  const double coefficient = std::exp(0.99);
  const double A[]{0, 1, -2, -3}, b[]{0, 0};
  const double x0[]{coefficient - 1, 1 - 2 * coefficient};
  EXPECT_NEAR(findSinglePointContactEndTime(1, 1, A, b, x0), 0.99, 1e-9);
}

TEST(SinglePointContactTime, FixedTransitionIsLocalToEachCall) {
  const double A[]{0, 1, 0, 0}, x0[]{1, 0};
  const double firstForce[]{0, -2}, secondForce[]{0, -8}, noForce[]{0, 0};
  EXPECT_NEAR(findSinglePointContactEndTime(1, 1.5, A, firstForce, x0),
              1, 1e-9);
  EXPECT_NEAR(findSinglePointContactEndTime(1, 1.5, A, secondForce, x0),
              0.5, 1e-9);
  EXPECT_DOUBLE_EQ(findSinglePointContactEndTime(1, 1.5, A, noForce, x0), -1);
}

TEST(SinglePointContactTime, RetainsCouplingBetweenDifferentPairs) {
  // State order: d1,d2,v1,v2. d2=1, d1''=-2*d2, so d1=1-t^2.
  const double A[]{0, 0, 1, 0,
                   0, 0, 0, 1,
                   0, -2, 0, 0,
                   0, 0, 0, 0};
  const double b[]{0, 0, 0, 0}, x0[]{1, 1, 0, 0};
  EXPECT_NEAR(findSinglePointContactEndTime(2, 1.5, A, b, x0), 1, 1e-9);
}

TEST(SinglePointContactTime, DoesNotDiscardShortPositiveContactTime) {
  const double A[]{0, 1, 0, 0}, b[]{0, 0}, x0[]{1e-9, -1};
  const double time = findSinglePointContactEndTime(1, 1e-8, A, b, x0);
  EXPECT_GT(time, 0);
  EXPECT_NEAR(time, 1e-9, 1e-15);
}

TEST(SinglePointContactTime, PersistentContactHasNoSeparation) {
  const double A[]{0, 1, 0, 0}, b[]{0, 0}, x0[]{1, 0};
  EXPECT_DOUBLE_EQ(findSinglePointContactEndTime(1, 1, A, b, x0), -1);
}

TEST(SinglePointContactTime, ZeroDepthContactDoesNotHideAnotherSeparation) {
  // d1 stays identically zero; d2=0.503-t. A zero minimum is not a root
  // at which the search may terminate: separation requires a negative depth.
  const double A[]{0, 0, 1, 0,
                   0, 0, 0, 1,
                   0, 0, 0, 0,
                   0, 0, 0, 0};
  const double b[]{0, 0, 0, 0}, x0[]{0, 0.503, 0, -1};
  EXPECT_NEAR(findSinglePointContactEndTime(2, 1, A, b, x0), 0.503, 2e-12);
}

TEST(SinglePointContactTime, EarliestOfTwoCrossingsInOneScanInterval) {
  // Both cross inside (0.5, 0.53125). Contact 2 has the more negative
  // right-end depth but contact 1 separates first; do not select by endpoint.
  const double A[]{0, 0, 1, 0,
                   0, 0, 0, 1,
                   0, 0, 0, 0,
                   0, 0, 0, 0};
  const double b[]{0, 0, 0, 0}, x0[]{0.503, 5.1, -1, -10};
  EXPECT_NEAR(findSinglePointContactEndTime(2, 1, A, b, x0), 0.503, 2e-12);
}

TEST(SinglePointContactTime, ExactZeroAtScanPointStillRequiresSeparation) {
  const double A[]{0, 1, 0, 0}, b[]{0, 0}, x0[]{0.5, -1};
  EXPECT_NEAR(findSinglePointContactEndTime(1, 1, A, b, x0), 0.5, 2e-12);
}

TEST(SinglePointContactMode, IsExplicitAndPreservesExistingDefault) {
  sire::physics::contact::ps_vs_solver3::PsVsSolver3 v3;
  sire::physics::contact::ps_vs_solver_v5::PsVsSolverV5 v5;
  EXPECT_EQ(v3.contactModelMode(), "height_field");
  EXPECT_EQ(v5.contactModelMode(), "height_field");
  v3.setContactModelMode("single_point");
  v5.setContactModelMode("single_point");
  EXPECT_TRUE(v3.singlePointContactMode());
  EXPECT_TRUE(v5.singlePointContactMode());
  EXPECT_THROW(v3.setContactModelMode("typo"), std::invalid_argument);
  EXPECT_TRUE(v3.singlePointContactMode());
  v3.setContactModelMode("height_field");
  EXPECT_FALSE(v3.singlePointContactMode());
}
