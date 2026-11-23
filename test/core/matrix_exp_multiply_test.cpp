#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "sire/core/force_screw.hpp"

using sire::core::screw::matrix_exp_multiply;
using sire::core::screw::MatrixExpMultiplyWorkspace;

TEST(MatrixExpPade, ReusedWorkspaceZeroJordanAndNegativeTime) {
  sire::core::screw::MatrixExpPadeWorkspace work(2);
  for (double t : {0.0, 0.7, -0.2, 2.0}) {
    double matrix[]{-1, 3, 0, -1};
    work.apply(matrix, matrix, t);
    EXPECT_NEAR(matrix[0], std::exp(-t), 2e-12);
    EXPECT_NEAR(matrix[1], 3*t*std::exp(-t), 2e-12);
    EXPECT_NEAR(matrix[2], 0, 2e-12);
    EXPECT_NEAR(matrix[3], std::exp(-t), 2e-12);
  }
  double zero[4]{};
  work.apply(zero, zero);
  EXPECT_DOUBLE_EQ(zero[0], 1);
  EXPECT_DOUBLE_EQ(zero[1], 0);
  EXPECT_DOUBLE_EQ(zero[2], 0);
  EXPECT_DOUBLE_EQ(zero[3], 1);
}

TEST(MatrixExpPade, AffineNilpotentAndRepeatedSquaring) {
  sire::core::screw::MatrixExpPadeWorkspace work(3);
  const double matrix[]{0, 1, 0, 0, 0, -2, 0, 0, 0};
  for (double t : {0.01, 1.0, 16.0}) {
    double out[9];
    work.apply(matrix, out, t);
    const double expected[]{1, t, -t*t, 0, 1, -2*t, 0, 0, 1};
    for (int i=0; i<9; ++i) EXPECT_NEAR(out[i], expected[i], 1e-10);
  }
}

TEST(MatrixExpPade, RotationRetainsFullCoupling) {
  sire::core::screw::MatrixExpPadeWorkspace work(2);
  const double matrix[]{0, 10, -10, 0};
  double out[4];
  work.apply(matrix, out, 1.0);
  EXPECT_NEAR(out[0], std::cos(10.0), 2e-12);
  EXPECT_NEAR(out[1], std::sin(10.0), 2e-12);
  EXPECT_NEAR(out[2], -std::sin(10.0), 2e-12);
  EXPECT_NEAR(out[3], std::cos(10.0), 2e-12);
}

TEST(MatrixExpMultiply, WorkspaceReusesMatrixWithDifferentTimesAndVectors) {
  double A[]{-1, 3, 0, -1};
  MatrixExpMultiplyWorkspace action(2, A);
  A[1] = 100;  // Prepared matrix owns its data.
  for (double time : {0.7, 0.0, -0.2, 2.0, 1e-8}) {
    double v[]{2, -1};
    action.apply(v, v, time);
    EXPECT_NEAR(v[0], std::exp(-time) * (2 - 3 * time), 2e-12);
    EXPECT_NEAR(v[1], -std::exp(-time), 2e-12);
  }
  const double v[]{1, 2};
  double out[2];
  action.apply(v, out, 0.7);
  EXPECT_NEAR(out[0], std::exp(-0.7) * 5.2, 2e-12);
  EXPECT_NEAR(out[1], std::exp(-0.7) * 2, 2e-12);
}

TEST(MatrixExpMultiply, WorkspaceAlternatesFallbackAndTaylor) {
  const double A[]{0, 1e5, 0, 0};
  MatrixExpMultiplyWorkspace action(2, A);
  for (double time : {1.0, 1e-7, -1e-7, 1.0}) {
    double v[]{1, 2};
    action.apply(v, v, time);
    EXPECT_NEAR(v[0], 1 + 2e5 * time, 1e-5);
    EXPECT_NEAR(v[1], 2, 1e-10);
  }
}

TEST(MatrixExpMultiply, WorkspaceLocalPropagationRetainsAffineTerm) {
  const double B[]{0, 1, 0, 0, 0, -2, 0, 0, 0};
  MatrixExpMultiplyWorkspace action(3, B);
  double state[]{1, 0, 1};
  double elapsed = 0;
  for (double dt : {0.001, 0.003, 0.01, 0.02, 0.05, 0.1, 0.5, 0.566}) {
    elapsed += dt;
    action.apply(state, state, dt);
    EXPECT_NEAR(state[0], 1 - elapsed * elapsed, 2e-13);
    EXPECT_NEAR(state[1], -2 * elapsed, 2e-13);
    EXPECT_DOUBLE_EQ(state[2], 1);
  }
}

TEST(MatrixExpMultiply, ZeroTimeZeroMatrixAndEmptyInput) {
  const double A[]{0, 0, 0, 0}, v[]{2, -3};
  double out[2];
  matrix_exp_multiply(2, A, v, out);
  EXPECT_DOUBLE_EQ(out[0], v[0]);
  EXPECT_DOUBLE_EQ(out[1], v[1]);
  const double nonzero[]{1, 2, 3, 4};
  matrix_exp_multiply(2, nonzero, v, out, 0);
  EXPECT_DOUBLE_EQ(out[0], v[0]);
  EXPECT_DOUBLE_EQ(out[1], v[1]);
  EXPECT_NO_THROW(matrix_exp_multiply(0, nullptr, nullptr, nullptr));
}

TEST(MatrixExpMultiply, InPlaceVectorAndNegativeTime) {
  const double A[]{-2, 0, 0, 3};
  double v[]{2, -3};
  matrix_exp_multiply(2, A, v, v, -0.2);
  EXPECT_NEAR(v[0], 2 * std::exp(0.4), 2e-13);
  EXPECT_NEAR(v[1], -3 * std::exp(-0.6), 2e-13);
  EXPECT_DOUBLE_EQ(A[0], -2);
  EXPECT_DOUBLE_EQ(A[3], 3);
}

TEST(MatrixExpMultiply, JordanCouplingAndTraceShift) {
  // exp(t*A) = exp(-t)*[[1, 3t], [0, 1]]; no eigenvector basis exists.
  const double A[]{-1, 3, 0, -1}, v[]{1, 2};
  double out[2];
  matrix_exp_multiply(2, A, v, out, 0.7);
  EXPECT_NEAR(out[0], std::exp(-0.7) * 5.2, 2e-13);
  EXPECT_NEAR(out[1], std::exp(-0.7) * 2, 2e-13);
}

TEST(MatrixExpMultiply, SingularAffineAugmentation) {
  // d''=-2 with d(0)=1, d'(0)=0. The last coordinate is the affine 1.
  const double Ab[]{0, 1, 0, 0, 0, -2, 0, 0, 0}, v[]{1, 0, 1};
  double out[3];
  matrix_exp_multiply(3, Ab, v, out, 1.25);
  EXPECT_NEAR(out[0], 1 - 1.25 * 1.25, 2e-13);
  EXPECT_NEAR(out[1], -2.5, 2e-13);
  EXPECT_DOUBLE_EQ(out[2], 1);
}

TEST(MatrixExpMultiply, MultipleScalingStepsWithOscillation) {
  const double A[]{-1, -40, 40, -1}, v[]{1, 0};
  double out[2];
  matrix_exp_multiply(2, A, v, out);
  EXPECT_NEAR(out[0], std::exp(-1.0) * std::cos(40.0), 2e-12);
  EXPECT_NEAR(out[1], std::exp(-1.0) * std::sin(40.0), 2e-12);
}

TEST(MatrixExpMultiply, LargeTraceDoesNotUnderflowBeforeMultiplication) {
  const double A[]{-1000}, v[]{1e300};
  double out[1];
  matrix_exp_multiply(1, A, v, out);
  const double expected = std::exp(std::log(v[0]) - 1000);
  EXPECT_GT(out[0], 0);
  EXPECT_NEAR(out[0] / expected, 1, 2e-13);
}

TEST(MatrixExpMultiply, LargeNonnormalNormUsesFiniteFallback) {
  // A^2=0 gives exp(A)*v=v+A*v, even though ||A|| is large.
  const double A[]{0, 1e5, 0, 0};
  double v[]{1, 2};
  matrix_exp_multiply(2, A, v, v);
  EXPECT_NEAR(v[0], 200001, 1e-5);
  EXPECT_NEAR(v[1], 2, 1e-10);
}

TEST(MatrixExpMultiply, RejectsNonFiniteInputAndOutput) {
  const double finite[]{1}, bad[]{std::numeric_limits<double>::infinity()};
  double out[]{123};
  EXPECT_THROW(matrix_exp_multiply(1, bad, finite, out), std::invalid_argument);
  EXPECT_THROW(matrix_exp_multiply(1, finite, bad, out), std::invalid_argument);
  EXPECT_THROW(matrix_exp_multiply(1, finite, finite, out,
                                 std::numeric_limits<double>::quiet_NaN()),
               std::invalid_argument);
  EXPECT_THROW(matrix_exp_multiply(1, nullptr, finite, out), std::invalid_argument);
  const double large[]{1000};
  EXPECT_THROW(matrix_exp_multiply(1, large, finite, out), std::overflow_error);
  EXPECT_DOUBLE_EQ(out[0], 123);
}
