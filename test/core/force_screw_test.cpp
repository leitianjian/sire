#include "sire/core/force_screw.hpp"

#include "gtest/gtest.h"

#include <aris/dynamic/math_matrix.hpp>

using namespace sire::core;
GTEST_TEST(ForceScrewTester, MatrixNormTest) {
  // matrix_norm
  const double matrix[4]{1, 2, 3, 4};
  EXPECT_EQ(screw::matrix_norm(2, 2, matrix, 0), 7);
  EXPECT_EQ(screw::matrix_norm(2, 2, matrix, 1), 6);
  const double matrix1[4]{-1, -2, -3, -4};
  EXPECT_EQ(screw::matrix_norm(2, 2, matrix1, 0), 7);
  EXPECT_EQ(screw::matrix_norm(2, 2, matrix1, 1), 6);
  const double matrix2[]{0, 1, 2, 0.5, 0, 1, 2, 1, 0};
  EXPECT_EQ(screw::matrix_norm(3, 3, matrix2, 0), 3);
  EXPECT_EQ(screw::matrix_norm(3, 3, matrix2, 1), 3);
}

GTEST_TEST(ForceScrewTester, MatrixExpTest) {
  // matrix_norm
  double matrix[]{0, 1, 2, 0.5, 0, 1, 2, 1, 0};
  double matExpMatlab[9] = {5.309081, 4.001203, 5.577840, 2.808790, 2.884516,
                            3.193014, 5.173746, 4.001203, 5.713176};
  screw::matrix_exp_pade(3, matrix, matrix);
  EXPECT_TRUE(aris::dynamic::s_is_equal(3, 3, matrix, matExpMatlab, 1e-6));

  double matrix1[]{-147, 72, -192, 93};
  double matExpMatlab1[4] = {-0.099574, 0.074681, -0.199148, 0.149361};
  screw::matrix_exp_pade(2, matrix1, matrix1);
  EXPECT_TRUE(aris::dynamic::s_is_equal(2, 2, matrix1, matExpMatlab1, 1e-6));

  double matrix2[]{-1, 1, 0, -1};
  double matExpMatlab2[4] = {0.367879, 0.367879, 0.000000, 0.367879};
  screw::matrix_exp_pade(2, matrix2, matrix2);
  EXPECT_TRUE(aris::dynamic::s_is_equal(2, 2, matrix2, matExpMatlab2, 1e-6));

  double A_1e4[64] = {
      0.000000,     0.000000,     0.000000,     0.000000,     0.000100,
      0.000000,     0.000000,     0.000000,     0.000000,     0.000000,
      0.000000,     0.000000,     0.000000,     0.000100,     0.000000,
      0.000000,     0.000000,     0.000000,     0.000000,     0.000000,
      0.000000,     0.000000,     0.000100,     0.000000,     0.000000,
      0.000000,     0.000000,     0.000000,     0.000000,     0.000000,
      0.000000,     0.000100,     -4730.041327, -1569.999537, -1569.999537,
      4373.440316,  -0.023650,    -0.007850,    -0.007850,    0.021867,
      -1569.999537, -4730.041327, 4373.440316,  -1569.999537, -0.007850,
      -0.023650,    0.021867,     -0.007850,    -1569.999537, 4373.440316,
      -4730.041327, -1569.999537, -0.007850,    0.021867,     -0.023650,
      -0.007850,    4373.440316,  -1569.999537, -1569.999537, -4730.041327,
      0.021867,     -0.007850,    -0.007850,    -0.023650};
  double expA_1e4_matlab[64] = {
      0.785846,     -0.077977,    -0.077977,    0.201294,     0.000092,
      -0.000003,    -0.000003,    0.000008,     -0.077977,    0.785846,
      0.201294,     -0.077977,    -0.000003,    0.000092,     0.000008,
      -0.000003,    -0.077977,    0.201294,     0.785846,     -0.077977,
      -0.000003,    0.000008,     0.000092,     -0.000003,    0.201294,
      -0.077977,    -0.077977,    0.785846,     0.000008,     -0.000003,
      -0.000003,    0.000092,     -3887.921009, -1550.604497, -1550.604497,
      3721.414081,  0.766406,     -0.085730,    -0.085730,    0.219901,
      -1550.604497, -3887.921009, 3721.414081,  -1550.604497, -0.085730,
      0.766406,     0.219901,     -0.085730,    -1550.604497, 3721.414081,
      -3887.921009, -1550.604497, -0.085730,    0.219901,     0.766406,
      -0.085730,    3721.414081,  -1550.604497, -1550.604497, -3887.921009,
      0.219901,     -0.085730,    -0.085730,    0.766406};
  screw::matrix_exp_pade(8, A_1e4, A_1e4);
  // 由于验证时输入的matlab的矩阵的位数限制导致的误差较大为 1e-2 可以不管
  EXPECT_TRUE(aris::dynamic::s_is_equal(8, 8, A_1e4, expA_1e4_matlab, 1e-2));
}
