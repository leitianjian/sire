#include <cmath>
#include <iostream>
#include <vector>

#include <Eigen/Dense>
#include <clarabel.hpp>
#include <gtest/gtest.h>

#include "sire/core/constants.hpp"
#include "sire/core/force_screw.hpp"
#include "sire/core/geometry/geometry_base.hpp"
#include "sire/core/sire_assert.hpp"
#include "sire/core/sire_fixed_joint.hpp"
#include "sire/physics/collision/collision_filter.hpp"
#include "sire/physics/collision/penetration_as_point_pair_callback.hpp"
#include "sire/physics/common/penetration_as_point_pair.hpp"
#include "sire/physics/contact/ps_vs_solver.hpp"
#include "sire/physics/geometry/sphere_collision_geometry.hpp"

using namespace Eigen;
using namespace clarabel;
using namespace sire::physics::contact;

TEST(ClarabelTest, IterativeFrictionSolve) {
  // 1. ===== 物理参数初始化 =====
  int N = 4;
  double h = 0.001;
  double mu = 0.3;

  // 速度与外力边界条件
  VectorXd v0(3 * N);
  v0 << 0.0526404, 0.0110721, 0.0579474, 0.0691226, 1.03524e-05, 0.0583075,
      0.0612058, -0.0166839, 0.0579826, 0.0562369, 0.0123911, 0.0585853;

  VectorXd b_ext(3 * N);
  b_ext << 25.8184, 35.8036, 144.122, 19.5788, 59.0772, 316.233, 27.917,
      54.8199, 323.789, 16.0631, 35.4056, 144.615;

  VectorXd v_target(N);
  v_target << 0.0582191, 0.0582164, 0.0582121, 0.0582143;

  // 原始矩阵 A_dense (使用逗号初始化)
  MatrixXd A_dense(12, 12);
  A_dense << -4.264208, -0.007513, -0.165674, -0.028791, 0.011917, 0.000612,
      -0.154848, -0.027276, 0.000104, 0.124287, 0.006857, 0.001586, -0.007513,
      -3.886122, -2.251832, 0.011517, -0.008019, -0.001716, 0.024421, 0.001278,
      -0.001996, -0.011560, -0.001604, -0.000798, -0.165674, -2.251832,
      -6.440301, 0.004865, -0.003849, -0.001034, 0.007146, -0.000370, -0.000950,
      -0.002020, -0.000762, -0.000349, -0.028791, 0.011517, 0.004865, -3.279944,
      -0.086687, 0.051068, 0.129998, 0.021669, 0.003730, -0.153854, -0.018874,
      -0.006894, 0.011917, -0.008019, -0.003849, -0.086687, -2.494906,
      -1.079557, -0.026896, -0.009045, -0.003082, 0.032468, 0.000911, -0.000209,
      0.000612, -0.001716, -0.001034, 0.051068, -1.079557, -7.358521, -0.004721,
      -0.003125, -0.000929, 0.000559, -0.001941, -0.000942, -0.154848, 0.024421,
      0.007146, 0.129998, -0.026896, -0.004721, -3.305642, 0.102957, -0.110156,
      -0.042572, -0.012158, -0.005429, -0.027276, 0.001278, -0.000370, 0.021669,
      -0.009045, -0.003125, 0.102957, -2.519981, -1.103224, -0.013531,
      -0.007532, -0.003784, 0.000104, -0.001996, -0.000950, 0.003730, -0.003082,
      -0.000929, -0.110156, -1.103224, -7.348570, -0.000821, -0.001674,
      -0.001022, 0.124287, -0.011560, -0.002020, -0.153854, 0.032468, 0.000559,
      -0.042572, -0.013531, -0.000821, -4.286257, 0.012754, 0.081210, 0.006857,
      -0.001604, -0.000762, -0.018874, 0.000911, -0.001941, -0.012158,
      -0.007532, -0.001674, 0.012754, -3.905390, -2.264855, 0.001586, -0.000798,
      -0.000349, -0.006894, -0.000209, -0.000942, -0.005429, -0.003784,
      -0.001022, 0.081210, -2.264855, -6.426958;

  // ================= 适配引擎原生矩阵架构的正确解法 =================
  MatrixXd W = -h * A_dense;
  // 强制转换为完全对称、正定矩阵
  MatrixXd P = 0.5 * (W + W.transpose());

  VectorXd v_free = v0 - h * b_ext;
  VectorXd q = v_free;

  // 获取法向 (n) 与切向 (t) 的索引数组
  std::vector<int> idx_n, idx_t;
  for (int i = 0; i < N; ++i) {
    idx_t.push_back(3 * i + 0);  // X 轴切向
    idx_t.push_back(3 * i + 1);  // Y 轴切向
    idx_n.push_back(3 * i + 2);  // Z 轴法向
  }

  // 分块抽取矩阵
  int dim_n = idx_n.size();  // N
  int dim_t = idx_t.size();  // 2N
  MatrixXd P_nn(dim_n, dim_n), P_nt(dim_n, dim_t), P_tt(dim_t, dim_t);
  VectorXd q_n(dim_n), q_t(dim_t);

  for (int i = 0; i < dim_n; ++i) {
    q_n(i) = q(idx_n[i]);
    for (int j = 0; j < dim_n; ++j) P_nn(i, j) = P(idx_n[i], idx_n[j]);
    for (int j = 0; j < dim_t; ++j) P_nt(i, j) = P(idx_n[i], idx_t[j]);
  }
  for (int i = 0; i < dim_t; ++i) {
    q_t(i) = q(idx_t[i]);
    for (int j = 0; j < dim_t; ++j) P_tt(i, j) = P(idx_t[i], idx_t[j]);
  }

  // ===== 结构用于 Clarabel 摩擦锥问题的稀疏化 =====
  SparseMatrix<double> P_tt_sparse =
      MatrixXd(P_tt.triangularView<Eigen::Upper>()).sparseView();
  P_tt_sparse.makeCompressed();

  // 构造 Clarabel A 矩阵: 维度是 (3N) x (2N)
  std::vector<Eigen::Triplet<double>> triplets;
  for (int i = 0; i < N; ++i) {
    triplets.push_back(Eigen::Triplet<double>(3 * i + 1, 2 * i + 0, -1.0));
    triplets.push_back(Eigen::Triplet<double>(3 * i + 2, 2 * i + 1, -1.0));
  }
  SparseMatrix<double> A_sparse(3 * N, 2 * N);
  A_sparse.setFromTriplets(triplets.begin(), triplets.end());
  A_sparse.makeCompressed();

  // 初始化锥结构
  std::vector<SupportedConeT<double>> cones;
  for (int i = 0; i < N; ++i) {
    cones.push_back(SecondOrderConeT<double>(3));
  }

  // 迭代初值
  VectorXd fn_val = VectorXd::Zero(N);
  VectorXd ft_val = VectorXd::Zero(2 * N);
  int max_iters = 20;
  bool converged = false;

  // std::cout << "\n[Clarabel-Iterative] 融合引擎底层符号求解开始...\n";
  for (int iter = 1; iter <= max_iters; ++iter) {
    // ---- 1. 法向量支持力求解 ----
    VectorXd rhs = v_target - q_n - P_nt * ft_val;
    VectorXd fn_new = P_nn.ldlt().solve(rhs);
    fn_new = fn_new.cwiseMax(0.0);
    fn_val = fn_new;

    // ---- 2. 构建切向优化问题 ----
    VectorXd q_tt_current = P_nt.transpose() * fn_val + q_t;

    // 由于 Clarabel 的 C++ 封装提供了对 Eigen 松散矩阵的直接集成，
    // 我们不需要自己构造 CscMatrix，直接使用 SparseMatrix 即可。
    // q_clarabel 和 b_clarabel 也应该转为 Eigen::VectorXd
    VectorXd b_eigen = VectorXd::Zero(3 * N);
    for (int i = 0; i < N; ++i) {
      b_eigen[3 * i] = mu * fn_val(i);
    }

    DefaultSettings<double> settings =
        DefaultSettingsBuilder<double>::default_settings().build();
    settings.verbose = false;

    DefaultSolver<double> solver(P_tt_sparse, q_tt_current, A_sparse, b_eigen,
                                 cones, settings);
    solver.solve();

    EXPECT_EQ(solver.solution().status, SolverStatus::Solved);

    VectorXd ft_new(2 * N);
    for (int i = 0; i < 2 * N; ++i) {
      ft_new(i) = solver.solution().x[i];
    }

    double err = (ft_new - ft_val).norm();
    ft_val = ft_new;

    if (err < 1e-6) {
      converged = true;
      std::cout << "[Clarabel-Iterative] 迭代已在第 " << iter
                << " 次内精准收敛!\n";
      break;
    }
  }

  EXPECT_TRUE(converged);

  // ===== 结果组装验证 =====
  VectorXd f_opt = VectorXd::Zero(3 * N);
  for (int i = 0; i < N; ++i) {
    f_opt(idx_n[i]) = fn_val(i);
    f_opt(idx_t[2 * i]) = ft_val(2 * i);
    f_opt(idx_t[2 * i + 1]) = ft_val(2 * i + 1);
  }

  VectorXd v_opt = q + P * f_opt;

  std::cout << "\n========== Clarabel (原生动力学匹配体系) 最优解 ==========\n";
  for (int i = 0; i < N; ++i) {
    int idx = 3 * i;
    std::cout << "Contact Point " << i + 1 << ":\n";

    std::cout << "  Vel  (v0) =  " << v0.segment(idx, 3).transpose() << "\n";
    std::cout << "  Vel   (v) =  " << v_opt.segment(idx, 3).transpose() << "\n";
    std::cout << "  Force (f) =  " << f_opt.segment(idx, 3).transpose() << "\n";

    // Validate final velocity targets
    EXPECT_NEAR(v_opt(idx + 2), v_target(i), 1e-5);
  }
}

TEST(ClarabelTest, IterativeFrictionSolveFunc) {
  // 1. ===== 物理参数初始化 =====
  int N = 4;
  double h = 0.001;
  double mu = 0.3;

  // 速度与外力边界条件
  VectorXd v0(3 * N);
  v0 << 0.0526404, 0.0110721, 0.0579474, 0.0691226, 1.03524e-05, 0.0583075,
      0.0612058, -0.0166839, 0.0579826, 0.0562369, 0.0123911, 0.0585853;

  VectorXd b_ext(3 * N);
  b_ext << 25.8184, 35.8036, 144.122, 19.5788, 59.0772, 316.233, 27.917,
      54.8199, 323.789, 16.0631, 35.4056, 144.615;

  VectorXd v_target(N);
  v_target << 0.0582191, 0.0582164, 0.0582121, 0.0582143;

  // 原始矩阵 A_dense (使用逗号初始化)
  MatrixXd A_dense(12, 12);
  A_dense << -4.264208, -0.007513, -0.165674, -0.028791, 0.011917, 0.000612,
      -0.154848, -0.027276, 0.000104, 0.124287, 0.006857, 0.001586, -0.007513,
      -3.886122, -2.251832, 0.011517, -0.008019, -0.001716, 0.024421, 0.001278,
      -0.001996, -0.011560, -0.001604, -0.000798, -0.165674, -2.251832,
      -6.440301, 0.004865, -0.003849, -0.001034, 0.007146, -0.000370, -0.000950,
      -0.002020, -0.000762, -0.000349, -0.028791, 0.011517, 0.004865, -3.279944,
      -0.086687, 0.051068, 0.129998, 0.021669, 0.003730, -0.153854, -0.018874,
      -0.006894, 0.011917, -0.008019, -0.003849, -0.086687, -2.494906,
      -1.079557, -0.026896, -0.009045, -0.003082, 0.032468, 0.000911, -0.000209,
      0.000612, -0.001716, -0.001034, 0.051068, -1.079557, -7.358521, -0.004721,
      -0.003125, -0.000929, 0.000559, -0.001941, -0.000942, -0.154848, 0.024421,
      0.007146, 0.129998, -0.026896, -0.004721, -3.305642, 0.102957, -0.110156,
      -0.042572, -0.012158, -0.005429, -0.027276, 0.001278, -0.000370, 0.021669,
      -0.009045, -0.003125, 0.102957, -2.519981, -1.103224, -0.013531,
      -0.007532, -0.003784, 0.000104, -0.001996, -0.000950, 0.003730, -0.003082,
      -0.000929, -0.110156, -1.103224, -7.348570, -0.000821, -0.001674,
      -0.001022, 0.124287, -0.011560, -0.002020, -0.153854, 0.032468, 0.000559,
      -0.042572, -0.013531, -0.000821, -4.286257, 0.012754, 0.081210, 0.006857,
      -0.001604, -0.000762, -0.018874, 0.000911, -0.001941, -0.012158,
      -0.007532, -0.001674, 0.012754, -3.905390, -2.264855, 0.001586, -0.000798,
      -0.000349, -0.006894, -0.000209, -0.000942, -0.005429, -0.003784,
      -0.001022, 0.081210, -2.264855, -6.426958;
  std::vector<double> f_opt(3 * N, 0.0);
  double error = ps_vs_solver::cptContactForceWithTargetState2(
      N, std::vector<double>(N, mu),
      std::vector<double>(A_dense.data(), A_dense.data() + A_dense.size()),
      std::vector<double>(v0.data(), v0.data() + v0.size()),
      std::vector<double>(v_target.data(), v_target.data() + v_target.size()),
      std::vector<double>(b_ext.data(), b_ext.data() + b_ext.size()), h, f_opt,
      20, 1e-6);
  EXPECT_TRUE(error < 1e-6);
}

TEST(ClarabelTest, IterativeFrictionSolveFunc2) {
  // 1. ===== 物理参数初始化 =====
  int N = 3;
  double h = 0.001;
  double mu = 0.6;

  // 速度与外力边界条件
  VectorXd v0(3 * N);
  v0 << 0.0497166, 0.385625, 0.345865, 1.06539, 0.431899, -0.576336, -0.0827677,
      -1.44893, -0.645305;

  VectorXd b_ext(3 * N);
  b_ext << 0.100907, 0.773947, 0.000121588, -0.0873541, -1.43152, -0.657577, 0.980619, 0.866741, -1.15425;

  VectorXd v_target(N);
  v_target << -5.08182e-06, -1.90533e-05, -6.13275e-06;

  // 原始矩阵 A_dense (使用逗号初始化)
  MatrixXd A_dense(9, 9);
  A_dense << -4.975263, 0.103792, 0.252984, -0.033713, 0.007041, -0.002186,
      -0.098221, -0.011562, -0.018066, 0.103792, -7.817534, 1.596598, 0.006437,
      -0.003465, 0.000892, 0.004368, -0.001333, 0.000380, 0.252984, 1.596598,
      -2.952896, 0.004755, -0.001332, 0.000362, 0.008567, 0.000922, 0.001568,
      -0.033713, 0.006437, 0.004755, -3.987417, -0.003287, -0.006455, 0.112899,
      0.008041, 0.018930, 0.007041, -0.003465, -0.001332, -0.003287, -4.927422,
      2.811272, -0.012354, -0.001465, -0.002235, -0.002186, 0.000892, 0.000362,
      -0.006455, 2.811272, -5.659237, 0.006142, 0.000508, 0.001078, -0.098221,
      0.004368, 0.008567, 0.112899, -0.012354, 0.006142, -4.599940, -0.202904,
      -0.345801, -0.011562, -0.001333, 0.000922, 0.008041, -0.001465, 0.000508,
      -0.202904, -7.137779, 1.665478, -0.018066, 0.000380, 0.001568, 0.018930,
      -0.002235, 0.001078, -0.345801, 1.665478, -3.112867;
  std::vector<double> f_opt(3 * N, 0.0);
  double error = ps_vs_solver::cptContactForceWithTargetState2(
      N, std::vector<double>(N, mu),
      std::vector<double>(A_dense.data(), A_dense.data() + A_dense.size()),
      std::vector<double>(v0.data(), v0.data() + v0.size()),
      std::vector<double>(v_target.data(), v_target.data() + v_target.size()),
      std::vector<double>(b_ext.data(), b_ext.data() + b_ext.size()), h, f_opt,
      20, 1e-6);
  EXPECT_TRUE(error < 1e-6);
}