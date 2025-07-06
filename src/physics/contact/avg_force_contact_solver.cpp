#include "sire/physics/contact/avg_force_contact_solver.hpp"

#include <array>
#include <cmath>
#include <fstream>
#include <map>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "log/easyloggingConfig.hpp"
#include <eigen3/Eigen/Eigenvalues>

#include <aris/core/reflection.hpp>
#include <aris/dynamic/math_matrix.hpp>
#include <aris/dynamic/model.hpp>
#include <aris/dynamic/model_force.hpp>
#include <aris/dynamic/model_interaction.hpp>
#include <aris/server/control_server.hpp>

#include "sire/core/constants.hpp"
#include "sire/core/force_screw.hpp"
#include "sire/core/material_manager.hpp"
#include "sire/core/prop_map.hpp"
#include "sire/physics/collision/collision_detection.hpp"
#include "sire/physics/collision/collision_exists_callback.hpp"
#include "sire/physics/geometry/collidable_geometry.hpp"
#include "sire/physics/physics_engine.hpp"
#include "sire/simulator/simulation_loop.hpp"

namespace sire::physics::contact {
using PartPool =
    aris::core::PointerArray<aris::dynamic::Part, aris::dynamic::Element>;
class FceActiveStateRecorder {
 public:
  std::vector<bool> fce_active_;
  aris::dynamic::Model* model_;

  FceActiveStateRecorder(aris::dynamic::Model* model) : model_(model) {
    for (auto& fce : model_->forcePool()) fce_active_.push_back(fce.active());
  }
  ~FceActiveStateRecorder() {
    for (auto& fce : model_->forcePool()) fce.activate(fce_active_[fce.id()]);
  }
};
auto cptAccelExtVector(
    aris::dynamic::Model& model,
    const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    const std::vector<std::array<double, 16>>& T_C_vec,
    const std::vector<sire::Size>& preservedPairsIdx,
    const sire::PartId* prtIdVector, double* accelExt) -> void {
  auto& partPool = model.partPool();
  // TODO: 可能需要关掉contactForce
  // TODO: 不知道是否需要，记录杆件的加速度数据，然后要重新填回去
  if (model.forwardDynamics())
    std::cout << "forward dynamic failed" << std::endl;

  // 遍历碰撞点，找到所有要求的杆件与碰撞点位姿
  for (sire::Size i{0}, accelExtColIdx{0}; i < preservedPairsIdx.size(); ++i) {
    auto& pair = penetration_pairs[preservedPairsIdx[i]];
    const double* contactPosition = pair.p_WC.data();
    for (sire::Size i2{0}; i2 < 2; ++i2) {
      auto& prt = partPool[prtIdVector[2 * i + i2]];
      double ap_o[3]{0}, ap_c[3]{0};
      aris::dynamic::s_as2ap(prt.vs(), prt.as(), contactPosition, ap_o);
      aris::dynamic::s_inv_pm_dot_v3(T_C_vec[i].data(), ap_o, ap_c);
      accelExt[accelExtColIdx] = ap_c[2];
      ++accelExtColIdx;
    }
  }
}
auto cptInitialCondition(
    sire::physics::PhysicsEngine& engine, sire::core::MaterialManager& manager,
    const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    const std::vector<std::array<double, 16>>& T_C_vec,
    const std::vector<sire::Size>& preservedPairsIdx, double* stiffness,
    double* damping, double* x0, double* v0) -> double {
  sire::Size n{preservedPairsIdx.size()};
  sire::Size n2{2 * n};
  double minStiff{1e20};
  for (int i{0}; i < n; ++i) {
    const common::PenetrationAsPointPair& pair =
        penetration_pairs[preservedPairsIdx[i]];
    auto* geometry_A = engine.queryGeometryPoolById(pair.id_A);
    auto* geometry_B = engine.queryGeometryPoolById(pair.id_B);
    const core::PropMap& pair_prop = manager.getPropMapOrDefault(
        {geometry_A->material(), geometry_B->material()});
    double k = pair_prop.getPropValueOrDefault("k", 2e8);
    double d = pair_prop.getPropValueOrDefault("d", 5e3);
    stiffness[i] = k;
    minStiff = minStiff < k ? minStiff : k;
    damping[i] = d;
  }
  double stiffScale = std::pow(10, -floor(std::log10(minStiff) / 2));
  for (int i{0}; i < n; ++i) {
    const common::PenetrationAsPointPair& pair =
        penetration_pairs[preservedPairsIdx[i]];
    // 因为检测的碰撞信息是相对于物体 A 的，所以相对速度是 B 相对于 A 的
    std::array<double, 3> v_contact;
    engine.cptContactVelocityB2A(pair, T_C_vec[preservedPairsIdx[i]],
                                 v_contact);
    stiffness[i] *= stiffScale;
    // double vn = enginePtr->cptProximityVelocity(pair);
    x0[i] = pair.depth / stiffScale;
    // 穿透速度 = - B的速度相对于A
    x0[n + i] = -v_contact[2];
    v0[3 * i] = v_contact[0];
    v0[3 * i + 1] = v_contact[1];
    v0[3 * i + 2] = v_contact[2];
  }
  return stiffScale;
}
auto cptKdMatrix(sire::Size n, const double* stiffness, const double* damping,
                 double* kdMatrix) -> void {
  // 假设ground不会与ground相撞。A指向B
  for (Size i{0}, lineIdx{0}; i < n; ++i) {
    Size lineBegin = lineIdx * 2 * n;
    kdMatrix[lineBegin + i] = -stiffness[i];
    kdMatrix[lineBegin + n + i] = -damping[i];  // a1A
    kdMatrix[lineBegin + 2 * n + i] = stiffness[i];
    kdMatrix[lineBegin + 3 * n + i] = damping[i];  // a1B
    lineIdx += 2;
  }
}
auto cptInvCpi(sire::Size n, sire::Size cpiWidth, double minDamp,
               const LhsVariableType* variableType, double* cpi,
               double* kdMatrix, double* fext, double* invCpi) -> void {
  // ----------------------------------------------------
  // ------------ 接触点惯量矩阵求逆 ---------------------
  // ----------------------------------------------------
  // 如果没有逆矩阵，就使用公式进行替换
  double divider = std::pow(10, floor(std::log10(minDamp)));
  std::vector<double> u(cpiWidth * cpiWidth), tau(cpiWidth), tau2(cpiWidth);
  std::vector<aris::Size> p(cpiWidth);
  aris::Size rank;
  aris::dynamic::s_householder_utp(cpiWidth, cpiWidth, cpi, u.data(),
                                   tau.data(), p.data(), rank);
  // std::cout << rank << " " << cpiWidth << std::endl;
  // aris::dynamic::dsp(cpiWidth, cpiWidth, cpi);
  if (rank != cpiWidth) {
    using MatrixXdRowMajor = Eigen::Matrix<double, -1, -1, Eigen::RowMajor>;
    MatrixXdRowMajor cpiMatrix =
        Eigen::Map<MatrixXdRowMajor>(cpi, cpiWidth, cpiWidth);
    MatrixXdRowMajor kdEigenMatrix =
        Eigen::Map<MatrixXdRowMajor>(kdMatrix, cpiWidth, 2 * n);
    do {
      // std::cout << rank << " " << cpiWidth << cpiMatrix << std::endl <<
      // kdEigenMatrix << std::endl;
      Eigen::MatrixXd nullSpace = cpiMatrix.fullPivLu().kernel();
      nullSpace.transposeInPlace();
      std::vector<aris::Size> replaceRowIdx;
      for (aris::Size i{0}; i < nullSpace.rows(); ++i) {
        for (aris::Size j{0}; j < nullSpace.cols(); ++j) {
          if (nullSpace(i, j) != 0 &&
              find(replaceRowIdx.begin(), replaceRowIdx.end(), j) ==
                  replaceRowIdx.end()) {
            replaceRowIdx.push_back(j);
            break;
          }
        }
      }
      SIRE_DEMAND(replaceRowIdx.size() == nullSpace.rows());
      Eigen::MatrixXd temp = nullSpace * kdEigenMatrix / divider;
      // temp 对应的变量为 delta1 ... deltan
      // 但是cpiMatrix对应的变量为 a1A a1B ... deltac ... anA anB
      // 需要对temp进行相应的拆分。
      for (int i{0}; i < replaceRowIdx.size(); ++i) {
        int idx = replaceRowIdx[i];
        // 等式左右两边共同除以damping的数量级，
        // 防止与矩阵中的其他元素数量级差的过大，导致矩阵条件数过大，求逆丧失的精度过大
        // 对于a1A 与 a1B需要进行拆分。
        for (sire::Size j{0}, cpiColIdx{0}; j < n; ++j) {
          if (variableType[j] == LhsVariableType::OneDelta) {
            cpiMatrix(idx, cpiColIdx) = temp(i, n + j);
            ++cpiColIdx;
          } else {
            cpiMatrix(idx, cpiColIdx) = temp(i, n + j);
            ++cpiColIdx;
            cpiMatrix(idx, cpiColIdx) = -temp(i, n + j);
            ++cpiColIdx;
          }
        }
        kdEigenMatrix(idx, Eigen::seqN(0, n)).setZero();
        kdEigenMatrix(idx, Eigen::seqN(n, n)) = -temp(i, Eigen::seqN(0, n));
        fext[idx] = 0;
      }
      aris::dynamic::s_householder_utp(cpiWidth, cpiWidth, cpiMatrix.data(),
                                       u.data(), tau.data(), p.data(), rank);
    } while (rank != cpiWidth);
    aris::dynamic::s_mc(cpiWidth, 2 * n, kdEigenMatrix.data(), kdMatrix);
    aris::dynamic::s_mc(cpiWidth, cpiWidth, cpiMatrix.data(), cpi);
  }

  aris::dynamic::s_householder_utp2pinv(cpiWidth, cpiWidth, rank, u.data(),
                                        tau.data(), p.data(), invCpi,
                                        tau2.data());
}

// clang-format off
/// @brief 
/// TODO: 处理一下由于cpi无逆情况下公式的delta1到a1b - a1a的分解。
/// 目前对于cpi无逆情况下的各种认识
/// 1. 矩阵 A 可能会出现无解的情况，无解由于delta与其他的线性相关，A 丧失自由度，不对，不管是否拆分，矩阵A因为下半部分的原因总会丢失自由度。
/// 
/// @param[] penetration_pairs 
/// @param[] T_C_vec 
/// @param[] stiffness 
/// @param[] damping 
/// @param[] A 
/// @param[] b
// clang-format on
auto cptDAECoeff(sire::physics::PhysicsEngine& engine, sire::Size n,
                 const double* stiffness, const double* damping,
                 double stiffScale, double* accelExt, double* invCpi, double* A,
                 double* b) -> void {
  sire::Size n2{2 * n};
  std::vector<double> kdMatrix(n2 * n2);
  // 假设ground不会与ground相撞。A指向B
  cptKdMatrix(n, stiffness, damping, kdMatrix.data());

  std::vector<double> temp1(n2 * n2, 0), temp2(n2, 0);
  // invcpi * kdMatrix
  aris::dynamic::s_mm(n2, n2, n2, invCpi, kdMatrix.data(), temp1.data());
  // ---------------------------------------------------------------
  // T = [1 -1 0 ... 0 0  0]
  //     [0  0 1 -1 ...0  0]
  //     [.  . .  . ....  .]
  //     [0 0 0 0 .... 1 -1] n * 2n
  // shrinked inverse cpi matrix = T * I^-1 * T'
  // ---------------计算缩小后的 cpi 矩阵  n x n ---------------------
  // 直接计算 A 与 b，通过中间变量储存一些中间值
  // ----------------------------------------------------------------
  for (sire::Size i{0}; i < n; ++i) {
    b[i] = 0;
    b[n + i] = accelExt[2 * i] - accelExt[2 * i + 1];
  }
  // A.resize(4 * n * n, 0);
  // TODO: 不存在两个ground相撞
  for (Size i{0}, lineIdx{0}; i < n; ++i) {
    A[n2 * i + n + i] = 1 / stiffScale;
    for (Size j{0}; j < n2; ++j) {
      A[n2 * (i + n) + j] =
          temp1[lineIdx * n2 + j] - temp1[(lineIdx + 1) * n2 + j];
    }
    lineIdx += 2;
  }
}
// clang-format off
/// @brief 计算多点接触微分公式 x(t) 的结果
/// 
/// expm(A * t) * (x0' + A \ b') - A \ b';
/// @param[in] n 2倍接触点数目（状态空间大小）
/// @param[in] A n x n 一阶微分动力方程状态转移矩阵
/// @param[in] t 1 需要计算的时间
/// @param[in] b n x 1 微分动力方程非齐次项
/// @param[in] x0 n x 1 微分动力方程初始状态
/// @param[out] x n x 1 输出状态
// clang-format on
auto cptFormulaX(sire::Size n, const double* A, double t, const double* b,
                 const double* x0, double* x) -> void {
  std::vector<double> u(n * n), tau(n), aldb(n);
  std::vector<aris::Size> p(n);
  aris::Size rank;
  aris::dynamic::s_householder_utp(n, n, A, u.data(), tau.data(), p.data(),
                                   rank);
  // 不需要求逆，直接 A = Q R 分解后得到 A \ b 的结果 A left divide b aldb
  // A x = b -> Q R x = b -> x = R^-1 *  Q' * b
  aris::dynamic::s_householder_utp_sov(n, n, 1, rank, u.data(), tau.data(),
                                       p.data(), b,
                                       aldb.data());      // aldb = A \ b
  aris::dynamic::s_vc(n, x0, tau.data());                 // tau = x0;
  aris::dynamic::s_va(n, aldb.data(), tau.data());        // tau = tau + A \ b
  aris::dynamic::s_mc(n, n, t, A, u.data());              // u = At
  core::screw::matrix_exp_pade(n, u.data(), u.data());    // u = e^At
  aris::dynamic::s_mm(n, 1, n, u.data(), tau.data(), x);  // e^At (x0 + A \ b)
  aris::dynamic::s_vs(n, aldb.data(), x);                 // x = x - A \ b
}
// clang-format off
/// @brief 计算多点接触状态积分 ix(tc - t0) 的结果
/// 
/// 用于计算接触力在接触时间内的冲量并用于求平均力。
/// 注意ix(t)并没有物理意义，因为我们没有初始状态，无法求常数项
/// 不定积分没有意义，但是定积分有意义
/// (A \ ((expm(A * tc) - expm(A * t0)) * (x0' + A \ b'))) -
/// (A \ b') * (tc - t0)
/// @param[in] n 2倍接触点数目（状态空间大小）
/// @param[in] A n x n 一阶微分动力方程状态转移矩阵
/// @param[in] t0 1 时间片开始时间
/// @param[in] tc 1 最小状态转换时间
/// @param[in] b n x 1 微分动力方程非齐次项
/// @param[in] x0 n x 1 微分动力方程初始状态
/// @param[out] ix n x 1 输出状态
// clang-format on
auto cptFormulaIXdt(sire::Size n, const double* A, double t0, double tc,
                    const double* b, const double* x0, double* ix) -> void {
  std::vector<double> eAt(n * n), u(n * n), tau(n), aldb(n);
  aris::dynamic::s_mc(n, n, t0, A, u.data());               // u = A * t0
  core::screw::matrix_exp_pade(n, u.data(), u.data());      // u = e^At0
  aris::dynamic::s_mc(n, n, tc, A, eAt.data());             // eAt = A * tc
  core::screw::matrix_exp_pade(n, eAt.data(), eAt.data());  // eAt = e^Atc
  aris::dynamic::s_ms(n, n, u.data(), eAt.data());          // eAt = eAt - u
  std::vector<aris::Size> p(n);
  aris::Size rank;
  aris::dynamic::s_householder_utp(n, n, A, u.data(), tau.data(), p.data(),
                                   rank);
  // 不需要求逆，直接 A = Q R 分解后得到 A \ b 的结果 A left divide b aldb
  // A x = b -> Q R x = b -> x = R^-1 *  Q' * b
  aris::dynamic::s_householder_utp_sov(n, n, 1, rank, u.data(), tau.data(),
                                       p.data(), b,
                                       aldb.data());  // aldb = A \ b
  aris::dynamic::s_vc(n, x0, ix);                     // ix = x0;
  aris::dynamic::s_va(n, aldb.data(), ix);            // ix = ix + aldb
  std::vector<double> eAtxpb(n);
  aris::dynamic::s_mm(n, 1, n, eAt.data(), ix,
                      eAtxpb.data());  // eAtxpb = eAt * ix
  aris::dynamic::s_householder_utp_sov(n, n, 1, rank, u.data(), tau.data(),
                                       p.data(), eAtxpb.data(),
                                       ix);          // ix = A \ eAtxpb
  aris::dynamic::s_va(n, t0 - tc, aldb.data(), ix);  // ix = ix - (A \ b)t
}
// clang-format off
/// @brief 计算多点接触微分公式 x(t) 的结果
/// 
/// expm(Ab * t) * x01';
/// @param[in] n 2倍接触点数目（状态空间大小）
/// @param[in] Ab n x n 一阶微分动力方程状态转移矩阵
/// @param[in] t 1 需要计算的时间
/// @param[in] x01 (n + 1) x 1 微分动力方程初始状态
/// @param[out] x1t (n + 1) x 1 输出状态
// clang-format on
auto cptFormulaXComposeAb(sire::Size n, const double* Ab, double t,
                          const double* x01, double* x1t) -> void {
  std::vector<double> temp(n * n);
  aris::dynamic::s_mc(n, n, t, Ab, temp.data());              // temp = At
  core::screw::matrix_exp_pade(n, temp.data(), temp.data());  // temp = e^At
  aris::dynamic::s_mm(n, 1, n, temp.data(), x01, x1t);        // e^At * x01
}
// clang-format off
/// @brief 计算多点接触状态积分 ix(tc - t0) 的结果
/// 
/// 用于计算接触力在接触时间内的冲量并用于求平均力。
/// 注意ix(t)并没有物理意义，因为我们没有初始状态，无法求常数项
/// 不定积分没有意义，但是定积分有意义
/// (A \ ((expm(A * tc) - expm(A * t0)) * (x0' + A \ b'))) -
/// (A \ b') * (tc - t0)
/// @param[in] n 2倍接触点数目（状态空间大小）
/// @param[in] A n x n 一阶微分动力方程状态转移矩阵
/// @param[in] t0 1 时间片开始时间
/// @param[in] tc 1 最小状态转换时间
/// @param[in] b n x 1 微分动力方程非齐次项
/// @param[in] x0 n x 1 微分动力方程初始状态
/// @param[out] ix n x 1 输出状态
// clang-format on
auto cptFormulaIXdtComposeAbx0(sire::Size n, const double* Abx0, double t0,
                               double tc, double* ixdt) -> void {
  std::vector<double> temp(n * n);
  aris::dynamic::s_mc(n, n, tc, Abx0, temp.data());  // temp = Abx0 * tc
  core::screw::matrix_exp_pade(n, temp.data(),
                               temp.data());  // temp = e^Abx0 tc
  for (sire::Size i{0}; i < n - 2; ++i) {
    ixdt[i] = temp[n * i + n - 1];
  }
  aris::dynamic::s_mc(n, n, t0, Abx0, temp.data());  // temp = Abx0 * t0
  core::screw::matrix_exp_pade(n, temp.data(), temp.data());  // eAt = e^Atc
  for (sire::Size i{0}; i < n - 2; ++i) {
    ixdt[i] -= temp[n * i + n - 1];  // result = eatc - eat0;
  }
}
// clang-format off
/// @brief 根据一阶矩阵微分动力方程的状态转移矩阵性质求最近的状态转换时刻
/// 
/// @param[in] nContact 接触点数目
/// @param[in] A 2n x 2n 状态转移矩阵
/// @param[in] b 2n x 1 非齐次项
/// @param[in] x0 2n x 1 初始状态
/// @param[in] tolerance 周期数去重精度/求根精度/结束条件
/// @param[in] maxIter 最大循环次数
/// @return double 最近的状态转移时刻，用于求平均力
// clang-format on
auto findMinRootBisection(sire::Size nContact, const double* A, const double* b,
                          const double* x0, double tolerance,
                          sire::Size maxIter) -> double {
  const sire::Size n2 = 2 * nContact;
  // 因为矩阵 A 经常无逆，所以使用其增广形式 [A b; 0 0] 作为状态转移矩阵，x0 =
  // [x0 1] 作为初始状态（求微分方程解的微分部分）
  std::vector<double> Ab((n2 + 1) * (n2 + 1), 0), x01(n2 + 1);
  sire::core::screw::matrixVectorComposeBack(n2, A, b, Ab.data());
  std::copy(x0, x0 + n2, x01.data());
  x01[n2] = 1;
  // 处理矩阵A的eigenvalue，用来寻找兴趣点
  Eigen::MatrixXd Ab_eig = Eigen::Map<Eigen::MatrixXd>(
      const_cast<double*>(Ab.data()), n2 + 1, n2 + 1);
  Eigen::EigenSolver<Eigen::MatrixXd> es(Ab_eig, false);
  Eigen::VectorXd absImgPrt = es.eigenvalues().imag().cwiseAbs();
  // sin(alpha t)
  std::vector<double> alphaVec;
  // 去掉 0 的 complex part
  std::copy_if(absImgPrt.data(), absImgPrt.data() + n2 + 1,
               std::back_inserter(alphaVec),
               [tolerance](double i) { return i > tolerance; });
  std::sort(alphaVec.begin(), alphaVec.end(), std::greater<double>());
  // 去掉重复的值
  alphaVec.erase(std::unique(alphaVec.begin(), alphaVec.end(),
                             [tolerance](double a, double b) {
                               return std::abs(a - b) < tolerance;
                             }),
                 alphaVec.end());
  // SIRE_ASSERT(alphaVec.size() != 0);
  if (alphaVec.size() == 0) {
    DLOG(DEBUG) << "Contact without split, no imaginary part";
    return -1;
  }
  // double temp[2]{sire::PI, 2 * sire::PI};
  std::vector<double> pois(4 * alphaVec.size());
  for (int i{0}; i < alphaVec.size(); ++i) {
    double temp = sire::PI / alphaVec[i];
    pois[2 * i] = 0.5 * temp;
    pois[2 * i + 1] = temp;
    pois[2 * i + 2] = 1.5 * temp;
    pois[2 * i + 3] = 2 * temp;
  }

  std::sort(pois.begin(), pois.end());
  double lowerBound = 1e-12;
  // TODO: 可以设置为仿真的默认步长，超过默认步长的穿透没必要单独解接触时间了
  double upperBound = 0.1;
  std::vector<double> x1t(n2 + 1);
  auto depthEnd = x1t.begin() + nContact;
  bool negativeDepthExists = false;
  for (double poi : pois) {
    cptFormulaXComposeAb(n2 + 1, Ab.data(), poi, x01.data(), x1t.data());
    if (std::find_if(x1t.begin(), depthEnd, [](double x) { return x < 0; }) ==
        depthEnd) {
      lowerBound = poi;  // x向量元素全部大于零
      continue;
    } else {
      upperBound = poi;  // 存在负数
      negativeDepthExists = true;
      break;
    }
  }
  if (!negativeDepthExists) {
    DLOG(WARNING) << "Contact without split, negative depth not exists";
    return -1;
  }
  // bisection
  double m{-1};
  int i{0};
  for (; i < maxIter; ++i) {
    m = (lowerBound + upperBound) / 2;
    // LOG_IF(aris::dynamic::s_is_equal(m, 0.1, tolerance), DEBUG) <<
    cptFormulaXComposeAb(n2 + 1, Ab.data(), m, x01.data(), x1t.data());
    // TODO: 添加 ub - lb过小情况的判断，否则每次都要跑完整个maxIter
    // if (upperBound - lowerBound < tolerance) {
    // if (aris::dynamic::s_is_equal(m, 0.1, tolerance * 5))
    //   m = 0.0001;
    // else
    //   m = -1;
    // }
    // 有数小于 tol (可以当作根)
    if (std::find_if(x1t.begin(), depthEnd, [tolerance](double x) {
          return std::abs(x) < tolerance;
        }) != depthEnd) {
      // 没有比 -tol 更小的值了
      if (std::find_if(x1t.begin(), depthEnd, [tolerance](double x) {
            return x < -tolerance;
          }) == depthEnd) {
        break;
      }
    }
    if (std::find_if(x1t.begin(), depthEnd, [](double x) { return x < 0; }) ==
        depthEnd) {
      lowerBound = m;  // x向量元素全部大于零
    } else {
      upperBound = m;  // 存在负数
    }
  }
  if (i == maxIter) m = -1;
  return m;
}

/// @brief 通过积分公式计算接触平均力
///
/// f (k ix) + (d x) dt / (tc - t0)
/// @param[in] nContact
/// @param[in] A
/// @param[in] b
/// @param[in] x0
/// @param[in] t0
/// @param[in] tc
/// @param[in] stiffness
/// @param[in] damping
/// @param[in] avgFce
auto cptAvgContactFce(sire::Size nContact, const double* A, const double* b,
                      const double* x0, double t0, double tc,
                      const double* stiffness, const double* damping,
                      double* avgFce) -> void {
  sire::Size n2 = 2 * nContact;
  sire::Size n22 = 2 * nContact + 2;
  std::vector<double> ixdt(n2);
  std::vector<double> Abx0(n22 * n22, 0);
  for (sire::Size i{0}; i < n2; ++i) {
    for (sire::Size j{0}; j < n2; ++j) {
      Abx0[n22 * i + j] = A[n2 * i + j];
    }
    Abx0[n22 * i + n2] = b[i];
    Abx0[n22 * i + n2 + 1] = x0[i];
  }
  Abx0[n22 * n2 + n2 + 1] = 1;
  cptFormulaIXdtComposeAbx0(n22, Abx0.data(), t0, tc, ixdt.data());
  for (int i{0}; i < nContact; ++i) {
    avgFce[i] =
        (stiffness[i] * ixdt[i] + damping[i] * ixdt[i + nContact]) / (tc - t0);
  }
}
auto contactPairsTooClosedFlag(
    const sire::physics::common::PenetrationAsPointPair& pairA,
    const sire::physics::common::PenetrationAsPointPair& pairB) -> bool {
  // 0.5 degree and 5cm difference.
  return pairA.nhat_AB_W.cross(pairB.nhat_AB_W).norm() < 1e-2 &&
         (pairA.p_WC - pairB.p_WC).cwiseAbs().sum() < 2e-2;
}
auto preprocessContactInfo(
    sire::physics::PhysicsEngine& engine,
    const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    const std::vector<sire::Size>& preservedPairsIdx, sire::PartId* prtIdVector)
    -> void {
  auto modelPtr = engine.currentModel();
  SIRE_ASSERT(modelPtr != nullptr);
  const sire::Size n = preservedPairsIdx.size();
  for (sire::Size i{0}; i < n; ++i) {
    auto& pair = penetration_pairs[preservedPairsIdx[i]];
    const geometry::CollidableGeometry* geometry_A_ptr =
        engine.queryGeometryPoolById(pair.id_A);
    const geometry::CollidableGeometry* geometry_B_ptr =
        engine.queryGeometryPoolById(pair.id_B);
    SIRE_DEMAND(geometry_A_ptr != nullptr);
    SIRE_DEMAND(geometry_B_ptr != nullptr);
    // 标记 ground 相关的idx，计算cpi的真实大小
    // 默认是两个加速度a
    prtIdVector[2 * i] = geometry_A_ptr->partId();
    prtIdVector[2 * i + 1] = geometry_B_ptr->partId();
  }
}
auto cptInverseNormalCpiMatrix(
    aris::dynamic::Model& model,
    const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    const std::vector<std::array<double, 16>>& T_C_vec,
    const std::vector<sire::Size>& preservedPairsIdx,
    const sire::PartId* prtIdVector, double* accelExt, double* invCpi) -> void {
  auto init_interaction = [](aris::dynamic::Interaction& interaction,
                             aris::dynamic::Model* m) -> void {
    if (interaction.prtNameM().empty() && interaction.prtNameN().empty() &&
        interaction.makNameI().empty() && interaction.makNameJ().empty())
      return;

    auto find_part = [m](std::string_view name) -> aris::dynamic::Part* {
      auto found = std::find_if(
          m->partPool().begin(), m->partPool().end(),
          [name](const auto& part) -> bool { return part.name() == name; });
      return found == m->partPool().end() ? nullptr : &*found;
    };

    auto find_marker = [](aris::dynamic::Part* part,
                          std::string_view name) -> aris::dynamic::Marker* {
      auto found = std::find_if(
          part->markerPool().begin(), part->markerPool().end(),
          [name](const auto& marker) -> bool { return marker.name() == name; });
      return found == part->markerPool().end() ? nullptr : &*found;
    };

    auto prt_m = find_part(interaction.prtNameM());
    auto mak_i = find_marker(prt_m, interaction.makNameI());
    auto prt_n = find_part(interaction.prtNameN());
    auto mak_j = find_marker(prt_n, interaction.makNameJ());

    interaction.setMakI(&*mak_i);
    interaction.setMakJ(&*mak_j);
  };
  // 初始化一些重复使用的变量
  auto& forcePool = model.forcePool();
  auto& partPool = model.partPool();
  sire::Size testForceIdxOffset = forcePool.size();

  const sire::Size n = preservedPairsIdx.size();
  // 给力并计算质量矩阵
  const double fceValue = 10.0;
  double testFce[3] = {0, 0, fceValue};
  for (Size i{0}; i < n; ++i) {
    for (Size j{0}; j < 2; ++j) {
      // add generalForce to forcePool() in Model and init
      auto& fce = forcePool.add<aris::dynamic::GeneralForce>(
          std::string("test_f" + j
                          ? "b"
                          : "a" + std::to_string(i + testForceIdxOffset)),
          &partPool.at(prtIdVector[2 * i + j]).markerPool().at(0),
          &partPool.at(model.ground().id()).markerPool().at(0));
      fce.resetModel(&model);
      fce.setFce(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());
      // force id 可以先不管
      init_interaction(fce, &model);
    }
  }
  for (Size iContact{0}, cpiLineIdx{0}; iContact < n; ++iContact) {
    for (Size i2{0}; i2 < 2; ++i2) {
      auto& gf = dynamic_cast<aris::dynamic::GeneralForce&>(
          forcePool.at(2 * iContact + i2 + testForceIdxOffset));
      double fs[6];
      sire::core::screw::s_fpm2fs(
          testFce, T_C_vec[preservedPairsIdx[iContact]].data(), fs);
      gf.setFce(fs);
      if (model.forwardDynamics())
        std::cout << "forward dynamic failed" << std::endl;

      for (Size jContact{0}, cpiColIdx{0}; jContact < n; ++jContact) {
        double ap_o[3]{0}, res[3]{0};
        const double* contactPosition =
            penetration_pairs[preservedPairsIdx[jContact]].p_WC.data();
        std::array<double, 6> as;
        for (Size j2 = 0; j2 < 2; ++j2) {
          auto& prt = partPool[prtIdVector[2 * jContact + j2]];
          prt.getAs(as.data());
          aris::dynamic::s_as2ap(prt.vs(), as.data(), contactPosition, ap_o);
          aris::dynamic::s_inv_pm_dot_v3(
              T_C_vec[preservedPairsIdx[jContact]].data(), ap_o, res);
          invCpi[cpiLineIdx * 2 * n + cpiColIdx] = core::screw::s_safe_div(
              res[2] - accelExt[cpiColIdx], fceValue, 1e-4);
          ++cpiColIdx;
        }
      }
      aris::dynamic::s_fill(1, 6, 0, const_cast<double*>(gf.fce()));
      ++cpiLineIdx;
    }
  }
  for (sire::Size i{0}; i < 2 * n; ++i) {
    forcePool.pop_back();
  }
}
auto cptInverseCpiMatrix(
    aris::dynamic::Model& model,
    const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    const std::vector<std::array<double, 16>>& T_C_vec,
    const std::vector<sire::Size>& preservedPairsIdx,
    const sire::PartId* prtIdVector, double* accelExt, double* invCpi) -> void {
  auto init_interaction = [](aris::dynamic::Interaction& interaction,
                             aris::dynamic::Model* m) -> void {
    if (interaction.prtNameM().empty() && interaction.prtNameN().empty() &&
        interaction.makNameI().empty() && interaction.makNameJ().empty())
      return;

    auto find_part = [m](std::string_view name) -> aris::dynamic::Part* {
      auto found = std::find_if(
          m->partPool().begin(), m->partPool().end(),
          [name](const auto& part) -> bool { return part.name() == name; });
      return found == m->partPool().end() ? nullptr : &*found;
    };

    auto find_marker = [](aris::dynamic::Part* part,
                          std::string_view name) -> aris::dynamic::Marker* {
      auto found = std::find_if(
          part->markerPool().begin(), part->markerPool().end(),
          [name](const auto& marker) -> bool { return marker.name() == name; });
      return found == part->markerPool().end() ? nullptr : &*found;
    };

    auto prt_m = find_part(interaction.prtNameM());
    auto mak_i = find_marker(prt_m, interaction.makNameI());
    auto prt_n = find_part(interaction.prtNameN());
    auto mak_j = find_marker(prt_n, interaction.makNameJ());

    interaction.setMakI(&*mak_i);
    interaction.setMakJ(&*mak_j);
  };
  // 初始化一些重复使用的变量
  auto& forcePool = model.forcePool();
  auto& partPool = model.partPool();
  sire::Size testForceIdxOffset = forcePool.size();

  const sire::Size n = preservedPairsIdx.size();
  // 给力并计算质量矩阵
  const double fceValue = 10.0;
  double testFce[3] = {0, 0, fceValue};
  for (Size i{0}; i < n; ++i) {
    for (Size j{0}; j < 2; ++j) {
      // add generalForce to forcePool() in Model and init
      auto& fce = forcePool.add<aris::dynamic::GeneralForce>(
          std::string("test_f" + j
                          ? "b"
                          : "a" + std::to_string(i + testForceIdxOffset)),
          &partPool.at(prtIdVector[2 * i + j]).markerPool().at(0),
          &partPool.at(model.ground().id()).markerPool().at(0));
      fce.resetModel(&model);
      fce.setFce(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());
      // force id 可以先不管
      init_interaction(fce, &model);
    }
  }
  for (Size iContact{0}, cpiLineIdx{0}; iContact < n; ++iContact) {
    for (Size i2{0}; i2 < 2; ++i2) {
      auto& gf = dynamic_cast<aris::dynamic::GeneralForce&>(
          forcePool.at(2 * iContact + i2 + testForceIdxOffset));
      double fs[6];
      sire::core::screw::s_fpm2fs(
          testFce, T_C_vec[preservedPairsIdx[iContact]].data(), fs);
      gf.setFce(fs);
      if (model.forwardDynamics())
        std::cout << "forward dynamic failed" << std::endl;

      for (Size jContact{0}, cpiColIdx{0}; jContact < n; ++jContact) {
        double ap_o[3]{0}, res[3]{0};
        const double* contactPosition =
            penetration_pairs[preservedPairsIdx[jContact]].p_WC.data();
        std::array<double, 6> as;
        for (Size j2 = 0; j2 < 2; ++j2) {
          auto& prt = partPool[prtIdVector[2 * jContact + j2]];
          prt.getAs(as.data());
          aris::dynamic::s_as2ap(prt.vs(), as.data(), contactPosition, ap_o);
          aris::dynamic::s_inv_pm_dot_v3(
              T_C_vec[preservedPairsIdx[jContact]].data(), ap_o, res);
          invCpi[cpiLineIdx * 2 * n + cpiColIdx] = core::screw::s_safe_div(
              res[2] - accelExt[cpiColIdx], fceValue, 1e-4);
          ++cpiColIdx;
        }
      }
      aris::dynamic::s_fill(1, 6, 0, const_cast<double*>(gf.fce()));
      ++cpiLineIdx;
    }
  }
  for (sire::Size i{0}; i < 2 * n; ++i) {
    forcePool.pop_back();
  }
}
auto cptCpiMatrix(
    aris::dynamic::Model& model,
    const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    const std::vector<std::array<double, 16>>& T_C_vec,
    const std::vector<sire::Size>& preservedPairsIdx,
    const sire::PartId* prtIdVector, sire::Size cpiWidth, const int* groundFlag,
    double* cpi) -> void {
  auto init_interaction = [](aris::dynamic::Interaction& interaction,
                             aris::dynamic::Model* m) -> void {
    if (interaction.prtNameM().empty() && interaction.prtNameN().empty() &&
        interaction.makNameI().empty() && interaction.makNameJ().empty())
      return;

    auto find_part = [m](std::string_view name) -> aris::dynamic::Part* {
      auto found = std::find_if(
          m->partPool().begin(), m->partPool().end(),
          [name](const auto& part) -> bool { return part.name() == name; });
      return found == m->partPool().end() ? nullptr : &*found;
    };

    auto find_marker = [](aris::dynamic::Part* part,
                          std::string_view name) -> aris::dynamic::Marker* {
      auto found = std::find_if(
          part->markerPool().begin(), part->markerPool().end(),
          [name](const auto& marker) -> bool { return marker.name() == name; });
      return found == part->markerPool().end() ? nullptr : &*found;
    };

    auto prt_m = find_part(interaction.prtNameM());
    auto mak_i = find_marker(prt_m, interaction.makNameI());
    auto prt_n = find_part(interaction.prtNameN());
    auto mak_j = find_marker(prt_n, interaction.makNameJ());

    interaction.setMakI(&*mak_i);
    interaction.setMakJ(&*mak_j);
  };
  // 初始化一些重复使用的变量
  auto& forcePool = model.forcePool();
  auto& partPool = model.partPool();
  sire::Size testForceIdxOffset = forcePool.size();
  // 在修改 forcePool 之前记录forcePool的 active 状态
  FceActiveStateRecorder recorder(&model);
  // 将在 forcePool 中的力全部 deactivate
  for (auto& fce : forcePool) fce.activate(false);

  const sire::Size n = preservedPairsIdx.size();
  // 给力并计算质量矩阵
  const double fceValue1 = 10.0;
  const double fceValue2 = 50.0;
  double testFce1[3] = {0, 0, fceValue1};
  double testFce2[3] = {0, 0, fceValue2};
  double* fces[2] = {testFce1, testFce2};
  const double* gravityAs = model.environment().gravity();
  for (Size i{0}; i < n; ++i) {
    for (Size j{0}; j < 2; ++j) {
      // add generalForce to forcePool() in Model and init
      auto& fce = forcePool.add<aris::dynamic::GeneralForce>(
          std::string("test_f" + j
                          ? "b"
                          : "a" + std::to_string(i + testForceIdxOffset)),
          &partPool.at(prtIdVector[2 * i + j]).markerPool().at(0),
          &partPool.at(model.ground().id()).markerPool().at(0));
      fce.resetModel(&model);
      fce.setFce(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());
      // force id 可以先不管
      init_interaction(fce, &model);
    }
  }
  for (Size iContact{0}, cpiLineIdx{0}; iContact < n; ++iContact) {
    for (Size i2{0}; i2 < 2; ++i2) {
      if (groundFlag[2 * iContact + i2]) continue;
      auto& gf = dynamic_cast<aris::dynamic::GeneralForce&>(
          forcePool.at(2 * iContact + i2 + testForceIdxOffset));
      std::vector<double> az1(cpiWidth);
      for (sire::Size fceIdx{0}; fceIdx < 2; ++fceIdx) {
        double fs[6];
        sire::core::screw::s_fpm2fs(
            fces[fceIdx], T_C_vec[preservedPairsIdx[iContact]].data(), fs);
        gf.setFce(fs);
        if (model.forwardDynamics())
          std::cout << "forward dynamic failed" << std::endl;

        for (Size jContact{0}, cpiColIdx{0}; jContact < n; ++jContact) {
          double ap_o[3]{0}, res[3]{0};
          const double* contactPosition =
              penetration_pairs[preservedPairsIdx[jContact]].p_WC.data();
          std::array<double, 6> as;
          for (Size j2 = 0; j2 < 2; ++j2) {
            if (groundFlag[2 * jContact + j2]) continue;
            auto& prt = partPool[prtIdVector[2 * jContact + j2]];
            prt.getAs(as.data());
            aris::dynamic::s_vs(6, gravityAs, as.data());
            aris::dynamic::s_as2ap(prt.vs(), as.data(), contactPosition, ap_o);
            aris::dynamic::s_inv_pm_dot_v3(
                T_C_vec[preservedPairsIdx[jContact]].data(), ap_o, res);
            if (fceIdx) {
              cpi[cpiLineIdx * cpiWidth + cpiColIdx] = core::screw::s_safe_div(
                  fceValue2 - fceValue1, res[2] - az1[cpiColIdx], 1e-4);
            } else {
              az1[cpiColIdx] = res[2];
            }
            ++cpiColIdx;
          }
        }
      }
      aris::dynamic::s_fill(1, 6, 0, const_cast<double*>(gf.fce()));
      ++cpiLineIdx;
    }
  }
  for (sire::Size i{0}; i < 2 * n; ++i) {
    forcePool.pop_back();
  }
}
auto filterPairsAndPreprocessInfo(
    sire::physics::PhysicsEngine& engine,
    const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    const std::vector<std::array<double, 16>>& T_C_vec,
    std::vector<sire::Size>& preservedPairsIdx,
    std::vector<sire::PartId>& prtIdVector, std::vector<double>& accelExt,
    std::vector<double>& invCpiResult) -> void {
  std::vector<std::pair<sire::Size, sire::Size>> closedContactPointPairs;

  for (sire::Size i{0}; i < penetration_pairs.size(); ++i) {
    if (penetration_pairs[i].depth >= 0) {
      preservedPairsIdx.push_back(i);
    }
  }
  sire::Size n{preservedPairsIdx.size()};
  sire::Size n2{2 * n};
  prtIdVector.resize(n2, 0);
  preprocessContactInfo(engine, penetration_pairs, preservedPairsIdx,
                        prtIdVector.data());
  accelExt.resize(n2, 0);
  engine.activateContactForce(false);
  auto modelPtr = engine.currentModel();
  SIRE_ASSERT(modelPtr != nullptr);
  cptAccelExtVector(*modelPtr, penetration_pairs, T_C_vec, preservedPairsIdx,
                    prtIdVector.data(), accelExt.data());

  invCpiResult.resize(n2 * n2, 0);
  cptInverseNormalCpiMatrix(*modelPtr, penetration_pairs, T_C_vec,
                            preservedPairsIdx, prtIdVector.data(),
                            accelExt.data(), invCpiResult.data());
  engine.activateContactForce(true);
}

struct AverageForceContactSolver::Imp {
  unique_ptr<core::MaterialManager> material_manager_;
  // 消耗系数
  double default_cr_;
  // 摩擦系数
  double default_cof_;
  // 速度阈值 velocity threshold
  double default_tv_;
  double default_k_;
  double default_d_;
  ContactSolverResult prevResult;

  auto combineContactMass(double m1, double m2) -> double {
    auto safe_div = [](double number, double denominator) -> double {
      return denominator == 0.0 ? 0.0 : number / denominator;
    };
    return safe_div(m1 * m2, m1 + m2);
  }
  auto combineContactStiffness(double k1, double k2) -> double {
    auto safe_div = [](double number, double denominator) -> double {
      return denominator == 0.0 ? 0.0 : number / denominator;
    };
    return safe_div(k1 * k2, k1 + k2);
  }
  Imp()
      : material_manager_(std::make_unique<core::MaterialManager>()),
        default_cr_(0.2),
        default_k_(2.8e8),
        default_cof_(0.3),
        default_tv_(0.1) {}
};
AverageForceContactSolver::AverageForceContactSolver()
    : imp_(std::make_unique<Imp>()) {}
AverageForceContactSolver::~AverageForceContactSolver() {};
SIRE_DEFINE_MOVE_CTOR_CPP(AverageForceContactSolver);
auto AverageForceContactSolver::resetMaterialManager(
    core::MaterialManager* manager) -> void {
  imp_->material_manager_.reset(manager);
}
auto AverageForceContactSolver::materialManager() -> core::MaterialManager& {
  return *imp_->material_manager_;
}
auto AverageForceContactSolver::setDefaultStiffness(double k) noexcept -> void {
  imp_->default_k_ = k;
}
auto AverageForceContactSolver::defaultStiffness() noexcept -> double {
  return imp_->default_k_;
}
auto AverageForceContactSolver::setDefaultCr(double cr) noexcept -> void {
  imp_->default_cr_ = cr;
}
auto AverageForceContactSolver::defaultCr() noexcept -> double {
  return imp_->default_cr_;
}
auto AverageForceContactSolver::setDefaultVelocityThreshold(double tv) noexcept
    -> void {
  imp_->default_tv_ = tv;
}
auto AverageForceContactSolver::defaultVelocityThreshold() noexcept -> double {
  return imp_->default_tv_;
}
// clang-format off
// 
// 质量矩阵的格式，假设有 n 个碰撞点
// I[num1][A|B][num2][A|B][x|y|z 1][x|y|z 2]
// I: inertia; 
// num1: idx of contact point which give force
// A|B: contact force direction
// num2: idx of influenced contact point
// A|B: influenced contact point's of PrtA or PrtB
// x|y|z 1: contact force 3 direction 
// x|y|z 2: influenced contact point 3 direction
//
// eg: 第一个碰撞点的对PrtA切向x方向的接触力对各个碰撞点的接触杆件A/B的比值质量
// [I1A1Axx I1A1Axy I1A1Axz I1A1Bxx I1A1Bxy I1A1Bxz ... I1AnAxx I1AnAxy I1AnAxz I1AnBxx I1AnBxy I1AnBxz]
// [I1A1Ayx I1A1Ayy I1A1Ayz I1A1Byx I1A1Byy I1A1Byz ... I1AnAyx I1AnAyy I1AnAyz I1AnByx I1AnByy I1AnByz]
// [I1A1Azx I1A1Azy I1A1Azz I1A1Bzx I1A1Bzy I1A1Bzz ... I1AnAzx I1AnAzy I1AnAzz I1AnBzx I1AnBzy I1AnBzz]
// [I1B1Axx I1B1Axy I1B1Axz I1B1Bxx I1B1Bxy I1B1Bxz ... I1BnAxx I1BnAxy I1BnAxz I1BnBxx I1BnBxy I1BnBxz]
// [I1B1Axy I1B1Ayy I1B1Ayz I1B1Byx I1B1Byy I1B1Byz ... I1BnAyx I1BnAyy I1BnAyz I1BnByx I1BnByy I1BnByz]
// [I1B1Azx I1B1Azy I1B1Azz I1B1Bzx I1B1Bzy I1B1Bzz ... I1BnAzx I1BnAzy I1BnAzz I1BnBzx I1BnBzy I1BnBzz]
// [   .       .       .       .       .       .    ...    .       .       .       .       .       .   ]
// [   .       .       .       .       .       .    ...    .       .       .       .       .       .   ]
// [InA1Axx InA1Axy InA1Axz InA1Bxx InA1Bxy InA1Bxz ... InAnAxx InAnAxy InAnAxz InAnBxx InAnBxy InAnBxz]
// [InA1Ayx InA1Ayy InA1Ayz InA1Bxx InA1Bxy InA1Bxz ... InAnAxx InAnAxy InAnAxz InAnByx InAnByy InAnByz]
// [InA1Azx InA1Azy InA1Azz InA1Bxx InA1Bxy InA1Bxz ... InAnAxx InAnAxy InAnAxz InAnBzx InAnBzy InAnBzz]
// [InB1Axx InB1Axy InB1Axz InB1Bxx InB1Bxy InB1Bxz ... InBnAxx InBnAxy InBnAxz InBnBxx InBnBxy InBnBxz]
// [InB1Ayx InB1Ayy InB1Ayz InB1Byx InB1Byy InB1Byz ... InBnAyx InBnAyy InBnAyz InBnByx InBnByy InBnByz]
// [InB1Azx InB1Azy InB1Azz InB1Bzx InB1Bzy InB1Bzz ... InBnAzx InBnAzy InBnAzz InBnBzx InBnBzy InBnBzz] 6n x 6n
//
// 矩阵的零元和无限元代表的意思：
// 零元：表示等式右边受到的力不会导致左边的某一特定接触点产生特定方向的加速度
// 无限元：与零元的意义一致，也是无论右边受到怎么样的力左边都不会产生相应的加速度
// 
// 通过碰撞检测得到的碰撞点信息，碰撞点的位姿矩阵计算得到惯量矩阵
// 通过 aris 拷贝过来的 init_interaction 来初始化重新加入的 fce
// Step1：记录model的 fce 的active状态情况，并全部设置为 deactive
// Step2: 遍历碰撞点，记录碰撞点的杆件的 vs 与 as
// Step3: 循环给力，计算各个接触点杆件的反应，无反应记录为0而不是正无穷 f / m = a
//
// 质量矩阵在接触点处的微分方程
// I a = F
// 其中 I 的格式由上边的矩阵给出，由此，a 与 F 的格式如下（由对角线元素决定，并由非对角线元素验证）
// a = [a1Ax a1Ay a1Az a1Bx a1By a1Bz ... anAx anAy anAz anBx anBy anBz]'; 6n x 1
// F = [F1Ax F1Ay F1Az F1Bx F1By F1Bz ... FnAx FnAy FnAz FnBx FnBy FnBz]'; 6n x 1
// 摩擦力对两个物体来说在同一坐标系下大小相同方向相反，分析力分解后对 x y 方向的切向加速度的影响，所以也是相减
// 所以需要先对矩阵 I 求逆矩阵得到 a = I^-1 F
// 并在对应项相减，得到基于接触模型的多点接触公式矩阵
// a = [a1Ax-a1Bx a1Ay-a1By a1Az-a1Bz ... anAx-anBy anAx-anBy anAz-anBz]'; 3n x 1
// F = I^-1 * [F1Ax F1Ay F1Az F1Bx F1By F1Bz ... FnAx FnAy FnAz FnBx FnBy FnBz]' 然后对应项目相减; 3n x 1
// 
// 方法一：使用当前时刻状态计算切向摩擦力大小，将切向摩擦力直接带入动力学，最后会成为法向平衡矩阵的外力项目，
//        不需要将切向问题引入平衡矩阵，直接代入动力学应该会产生一致的效果。
// 方法二：将切向问题与法向问题引入公式与大矩阵同时求解。
// clang-format on
auto AverageForceContactSolver::cptContactPointInertiaMatrix(
    const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    const std::vector<std::array<double, 16>>& T_C_vec,
    std::vector<double>& cpi) -> void {
  auto init_interaction = [](aris::dynamic::Interaction& interaction,
                             aris::dynamic::Model* m) -> void {
    if (interaction.prtNameM().empty() && interaction.prtNameN().empty() &&
        interaction.makNameI().empty() && interaction.makNameJ().empty())
      return;

    auto find_part = [m](std::string_view name) -> aris::dynamic::Part* {
      auto found = std::find_if(
          m->partPool().begin(), m->partPool().end(),
          [name](const auto& part) -> bool { return part.name() == name; });
      return found == m->partPool().end() ? nullptr : &*found;
    };

    auto find_marker = [](aris::dynamic::Part* part,
                          std::string_view name) -> aris::dynamic::Marker* {
      auto found = std::find_if(
          part->markerPool().begin(), part->markerPool().end(),
          [name](const auto& marker) -> bool { return marker.name() == name; });
      return found == part->markerPool().end() ? nullptr : &*found;
    };

    auto prt_m = find_part(interaction.prtNameM());
    auto mak_i = find_marker(prt_m, interaction.makNameI());
    auto prt_n = find_part(interaction.prtNameN());
    auto mak_j = find_marker(prt_n, interaction.makNameJ());

    interaction.setMakI(&*mak_i);
    interaction.setMakJ(&*mak_j);
  };
  // 初始化一些重复使用的变量
  auto enginePtr = physicsEnginePtr();
  SIRE_ASSERT(enginePtr != nullptr);
  auto modelPtr = enginePtr->currentModel();
  SIRE_ASSERT(modelPtr != nullptr);
  Size numContactPoint = penetration_pairs.size();
  auto& forcePool = modelPtr->forcePool();
  auto& partPool = modelPtr->partPool();
  Size testForceIdxOffset = forcePool.size();
  // Size testForceIdxOffset = 0;
  const double fceValue = 10.0;
  // 初始化结果容器
  const Size cpiWidth = 6 * numContactPoint;
  cpi.resize(cpiWidth * cpiWidth);
  // 在修改 forcePool 之前记录forcePool的 active 状态
  FceActiveStateRecorder recorder(modelPtr);
  // 将在 forcePool 中的力全部 deactivate
  for (auto& fce : forcePool) fce.activate(false);
  // add generalForce to forcePool() in Model and init
  typedef std::map<sire::geometry::GeometryId, std::array<double, 6>>
      PrtAsVsType;
  PrtAsVsType contactPrtVsMap;

  for (Size i = 0; i < numContactPoint; ++i) {
    const geometry::CollidableGeometry* geometry_A_ptr =
        enginePtr->queryGeometryPoolById(penetration_pairs[i].id_A);
    const geometry::CollidableGeometry* geometry_B_ptr =
        enginePtr->queryGeometryPoolById(penetration_pairs[i].id_B);
    SIRE_DEMAND(geometry_A_ptr != nullptr);
    SIRE_DEMAND(geometry_B_ptr != nullptr);
    sire::PartId contactPrt[2]{geometry_A_ptr->partId(),
                               geometry_B_ptr->partId()};
    auto& fcea = forcePool.add<aris::dynamic::GeneralForce>(
        std::string("test_fa" + std::to_string(i + testForceIdxOffset)),
        &partPool.at(contactPrt[0]).markerPool().at(0),
        &partPool.at(modelPtr->ground().id()).markerPool().at(0));
    auto& fceb = forcePool.add<aris::dynamic::GeneralForce>(
        std::string("test_fb" + std::to_string(i + testForceIdxOffset)),
        &partPool.at(contactPrt[1]).markerPool().at(0),
        &partPool.at(modelPtr->ground().id()).markerPool().at(0));

    fcea.resetModel(modelPtr);
    fcea.setFce(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());
    // force id 可以先不管
    init_interaction(fcea, modelPtr);
    fceb.resetModel(modelPtr);
    fceb.setFce(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());
    // force id 可以先不管
    init_interaction(fceb, modelPtr);
    for (Size j = 0; j < 2; ++j) {
      auto& prt = partPool[contactPrt[j]];
      PrtAsVsType::iterator lbofPrt =
          contactPrtVsMap.lower_bound(contactPrt[j]);
      // 如果 prt_id 在 contactPrtVsMap 中找不到，计算As与Vs并存入map
      if (lbofPrt == contactPrtVsMap.end() ||
          contactPrtVsMap.key_comp()(contactPrt[j], lbofPrt->first)) {
        std::array<double, 6> vs{0};
        prt.getVs(vs.data());
        contactPrtVsMap.insert(
            PrtAsVsType::value_type(contactPrt[j], std::move(vs)));
      }
    }
  }
  // std::cout << aris::core::toXmlString(*modelPtr) << std::endl;

  // 给力并计算质量矩阵
  double testFce[3][3] = {{fceValue, 0, 0}, {0, fceValue, 0}, {0, 0, fceValue}};
  const double* gravityAs = modelPtr->environment().gravity();
  for (Size iContact = 0; iContact < numContactPoint; ++iContact) {
    // 遍历行时不需要碰撞点到底是哪个 id_A or id_B
    for (Size i2 = 0; i2 < 2; ++i2) {
      for (Size idir = 0; idir < 3; ++idir) {
        double fs[6];
        sire::core::screw::s_fpm2fs(testFce[idir], T_C_vec[iContact].data(),
                                    fs);
        dynamic_cast<aris::dynamic::GeneralForce&>(
            forcePool.at(2 * iContact + i2 + testForceIdxOffset))
            .setFce(fs);
        if (modelPtr->forwardDynamics()) {
          std::cout << "forward dynamic failed" << std::endl;
        }
        // 记录所有碰撞点的As，方便取用
        PrtAsVsType contactPrtAsMap;
        for (Size jContact = 0; jContact < numContactPoint; ++jContact) {
          const geometry::CollidableGeometry* geometry_A_ptr =
              enginePtr->queryGeometryPoolById(
                  penetration_pairs[jContact].id_A);
          const geometry::CollidableGeometry* geometry_B_ptr =
              enginePtr->queryGeometryPoolById(
                  penetration_pairs[jContact].id_B);
          SIRE_DEMAND(geometry_A_ptr != nullptr);
          SIRE_DEMAND(geometry_B_ptr != nullptr);
          sire::PartId contactPrt[2]{geometry_A_ptr->partId(),
                                     geometry_B_ptr->partId()};
          for (Size j2 = 0; j2 < 2; ++j2) {
            auto& prt = partPool[contactPrt[j2]];
            PrtAsVsType::iterator lbofPrt =
                contactPrtAsMap.lower_bound(contactPrt[j2]);
            // 如果 prt_id 在 contactPrtAsMap 中找不到，计算As并存入map
            if (lbofPrt == contactPrtAsMap.end() ||
                contactPrtAsMap.key_comp()(contactPrt[j2], lbofPrt->first)) {
              if (prt.id() == modelPtr->ground().id()) continue;
              std::array<double, 6> as{0};
              prt.getAs(as.data());
              // ground 的 as 就是
              // 0，不会有相应加速度，我们仿真碰撞里面要把这部分去掉
              // 虽然也放在我们的cpi计算步骤中，但是只要没有as，对应项目就一直是零，
              // 当作无限大质量物体处理
              if (prt.id() != modelPtr->ground().id())
                aris::dynamic::s_vs(6, gravityAs, as.data());
              contactPrtAsMap.insert(
                  PrtAsVsType::value_type(contactPrt[j2], std::move(as)));
            }
          }
        }
        for (Size jContact = 0; jContact < numContactPoint; ++jContact) {
          double ap_o[3]{0}, res[3]{0};
          const geometry::CollidableGeometry* geometry_A_ptr =
              enginePtr->queryGeometryPoolById(
                  penetration_pairs[jContact].id_A);
          const geometry::CollidableGeometry* geometry_B_ptr =
              enginePtr->queryGeometryPoolById(
                  penetration_pairs[jContact].id_B);
          SIRE_DEMAND(geometry_A_ptr != nullptr);
          SIRE_DEMAND(geometry_B_ptr != nullptr);
          auto& part1 = partPool[geometry_A_ptr->partId()];
          auto& part2 = partPool[geometry_B_ptr->partId()];
          const double* contactPosition =
              penetration_pairs[jContact].p_WC.data();
          auto& vs1 = contactPrtVsMap.at(geometry_A_ptr->partId());
          auto& as1 = contactPrtAsMap.at(geometry_A_ptr->partId());
          aris::dynamic::s_as2ap(vs1.data(), as1.data(), contactPosition, ap_o);
          aris::dynamic::s_inv_pm_dot_v3(T_C_vec[jContact].data(), ap_o, res);
          core::screw::s_dvi(fceValue, res, 3, 1e-5);
          std::copy_n(
              res, 3,
              &cpi[(6 * iContact + 3 * i2 + idir) * cpiWidth + jContact * 6]);
          auto& vs2 = contactPrtVsMap.at(geometry_B_ptr->partId());
          auto& as2 = contactPrtAsMap.at(geometry_B_ptr->partId());
          aris::dynamic::s_as2ap(vs2.data(), as2.data(), contactPosition, ap_o);
          aris::dynamic::s_inv_pm_dot_v3(T_C_vec[jContact].data(), ap_o, res);
          core::screw::s_dvi(fceValue, res, 3, 1e-5);
          std::copy_n(res, 3,
                      &cpi[(6 * iContact + 3 * i2 + idir) * cpiWidth +
                           jContact * 6 + 3]);
        }
      }
      double fs[6]{0};
      dynamic_cast<aris::dynamic::GeneralForce&>(
          forcePool.at(2 * iContact + i2 + testForceIdxOffset))
          .setFce(fs);
    }
  }
  for (Size i = 0; i < numContactPoint * 2; ++i) {
    forcePool.pop_back();
  }
}

// clang-format off
/// @brief 计算接触法向惯量矩阵，需要去掉ground相关的行和列
/// 代码的第一个版本直接给外力的情况下根据加速度计算接触点惯量，
/// 问题是如果机器人本身的力较大或者运动情况带来的加速度太大，
/// 会导致计算出来的惯量为负数，从物理意义上来说，在不关闭系统内部力与系统
/// 状态的情况下，应该使用斜率作为质量，因为f = ma方程在这种情况下
/// 不过原点，所以需要两个力，算到加速度的差值，作为f的计算依据
/// f1 = 10N -> f2 = 50N
/// 
/// 接触法向质量矩阵的格式，假设有 n 个碰撞点
/// I[num1][A|B][num2][A|B]
/// I: inertia; 
/// num1: idx of contact point which give force
/// A|B: contact force direction
/// num2: idx of influenced contact point
/// A|B: influenced contact point's of PrtA or PrtB
/// 只包含碰撞点间法向力的惯量矩阵
///
/// eg: 第一个碰撞点的对PrtA切向x方向的接触力对各个碰撞点的接触杆件A/B的比值质量
/// [I1A1Azz I1A1Bzz ... I1AnAzz I1AnBzz]
/// [I1B1Azz I1B1Bzz ... I1BnAzz I1BnBzz]
/// [   .       .    ...    .       .   ]
/// [   .       .    ...    .       .   ]
/// [InA1Azz InA1Bxz ... InAnAxz InAnBzz]
/// [InB1Azz InB1Bzz ... InBnAzz InBnBzz] 2n x 2n
///
/// 矩阵的零元和无限元代表的意思：
/// 零元：表示等式右边受到的力不会导致左边的某一特定接触点产生特定方向的加速度
/// 无限元：与零元的意义一致，也是无论右边受到怎么样的力左边都不会产生相应的加速度
/// 
/// 通过碰撞检测得到的碰撞点信息，碰撞点的位姿矩阵计算得到惯量矩阵
/// 通过 aris 拷贝过来的 init_interaction 来初始化重新加入的 fce
/// Step1：记录model的 fce 的active状态情况，并全部设置为 deactive
/// Step2: 遍历碰撞点，记录碰撞点的杆件的 vs 与 as
/// Step3: 循环给力，计算各个接触点杆件的反应，无反应记录为0而不是正无穷 f / m = a
///
/// 质量矩阵在接触点处的微分方程
/// I a = F
/// 其中 I 的格式由上边的矩阵给出，由此，a 与 F 的格式如下（由对角线元素决定，并由非对角线元素验证）
/// a = [a1Az a1Bz ... anAz anBz]'; 2n x 1
/// F = [F1Az F1Bz ... FnAz FnBz]'; 2n x 1
/// 摩擦力对两个物体来说在同一坐标系下大小相同方向相反，分析力分解后对 x y 方向的切向加速度的影响，所以也是相减
/// 所以需要先对矩阵 I 求逆矩阵得到 a = I^-1 F
/// 并在对应项相减，得到基于接触模型的多点接触公式矩阵
/// a = [a1Az-a1Bz ... anAz-anBz]'; n x 1
/// F = I^-1 * [F1Az F1Bz ... FnAz FnBz]' 然后对应项目相减; n x 1
/// 
/// 方法一：使用当前时刻状态计算切向摩擦力大小，将切向摩擦力直接带入动力学，最后会成为法向平衡矩阵的外力项目，
///        不需要将切向问题引入平衡矩阵，直接代入动力学应该会产生一致的效果。
/// 注意事项：
///  1. 得到的加速度需要减去 gravityAs 才能是我们的因为力产生的加速度才能添加计算质量
/// 在创建的时候记录 fce 的激活状态
/// 并在销毁的时候将记录的状态设置回去，放置在方法调用时放置过程中修改导致的状态不一致
///
/// 关于计算的 A b 的解释
/// x(t) = e^(At)(x(0) - A\b) + A\b
/// 因为要每个接触点的法向的位移相减，同时，一组接触变量控制了两个物体的状态
/// 需要使用下面的矩阵对 I 进行缩小，并与接触点的刚度与阻尼相乘，得到矩阵 K D
/// T = [1 -1 0 ... 0 0  0]
///     [0  0 1 -1 ...0  0]
///     [.  . .  . ....  .]
///     [0 0 0 0 .... 1 -1] n * 2n
/// K = T * I^-1 * T' * diag(k)  k 与 d 都是 1 * n 的向量
/// D = T * I^-1 * T' * diag(d)
/// 矩阵微分方程的状态转移矩阵 A 可以由三块组成，如下图所示
/// A = [0 I]
///     [K D] 2n * 2n 的矩阵
/// 接触点状态如下
/// x = [d1 ... dn d1' ... dn'] 2 * n  d表示接触距离（穿深）
/// x' = [d1' ... dn' d1'' ... dn''] 2 * n
/// x(0) = [由积分截断时记录的 v 与 p 决定]
/// 其中，关于非齐次方程的常数项 b，需要经过如下计算
/// F = [FeA1 FeB1 ... FeAn FeBn] 1 * 2n vector -> parameter fext
/// f = T * I^-1 * F' -> 1 * n vector
/// b = [0 f] -> 1 * 2n vector
/// 其中的 A \ b 方法 使用基于Householder方法的QR分解计算
/// 
/// @param[in] penetration_pairs (n x 1) 检测到的接触点信息
/// @param[in] T_C_vec (16 x n) 接触点相对于世界坐标系的坐标，接触点坐标系的z方向从物体A指向物体B，平行接触方向
/// @param[out] cpi (2n-g x 2n-g) 接触点惯量矩阵计算结果（contact point inertia matrix）g表示ground相关的数目
// clang-format on
auto AverageForceContactSolver::cptContactPointNormalInertiaMatrix(
    const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    const std::vector<std::array<double, 16>>& T_C_vec, const double* stiffness,
    const double* damping, std::vector<double>& cpi,
    std::vector<double>& extInvCpi, std::vector<double>& A) -> sire::Size {
  auto init_interaction = [](aris::dynamic::Interaction& interaction,
                             aris::dynamic::Model* m) -> void {
    if (interaction.prtNameM().empty() && interaction.prtNameN().empty() &&
        interaction.makNameI().empty() && interaction.makNameJ().empty())
      return;

    auto find_part = [m](std::string_view name) -> aris::dynamic::Part* {
      auto found = std::find_if(
          m->partPool().begin(), m->partPool().end(),
          [name](const auto& part) -> bool { return part.name() == name; });
      return found == m->partPool().end() ? nullptr : &*found;
    };

    auto find_marker = [](aris::dynamic::Part* part,
                          std::string_view name) -> aris::dynamic::Marker* {
      auto found = std::find_if(
          part->markerPool().begin(), part->markerPool().end(),
          [name](const auto& marker) -> bool { return marker.name() == name; });
      return found == part->markerPool().end() ? nullptr : &*found;
    };

    auto prt_m = find_part(interaction.prtNameM());
    auto mak_i = find_marker(prt_m, interaction.makNameI());
    auto prt_n = find_part(interaction.prtNameN());
    auto mak_j = find_marker(prt_n, interaction.makNameJ());

    interaction.setMakI(&*mak_i);
    interaction.setMakJ(&*mak_j);
  };
  // 初始化一些重复使用的变量
  auto enginePtr = physicsEnginePtr();
  SIRE_ASSERT(enginePtr != nullptr);
  auto modelPtr = enginePtr->currentModel();
  SIRE_ASSERT(modelPtr != nullptr);
  Size n = penetration_pairs.size();
  auto& forcePool = modelPtr->forcePool();
  auto& partPool = modelPtr->partPool();
  Size testForceIdxOffset = forcePool.size();
  // 在修改 forcePool 之前记录forcePool的 active 状态
  FceActiveStateRecorder recorder(modelPtr);
  // 将在 forcePool 中的力全部 deactivate
  for (auto& fce : forcePool) fce.activate(false);

  sire::Size cpiWidth = 2 * n;
  std::vector<sire::Size> groundFlag(cpiWidth, 0);
  for (Size i = 0; i < n; ++i) {
    const geometry::CollidableGeometry* geometry_A_ptr =
        enginePtr->queryGeometryPoolById(penetration_pairs[i].id_A);
    const geometry::CollidableGeometry* geometry_B_ptr =
        enginePtr->queryGeometryPoolById(penetration_pairs[i].id_B);
    SIRE_DEMAND(geometry_A_ptr != nullptr);
    SIRE_DEMAND(geometry_B_ptr != nullptr);
    sire::PartId contactPrt[2]{geometry_A_ptr->partId(),
                               geometry_B_ptr->partId()};
    for (Size j{0}; j < 2; ++j) {
      // add generalForce to forcePool() in Model and init
      auto& fce = forcePool.add<aris::dynamic::GeneralForce>(
          std::string("test_f" + j
                          ? "b"
                          : "a" + std::to_string(i + testForceIdxOffset)),
          &partPool.at(contactPrt[j]).markerPool().at(0),
          &partPool.at(modelPtr->ground().id()).markerPool().at(0));
      fce.resetModel(modelPtr);
      fce.setFce(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());
      // force id 可以先不管
      init_interaction(fce, modelPtr);
      if (contactPrt[j] == modelPtr->ground().id()) {
        // 标记 ground 相关的idx，计算cpi的真实大小
        groundFlag[2 * i + j] = 1;
        --cpiWidth;
      }
    }
  }
  // 初始化结果容器
  cpi.resize(cpiWidth * cpiWidth);
  // 给力并计算质量矩阵
  const double fceValue1 = 10.0;
  const double fceValue2 = 50.0;
  double testFce1[3] = {0, 0, fceValue1};
  double testFce2[3] = {0, 0, fceValue2};
  double* fces[2] = {testFce1, testFce2};
  const double* gravityAs = modelPtr->environment().gravity();
  sire::Size cpiLineIdx{0}, cpiColIdx{0};
  for (Size iContact = 0; iContact < n; ++iContact) {
    for (Size i2 = 0; i2 < 2; ++i2) {
      if (groundFlag[2 * iContact + i2]) continue;
      std::vector<double> az1(cpiWidth);
      for (Size fceIdx = 0; fceIdx < 2; ++fceIdx) {
        double fs[6];
        sire::core::screw::s_fpm2fs(fces[fceIdx], T_C_vec[iContact].data(), fs);
        dynamic_cast<aris::dynamic::GeneralForce&>(
            forcePool.at(2 * iContact + i2 + testForceIdxOffset))
            .setFce(fs);
        if (modelPtr->forwardDynamics())
          std::cout << "forward dynamic failed" << std::endl;
        cpiColIdx = 0;
        for (Size jContact = 0; jContact < n; ++jContact) {
          double ap_o[3]{0}, res[3]{0};
          const geometry::CollidableGeometry* geometry_A_ptr =
              enginePtr->queryGeometryPoolById(
                  penetration_pairs[jContact].id_A);
          const geometry::CollidableGeometry* geometry_B_ptr =
              enginePtr->queryGeometryPoolById(
                  penetration_pairs[jContact].id_B);
          SIRE_DEMAND(geometry_A_ptr != nullptr);
          SIRE_DEMAND(geometry_B_ptr != nullptr);
          sire::PartId contactPrt[2]{geometry_A_ptr->partId(),
                                     geometry_B_ptr->partId()};
          const double* contactPosition =
              penetration_pairs[jContact].p_WC.data();
          std::array<double, 6> as;
          for (Size j2 = 0; j2 < 2; ++j2) {
            if (groundFlag[2 * jContact + j2]) continue;
            auto& prt = partPool[contactPrt[j2]];
            prt.getAs(as.data());
            aris::dynamic::s_vs(6, gravityAs, as.data());
            aris::dynamic::s_as2ap(prt.vs(), as.data(), contactPosition, ap_o);
            aris::dynamic::s_inv_pm_dot_v3(T_C_vec[jContact].data(), ap_o, res);
            if (fceIdx) {
              cpi[cpiLineIdx * cpiWidth + cpiColIdx] = core::screw::s_safe_div(
                  fceValue2 - fceValue1, res[2] - az1[cpiColIdx], 1e-4);
            } else {
              az1[cpiColIdx] = res[2];
            }
            // core::screw::s_dvi(fceValue, res, 3, 1e-5);
            // cpi[cpiLineIdx * cpiWidth + cpiColIdx] = res[2];
            ++cpiColIdx;
          }
        }
      }
      double fs1[6]{0};
      dynamic_cast<aris::dynamic::GeneralForce&>(
          forcePool.at(2 * iContact + i2 + testForceIdxOffset))
          .setFce(fs1);
      ++cpiLineIdx;
    }
  }
  for (Size i{0}; i < n * 2; ++i) {
    forcePool.pop_back();
  }
  // ConstrctKDMatrixRhs(){
  std::vector<double> kdMatrix(cpiWidth * 2 * n);
  for (Size i{0}, lineIdx{0}; i < 2 * n; ++i) {
    if (groundFlag[i]) continue;
    Size lineBegin = lineIdx * 2 * n;
    int sgn = (i % 2) ? 1 : -1;
    kdMatrix[lineBegin + i / 2] = sgn * stiffness[i / 2];
    kdMatrix[lineBegin + n + i / 2] = sgn * damping[i / 2];
    ++lineIdx;
  }
  // }
  // ----------------------------------------------------
  // ------------ 接触点惯量矩阵求逆 ---------------------
  // ----------------------------------------------------
  std::vector<double> invCpi(cpiWidth * cpiWidth);
  std::vector<double> u(cpiWidth * cpiWidth), tau(cpiWidth), tau2(cpiWidth);
  std::vector<aris::Size> p(cpiWidth);
  aris::Size rank;
  aris::dynamic::s_householder_utp(cpiWidth, cpiWidth, cpi.data(), u.data(),
                                   tau.data(), p.data(), rank);
  if (rank != cpiWidth)
    THROW_FILE_LINE("Invalid singular matrix for normal contact inertia");
  aris::dynamic::s_householder_utp2pinv(cpiWidth, cpiWidth, rank, u.data(),
                                        tau.data(), p.data(), invCpi.data(),
                                        tau2.data());

  std::vector<double> temp1(cpiWidth * 2 * n, 0);
  aris::dynamic::s_mm(cpiWidth, 2 * n, cpiWidth, invCpi.data(), kdMatrix.data(),
                      temp1.data());
  A.resize(4 * n * n, 0);
  // TODO: 不存在两个ground相撞
  for (Size i{0}, lineIdx{0}; i < n; ++i) {
    A[2 * n * i + n + i] = 1;
    if (!groundFlag[2 * i] && !groundFlag[2 * i]) {
      for (Size j{0}; j < 2 * n; ++j) {
        A[2 * n * (i + n) + j] =
            temp1[lineIdx * 2 * n + j] - temp1[(lineIdx + 1) * 2 * n + j];
      }
      lineIdx += 2;
    } else {
      int sgn = groundFlag[2 * i] ? -1 : 1;
      for (Size j{0}; j < 2 * n; ++j) {
        A[2 * n * (i + n) + j] = sgn * temp1[lineIdx * 2 * n + j];
      }
      ++lineIdx;
    }
  }

  extInvCpi.resize(4 * n * n, 0);
  cpiLineIdx = cpiColIdx = 0;
  for (sire::Size i{0}; i < 2 * n; ++i) {
    if (groundFlag[i]) continue;
    cpiColIdx = 0;
    for (sire::Size j{0}; j < 2 * n; ++j) {
      if (groundFlag[j]) continue;
      extInvCpi[i * 2 * n + j] = invCpi[cpiLineIdx * cpiWidth + cpiColIdx];
      ++cpiColIdx;
    }
    ++cpiLineIdx;
  }
  // A.resize(4 * n * n, 0);
  // for (int i{0}; i < n; ++i) {
  //   for (int j{0}; j < n; ++j) {
  //     double evenColMinus = extInvCpi[2 * n * 2 * i + 2 * j] -
  //                           extInvCpi[2 * n * (2 * i + 1) + 2 * j];
  //     double oddColMinus = extInvCpi[2 * n * 2 * i + 2 * j + 1] -
  //                          extInvCpi[2 * n * (2 * i + 1) + 2 * j + 1];
  //     A[2 * n * i + n + j] = (i == j);
  //     // 因为接触力方向与穿深加速度方向一定反向，所以都需要加上负号
  //     A[2 * n * (i + n) + j] = -stiffness[i] * (evenColMinus - oddColMinus);
  //     A[2 * n * (i + n) + n + j] = -damping[i] * (evenColMinus -
  //     oddColMinus);
  //   }
  // }

  return cpiWidth;
}

// clang-format off
/// @brief 计算除了接触力外的所有力当作接触模型求解的外力项，假设检测到 n 个接触点。
/// 关于计算的 A b 的解释
/// x(t) = e^(At)(x(0) - A\b) + A\b
/// 因为要每个接触点的法向的位移相减，同时，一组接触变量控制了两个物体的状态
/// 需要使用下面的矩阵对 I 进行缩小，并与接触点的刚度与阻尼相乘，得到矩阵 K D
/// T = [1 -1 0 ... 0 0  0]
///     [0  0 1 -1 ...0  0]
///     [.  . .  . ....  .]
///     [0 0 0 0 .... 1 -1] n * 2n
/// K = T * I^-1 * T' * diag(k)  k 与 d 都是 1 * n 的向量
/// D = T * I^-1 * T' * diag(d)
/// 矩阵微分方程的状态转移矩阵 A 可以由三块组成，如下图所示
/// A = [0 I]
///     [K D] 2n * 2n 的矩阵
/// 接触点状态如下
/// x = [d1 ... dn d1' ... dn'] 2 * n  d表示接触距离（穿深）
/// x' = [d1' ... dn' d1'' ... dn''] 2 * n
/// x(0) = [由积分截断时记录的 v 与 p 决定]
/// 其中，关于非齐次方程的常数项 b，需要经过如下计算
/// F = [FeA1 FeB1 ... FeAn FeBn] 1 * 2n vector -> parameter fext
/// f = T * I^-1 * F' -> 1 * n vector
/// b = [0 f] -> 1 * 2n vector
/// 其中的 A \ b 方法 使用基于Householder方法的QR分解计算
///        
/// @param[in] penetration_pairs n x 1 检测到的接触点信息
/// @param[in] T_C_vec 16 x n 接触点相对于世界坐标系的坐标，接触点坐标系的z方向从物体B指向物体A，平行接触方向
/// @param[in] cpi 2n x 2n 接触点惯量矩阵（contact point inertia matrix）
/// @param[out] fext (2n - g) x 1 计算得到的法向外力结果，表示为 [FeA1, FeB1, ... , FeAn, FeBn]，去掉ground相关的外力项
// clang-format on
auto AverageForceContactSolver::cptContactPrtExtForce(
    const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    const std::vector<std::array<double, 16>>& T_C_vec, const double* cpi,
    sire::Size cpiWidth, const double* extInvCpi, double* fext, double* b)
    -> void {
  auto enginePtr = physicsEnginePtr();
  SIRE_ASSERT(enginePtr != nullptr);
  // TODO: 可能需要关掉contactForce
  enginePtr->activateContactForce(false);
  // TODO: 不知道是否需要，记录杆件的加速度数据，然后要重新填回去
  auto modelPtr = enginePtr->currentModel();
  SIRE_ASSERT(modelPtr != nullptr);
  Size numContactPoint = penetration_pairs.size();
  if (modelPtr->forwardDynamics())
    std::cout << "forward dynamic failed" << std::endl;
  auto& partPool = modelPtr->partPool();
  const sire::Size nContact = penetration_pairs.size();
  // 遍历碰撞点，找到所有要求的杆件与碰撞点位姿
  sire::Size cpiIdx{0};
  for (int i{0}; i < nContact; ++i) {
    auto& pair = penetration_pairs[i];
    const geometry::CollidableGeometry* geometry_A_ptr =
        enginePtr->queryGeometryPoolById(pair.id_A);
    const geometry::CollidableGeometry* geometry_B_ptr =
        enginePtr->queryGeometryPoolById(pair.id_B);
    SIRE_DEMAND(geometry_A_ptr != nullptr);
    SIRE_DEMAND(geometry_B_ptr != nullptr);
    sire::PartId contactPrt[2]{geometry_A_ptr->partId(),
                               geometry_B_ptr->partId()};
    const double* contactPosition = pair.p_WC.data();
    for (Size i2 = 0; i2 < 2; ++i2) {
      auto& prt = partPool[contactPrt[i2]];
      double ap_o[3]{0}, ap_c[3]{0};
      aris::dynamic::s_as2ap(prt.vs(), prt.as(), contactPosition, ap_o);
      aris::dynamic::s_inv_pm_dot_v3(T_C_vec[i].data(), ap_o, ap_c);
      // f = ma;
      if (prt.id() == modelPtr->ground().id()) {
        fext[i * 2 + i2] = 0;
      } else {
        fext[i * 2 + i2] = ap_c[2] * cpi[(cpiIdx)*cpiWidth + cpiIdx];
        ++cpiIdx;
      }
    }
  }
  // ---------------------------------------------------------------
  // T = [1 -1 0 ... 0 0  0]
  //     [0  0 1 -1 ...0  0]
  //     [.  . .  . ....  .]
  //     [0 0 0 0 .... 1 -1] n * 2n
  // shrinked inverse cpi matrix = T * I^-1 * T'
  // ---------------计算缩小后的 cpi 矩阵  n x n ---------------------
  // 直接计算 A 与 b，通过中间变量储存一些中间值
  // ----------------------------------------------------------------
  for (int i{0}; i < nContact; ++i) {
    b[i] = 0;
    b[nContact + i] = 0;
    for (int j{0}; j < nContact; ++j) {
      double evenColMinus = extInvCpi[2 * nContact * 2 * i + 2 * j] -
                            extInvCpi[2 * nContact * (2 * i + 1) + 2 * j];
      double oddColMinus = extInvCpi[2 * nContact * 2 * i + 2 * j + 1] -
                           extInvCpi[2 * nContact * (2 * i + 1) + 2 * j + 1];
      // 这中间确实应该是加号，之后需要测试一下
      b[nContact + i] +=
          evenColMinus * fext[2 * j] + oddColMinus * fext[2 * j + 1];
    }
  }
  enginePtr->activateContactForce(true);
}

auto AverageForceContactSolver::preprocessContactInfo(
    const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    sire::PartId* prtIdVector, LhsVariableType* variableType, int* groundFlag)
    -> sire::Size {
  auto enginePtr = physicsEnginePtr();
  SIRE_ASSERT(enginePtr != nullptr);
  auto modelPtr = enginePtr->currentModel();
  SIRE_ASSERT(modelPtr != nullptr);
  const sire::Size n = penetration_pairs.size();
  sire::Size cpiWidth = 2 * n;
  const sire::PartId groundId = modelPtr->ground().id();
  for (sire::Size i{0}; i < n; ++i) {
    const geometry::CollidableGeometry* geometry_A_ptr =
        enginePtr->queryGeometryPoolById(penetration_pairs[i].id_A);
    const geometry::CollidableGeometry* geometry_B_ptr =
        enginePtr->queryGeometryPoolById(penetration_pairs[i].id_B);
    SIRE_DEMAND(geometry_A_ptr != nullptr);
    SIRE_DEMAND(geometry_B_ptr != nullptr);
    // 标记 ground 相关的idx，计算cpi的真实大小
    // 默认是两个加速度a
    variableType[i] = LhsVariableType::TwoAccel;
    prtIdVector[2 * i] = geometry_A_ptr->partId();
    if (prtIdVector[2 * i] == groundId) {
      // 假定没有两个 Ground 相撞
      groundFlag[2 * i] = 1;
      variableType[i] = LhsVariableType::OneDelta;
      --cpiWidth;
    }
    prtIdVector[2 * i + 1] = geometry_B_ptr->partId();
    if (prtIdVector[2 * i + 1] == groundId) {
      // 假定没有两个 Ground 相撞
      groundFlag[2 * i + 1] = 1;
      variableType[i] = LhsVariableType::OneDelta;
      --cpiWidth;
    }
  }
  return cpiWidth;
}
auto AverageForceContactSolver::cptCpiMatrix(
    const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    const std::vector<std::array<double, 16>>& T_C_vec,
    const sire::PartId* prtIdVector, sire::Size cpiWidth, const int* groundFlag,
    double* cpi) -> void {
  auto init_interaction = [](aris::dynamic::Interaction& interaction,
                             aris::dynamic::Model* m) -> void {
    if (interaction.prtNameM().empty() && interaction.prtNameN().empty() &&
        interaction.makNameI().empty() && interaction.makNameJ().empty())
      return;

    auto find_part = [m](std::string_view name) -> aris::dynamic::Part* {
      auto found = std::find_if(
          m->partPool().begin(), m->partPool().end(),
          [name](const auto& part) -> bool { return part.name() == name; });
      return found == m->partPool().end() ? nullptr : &*found;
    };

    auto find_marker = [](aris::dynamic::Part* part,
                          std::string_view name) -> aris::dynamic::Marker* {
      auto found = std::find_if(
          part->markerPool().begin(), part->markerPool().end(),
          [name](const auto& marker) -> bool { return marker.name() == name; });
      return found == part->markerPool().end() ? nullptr : &*found;
    };

    auto prt_m = find_part(interaction.prtNameM());
    auto mak_i = find_marker(prt_m, interaction.makNameI());
    auto prt_n = find_part(interaction.prtNameN());
    auto mak_j = find_marker(prt_n, interaction.makNameJ());

    interaction.setMakI(&*mak_i);
    interaction.setMakJ(&*mak_j);
  };
  // 初始化一些重复使用的变量
  auto enginePtr = physicsEnginePtr();
  SIRE_ASSERT(enginePtr != nullptr);
  auto modelPtr = enginePtr->currentModel();
  SIRE_ASSERT(modelPtr != nullptr);
  auto& forcePool = modelPtr->forcePool();
  auto& partPool = modelPtr->partPool();
  sire::Size testForceIdxOffset = forcePool.size();
  // 在修改 forcePool 之前记录forcePool的 active 状态
  FceActiveStateRecorder recorder(modelPtr);
  // 将在 forcePool 中的力全部 deactivate
  for (auto& fce : forcePool) fce.activate(false);

  const sire::Size n = penetration_pairs.size();
  // 给力并计算质量矩阵
  const double fceValue1 = 10.0;
  const double fceValue2 = 50.0;
  double testFce1[3] = {0, 0, fceValue1};
  double testFce2[3] = {0, 0, fceValue2};
  double* fces[2] = {testFce1, testFce2};
  const double* gravityAs = modelPtr->environment().gravity();
  for (Size i{0}; i < n; ++i) {
    for (Size j{0}; j < 2; ++j) {
      // add generalForce to forcePool() in Model and init
      auto& fce = forcePool.add<aris::dynamic::GeneralForce>(
          std::string("test_f" + j
                          ? "b"
                          : "a" + std::to_string(i + testForceIdxOffset)),
          &partPool.at(prtIdVector[2 * i + j]).markerPool().at(0),
          &partPool.at(modelPtr->ground().id()).markerPool().at(0));
      fce.resetModel(modelPtr);
      fce.setFce(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());
      // force id 可以先不管
      init_interaction(fce, modelPtr);
    }
  }
  for (Size iContact{0}, cpiLineIdx{0}; iContact < n; ++iContact) {
    for (Size i2{0}; i2 < 2; ++i2) {
      if (groundFlag[2 * iContact + i2]) continue;
      auto& gf = dynamic_cast<aris::dynamic::GeneralForce&>(
          forcePool.at(2 * iContact + i2 + testForceIdxOffset));
      std::vector<double> az1(cpiWidth);
      for (sire::Size fceIdx{0}; fceIdx < 2; ++fceIdx) {
        double fs[6];
        sire::core::screw::s_fpm2fs(fces[fceIdx], T_C_vec[iContact].data(), fs);
        gf.setFce(fs);
        if (modelPtr->forwardDynamics())
          std::cout << "forward dynamic failed" << std::endl;

        for (Size jContact{0}, cpiColIdx{0}; jContact < n; ++jContact) {
          double ap_o[3]{0}, res[3]{0};
          const double* contactPosition =
              penetration_pairs[jContact].p_WC.data();
          std::array<double, 6> as;
          for (Size j2 = 0; j2 < 2; ++j2) {
            if (groundFlag[2 * jContact + j2]) continue;
            auto& prt = partPool[prtIdVector[2 * jContact + j2]];
            prt.getAs(as.data());
            aris::dynamic::s_vs(6, gravityAs, as.data());
            aris::dynamic::s_as2ap(prt.vs(), as.data(), contactPosition, ap_o);
            aris::dynamic::s_inv_pm_dot_v3(T_C_vec[jContact].data(), ap_o, res);
            if (fceIdx) {
              cpi[cpiLineIdx * cpiWidth + cpiColIdx] = core::screw::s_safe_div(
                  fceValue2 - fceValue1, res[2] - az1[cpiColIdx], 1e-4);
            } else {
              az1[cpiColIdx] = res[2];
            }
            ++cpiColIdx;
          }
        }
      }
      aris::dynamic::s_fill(1, 6, 0, const_cast<double*>(gf.fce()));
      ++cpiLineIdx;
    }
  }
  for (sire::Size i{0}; i < 2 * n; ++i) {
    forcePool.pop_back();
  }
}
auto AverageForceContactSolver::cptContactSolverResult(
    const aris::dynamic::Model* current_state,
    const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    const std::vector<std::array<double, 16>>& T_C_vec,
    ContactSolverResult& result) -> void {
  if (penetration_pairs.size() == 0) {
    imp_->prevResult.reset();
    return;
  }
  auto enginePtr = physicsEnginePtr();
  SIRE_ASSERT(enginePtr != nullptr);
  auto modelPtr = enginePtr->currentModel();
  SIRE_ASSERT(modelPtr != nullptr);
  auto& partPool = modelPtr->partPool();
  result.resize(partPool.size() * 6, penetration_pairs.size());
  std::vector<sire::Size> preservedPairsIdx;
  std::vector<double> invCpi;
  std::vector<double> accelExt;
  std::vector<sire::PartId> prtIdVector;
  for (sire::Size i{0}; i < penetration_pairs.size(); ++i) {
    auto& pair = penetration_pairs[i];
    const geometry::CollidableGeometry* geometry_A_ptr =
        enginePtr->queryGeometryPoolById(pair.id_A);
    const geometry::CollidableGeometry* geometry_B_ptr =
        enginePtr->queryGeometryPoolById(pair.id_B);
    SIRE_DEMAND(geometry_A_ptr != nullptr);
    SIRE_DEMAND(geometry_B_ptr != nullptr);
    // 标记 ground 相关的idx，计算cpi的真实大小
    // 默认是两个加速度a
    result.prtsA[i] = geometry_A_ptr->partId();
    result.prtsB[i] = geometry_B_ptr->partId();
    result.contactPairIdxMap_.insert(
        {sire::core::SortedPair<sire::PartId>(geometry_A_ptr->partId(),
                                              geometry_B_ptr->partId()),
         i});
  }
  filterPairsAndPreprocessInfo(*enginePtr, penetration_pairs, T_C_vec,
                               preservedPairsIdx, prtIdVector, accelExt,
                               invCpi);
  DLOG(DEBUG) << "invCpi = " << invCpi << " accelExt = " << accelExt;
  sire::Size n{preservedPairsIdx.size()};
  if (n == 0) return;
  sire::Size n2{2 * n};
  // std::vector<double> accelExt(n2);
  // cptAccelExtVector(*enginePtr, penetration_pairs, T_C_vec,
  // preservedPairsIdx,
  //                   prtIdVector.data(), accelExt.data());
  std::vector<double> stiffness(n), damping(n), x0(n2), v0(3 * n);
  double stiffScale = cptInitialCondition(
      *enginePtr, *(imp_->material_manager_), penetration_pairs, T_C_vec,
      preservedPairsIdx, stiffness.data(), damping.data(), x0.data(),
      v0.data());

  // cpi因为要去掉ground，所以可能不是n2的，但是最后相减之后应该是 n 的
  // 对于算出来的cpi，在算逆前先类似得到矩阵A的处理一下（相减）应该就可以，
  // 同时PrtExtForce也不用管。
  // remove ground related cpi and fext;
  // 注意 cpi 可能是奇数，因为要去掉相应的ground，但A一定是偶数矩阵
  std::vector<double> A(n2 * n2), b(n2);
  cptDAECoeff(*enginePtr, n, stiffness.data(), damping.data(), stiffScale,
              accelExt.data(), invCpi.data(), A.data(), b.data());

  double minTime =
      findMinRootBisection(n, A.data(), b.data(), x0.data(), 1e-10, 200);
  // DLOG(DEBUG) << " b: " << b << " A: " << A << " x0: " << x0 << " minTime: "
  // << minTime << std::endl;
  // 没有零点的情况下，取A中的最大值作为参考计算步长
  if (minTime <= 0) {
    double maxA = 0;
    for (sire::Size i{0}; i < A.size(); ++i) {
      double temp = std::abs(A[i]);
      if (temp > maxA) maxA = temp;
    }
    double timeAuto = std::pow(10, - 1 - int(floor(std::log10(maxA)) / 2));
    minTime = result.dt > timeAuto ? timeAuto : result.dt;
    result.dt = minTime;
  } else {
    if (minTime > result.dt) {
      minTime = result.dt;
    } else {
      result.dt = minTime;
    }
  }
  std::vector<double> avgFce(n);
  cptAvgContactFce(n, A.data(), b.data(), x0.data(), 0, minTime,
                   stiffness.data(), damping.data(), avgFce.data());
  // sire::Size avgFceIdx{0};
  // solver_result.resize(imp_->part_size_ * 6, penetration_pairs.size());
  DLOG(DEBUG) << " minTime: " << minTime << " b: " << b << " A: " << A
              << " x0: " << x0;
  for (sire::Size i{0}; i < n; ++i) {
    sire::Size idx = preservedPairsIdx[i];
    const common::PenetrationAsPointPair& pair = penetration_pairs[idx];
    result.fn[idx] = avgFce[i];
  }

  if (imp_->prevResult.isEmpty_) {
    for (sire::Size i{0}; i < n; ++i) {
      sire::Size idx = preservedPairsIdx[i];
      result.ft[2 * idx] = 0;
      result.ft[2 * idx + 1] = 0;
    }
  } else {
    for (sire::Size i{0}; i < n; ++i) {
      sire::Size idx = preservedPairsIdx[i];
      double* v_contact = v0.data() + 3 * i;
      double vt = aris::dynamic::s_norm(2, v_contact);
      double zero_check = 1e-5;
      if (vt < zero_check ||
          (imp_->prevResult.contactPairIdxMap_.find(
               sire::core::SortedPair(result.prtsA[idx], result.prtsB[idx])) ==
           imp_->prevResult.contactPairIdxMap_.end())) {
        result.ft[2 * idx] = 0;
        result.ft[2 * idx + 1] = 0;
      } else {
        auto safe_div = [](double number, double denominator, double zero_check,
                           double err_set) -> double {
          return std::abs(denominator) <= zero_check ? err_set
                                                     : number / denominator;
        };
        const common::PenetrationAsPointPair& pair = penetration_pairs[idx];
        auto* geometry_A = enginePtr->queryGeometryPoolById(pair.id_A);
        auto* geometry_B = enginePtr->queryGeometryPoolById(pair.id_B);
        const core::PropMap& pair_prop =
            imp_->material_manager_->getPropMapOrDefault(
                {geometry_A->material(), geometry_B->material()});
        double threshold_velocity = pair_prop.getPropValueOrDefault(
            "threshold_velocity", imp_->default_tv_);
        double friction_coefficient =
            pair_prop.getPropValueOrDefault("cof", imp_->default_cof_);
        double t1 = std::abs(safe_div(v_contact[0], v_contact[1], 1e-8, 1e10));
        double t2 = std::sqrt(t1 * t1 + 1);

        double ft{0};
        sire::Size prevIdx =
            imp_->prevResult.contactPairIdxMap_[sire::core::SortedPair(
                result.prtsA[idx], result.prtsB[idx])];
        if (vt > threshold_velocity) {
          ft = std::abs(0.95 * friction_coefficient *
                        imp_->prevResult.fn[prevIdx]);
        } else {
          ft = std::abs(friction_coefficient * imp_->prevResult.fn[prevIdx] *
                        (std::expm1(-3 * vt / threshold_velocity)));
        }
        // if (vt > threshold_velocity) {
        //   ft = std::abs(0.95 * friction_coefficient * result.fn[idx]);
        // } else {
        //   ft = std::abs(friction_coefficient * result.fn[idx] *
        //                 (std::expm1(-3 * vt / threshold_velocity)));
        // }
        result.ft[2 * idx] = -1 * aris::dynamic::s_sgn(v_contact[0]) * ft *
                             safe_div(t1, t2, zero_check, 0.0);
        result.ft[2 * idx + 1] = -1 * aris::dynamic::s_sgn(v_contact[1]) * ft *
                                 safe_div(1, t2, zero_check, 0.0);
      }
    }
  }
  for (sire::Size i{0}; i < n; ++i) {
    sire::Size idx = preservedPairsIdx[i];
    DLOG(DEBUG) << "id: " << penetration_pairs[idx].id_A << " "
                << penetration_pairs[idx].id_B << " v_contact " << v0[3 * i]
                << " " << v0[3 * i + 1] << " ft1: " << result.ft[2 * i]
                << " ft2: " << result.ft[2 * i + 1] << " fn: " << result.fn[idx]
                << " depth: " << penetration_pairs[idx].depth
                << " pos: " << penetration_pairs[idx].p_WC.transpose()
                << " n1: " << penetration_pairs[idx].p_WCa.transpose()
                << " n2: " << penetration_pairs[idx].p_WCb.transpose();
  }
  imp_->prevResult = result;
  // for (sire::Size i{0}; i < n; ++i) {
  //   sire::Size idx = preservedPairsIdx[i];
  //   const common::PenetrationAsPointPair& pair = penetration_pairs[idx];
  //   result.fn[idx] = avgFce[i];
  //   if (imp_->prevResult.isEmpty_) {
  //     result.ft[2 * idx] = 0;
  //     result.ft[2 * idx + 1] = 0;
  //   } else {
  //     auto* geometry_A = enginePtr->queryGeometryPoolById(pair.id_A);
  //     auto* geometry_B = enginePtr->queryGeometryPoolById(pair.id_B);
  //     const core::PropMap& pair_prop =
  //         imp_->material_manager_->getPropMapOrDefault(
  //             {geometry_A->material(), geometry_B->material()});
  //     double* v_contact = v0.data() + 3 * i;
  //     double threshold_velocity = pair_prop.getPropValueOrDefault(
  //         "threshold_velocity", imp_->default_tv_);
  //     double friction_coefficient =
  //         pair_prop.getPropValueOrDefault("cof", imp_->default_cof_);

  //     double zero_check = 1e-8;
  //     auto safe_div = [](double number, double denominator, double
  //     zero_check,
  //                        double err_set) -> double {
  //       return std::abs(denominator) <= zero_check ? err_set
  //                                                  : number / denominator;
  //     };
  //     double vt = aris::dynamic::s_norm(2, v_contact);
  //     if (vt < zero_check) {
  //       result.ft[2 * idx] = 0;
  //       result.ft[2 * idx + 1] = 0;
  //     } else {
  //       double t1 = std::abs(safe_div(v_contact[0], v_contact[1], 1e-8,
  //       1e10)); double t2 = std::sqrt(t1 * t1 + 1);

  //       double ft{0};
  //       if (vt > threshold_velocity) {
  //         ft = std::abs(0.95 * friction_coefficient *
  //         imp_->prevResult.fn[idx]);
  //       } else {
  //         ft = std::abs(friction_coefficient * imp_->prevResult.fn[idx] *
  //                       (std::expm1(-3 * vt / threshold_velocity)));
  //       }
  //       // if (vt > threshold_velocity) {
  //       //   ft = std::abs(0.95 * friction_coefficient * result.fn[idx]);
  //       // } else {
  //       //   ft = std::abs(friction_coefficient * result.fn[idx] *
  //       //                 (std::expm1(-3 * vt / threshold_velocity)));
  //       // }
  //       result.ft[2 * idx] = -1 * aris::dynamic::s_sgn(v_contact[0]) * ft *
  //                            safe_div(t1, t2, zero_check, 0.0);
  //       result.ft[2 * idx + 1] = -1 * aris::dynamic::s_sgn(v_contact[1]) * ft
  //       *
  //                                safe_div(1, t2, zero_check, 0.0);
  //     }
  //   }
  //   // std::cout << result.ft[2 * i] << " " << result.ft[2 * i + 1] << " "
  //   //           << v_contact[0] << " " << v_contact[1] << std::endl;
  //   // result.ft[2 * idx] = 0;
  //   // result.ft[2 * idx + 1] = 0;
  //   // std::cout << "ft = " << result.ft[2 * i] << " " << result.ft[2 * i +
  //   1]
  //   //           << " vt = " << vt << " " << v_contact[0] << " " <<
  //   v_contact[1]
  //   //           << std::endl;
  //   // DLOG_IF(result.fn[i] > 1e6, DEBUG)
  //   //     << "Huge impact recorded: " << result.fn[i] << " minTime: " <<
  //   //     minTime
  //   //     << " b: " << b << " A: " << A << " x0: " << x0 << std::endl
  //   //     << "depth: " << penetration_pairs[i].depth
  //   //     << " id: " << penetration_pairs[i].id_A << " "
  //   //     << penetration_pairs[i].id_B;
  //   DLOG(DEBUG) << "id: " << penetration_pairs[idx].id_A << " "
  //               << penetration_pairs[idx].id_B << " v_contact " << v0[3 * i]
  //               << " " << v0[3 * i + 1] << " ft1: " << result.ft[2 * i]
  //               << " ft2: " << result.ft[2 * i + 1] << " fn: " <<
  //               result.fn[idx]
  //               << " depth: " << penetration_pairs[idx].depth
  //               << " pos: " << penetration_pairs[idx].p_WC.transpose();
  // }
  // imp_->prevResult = result;
}
auto AverageForceContactSolver::cptContactForce(double A, double B, double k,
                                                double D, double r, double w,
                                                double t) -> double {
  // 积分
  double first = (A * k + D * A * r + D * B * w) * r * r *
                 ((std::cos(w * t) * r) + w * std::sin(w * t)) *
                 std::exp(r * t) / (r * r + w * w);
  double second = (B * k + D * B * r - D * A * w) * r * r *
                  ((std::sin(w * t) * r) - w * std::cos(w * t)) *
                  std::exp(r * t) / (r * r - w * w);
  double force = first + second;
  // double first = (A * k * r * r + D * A * r * r * r + D * B * w * r * r) *
  //                ((std::cos(w * t) * r) + w * std::sin(w * t)) *
  //                std::exp(r * t) / (r * r + w * w);
  // double second = (B * k * r * r + D * B * r * r * r - D * A * w * r * r) *
  //                 ((std::sin(w * t) * r) - w * std::cos(w * t)) *
  //                 std::exp(r * t) / (r * r - w * w);
  // double force = first + second;
  // double force = (A * k + D * A * r + D * B * w)*((std::cos(w * t) / r) + w *
  // std::sin(w * t) / (r * r)) *
  //                    std::exp(r * t) / (1 + w * w / (r * r)) + (B * k + D * B
  //                    * r - D * A * w)*( (std::sin(w * t) / r) - w *
  //                    std::cos(w * t) / (r * r)) * std::exp(r * t) / (1 - w *
  //                    w / (r * r));
  return force;
}
auto AverageForceContactSolver::cptPenaltyODE(double contact_time,
                                              double proj_start_diff_v,
                                              double cr, double m, double k,
                                              double delta_t) -> double {
  // double dt = delta_t;
  // // penalty method with ODE
  // static double d =
  //     2 * std::abs(std::log(cr)) *
  //     std::sqrt(k * m / (aris::PI * aris::PI + std::log(cr) * std::log(cr)));
  // static double r = -d / (2 * m), w = std::sqrt((4 * k * m - d * d)) / (2 *
  // m);
  // // x_0 x_0'
  // if (contact_time < 2 * delta_t) {
  //   // dt = contact_time;
  //   // imp_->contact_x_init = sphere_pq[2];
  //   // imp_->contact_v_init = sphere_vs[2];
  //   // imp_->position_contact.push_back(0);
  //   // imp_->velocity_contact.push_back(sphere_vs[2]);
  //   // imp_->acceleration_contact.push_back(-9.8);
  //   // imp_->force_contact.push_back(0);
  // }
  // double F_ext = m * imp_->g;
  // double A = -F_ext / k, B = (proj_start_diff_v + r * F_ext / k) / w;
  // // velocity
  // double delta_v =
  //     (A * r + B * w) * std::exp(r * contact_time) *
  //         std::cos(w * contact_time) +
  //     (B * r - A * w) * std::exp(r * contact_time) * std::sin(w *
  //     contact_time);
  // double next_t = contact_time /* + dt*/;
  //
  // // double delta_x =
  // //     A * std::exp(r * next_t) * std::cos(w * next_t) +
  // //                  B * std::exp(r * next_t) * std::sin(w * next_t) + F_ext
  // /
  // //                  k;
  // double delta_x1 =
  //     A * std::exp(r * contact_time) * std::cos(w * contact_time) +
  //     B * std::exp(r * contact_time) * std::sin(w * contact_time) - F_ext /
  //     k;
  // average contact force
  // double force = m * (delta_v - sphere_vs[2]) / dt;
  //
  // sphere_pq[2] = imp_->contact_x_init + delta_x1;
  // sphere_vs[2] = delta_v;
  //
  // double F = cptContactForce(A, B, k, d, r, w, contact_time) -
  //            cptContactForce(A, B, k, d, r, w,
  //                            contact_time - delta_t) /*+ F_ext * delat_t*/;
  // return F;
  return 0;

  // imp_->position_contact.push_back(delta_x1);
  // imp_->velocity_contact.push_back(sphere_vs[2]);
  // imp_->acceleration_contact.push_back(-force / m);
  // imp_->force_contact.push_back(force - F_ext);

  // if (sphere_pq[2] > imp_->contact_x_init) {
  //  //用速度回退到初始碰撞面
  //   double temp_x = sphere_pq[2];
  //   double temp_v = sphere_vs[2];
  //   double temp_a = contact_force / m;
  //   sphere_vs[2] = std::sqrt(temp_v * temp_v + 2 * temp_a *
  //   std::abs(imp_->contact_x_init - temp_x));  // a != g sphere_pq[2] =
  //   imp_->contact_x_init; double dt_modify =
  //       std::abs((sphere_vs[2] - temp_v) / temp_a);  //退回的时间差 // a != g
  //   std::cout << "----out dt_modify" << dt_modify << std::endl;
  //   sphere_vs[2] += -imp_->g * dt_modify;//只有重力
  //   sphere_pq[2] += sphere_vs[2] * dt_modify;
  // }
}

ARIS_REGISTRATION {
  typedef sire::physics::collision::CollisionFilter& (
      AverageForceContactSolver::*CollisionFilterPoolFunc)();
  typedef sire::core::MaterialManager& (
      AverageForceContactSolver::*MaterialManagerFunc)();
  aris::core::class_<AverageForceContactSolver>("AverageForceContactSolver")
      .inherit<ContactSolver>()
      .prop("material_manager",
            &AverageForceContactSolver::resetMaterialManager,
            MaterialManagerFunc(&AverageForceContactSolver::materialManager))
      .prop("default_k", &AverageForceContactSolver::setDefaultStiffness,
            &AverageForceContactSolver::defaultStiffness)
      .prop("default_cr", &AverageForceContactSolver::setDefaultCr,
            &AverageForceContactSolver::defaultCr);
}
}  // namespace sire::physics::contact