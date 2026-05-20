#include "sire/physics/contact/ps_vs_solver2.hpp"

#include <array>
#include <cmath>
#include <fstream>
#include <map>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <clarabel.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/Eigen/Sparse>

#include <aris/core/reflection.hpp>
#include <aris/dynamic/math_matrix.hpp>
#include <aris/dynamic/model.hpp>
#include <aris/dynamic/model_force.hpp>
#include <aris/dynamic/model_interaction.hpp>
#include <aris/server/control_server.hpp>

#include "sire/core/constants.hpp"
#include "sire/core/force_screw.hpp"
#include "sire/core/material_manager.hpp"
#include "sire/core/profiler.hpp"
#include "sire/core/prop_map.hpp"
#include "sire/physics/collision/collision_detection.hpp"
#include "sire/physics/collision/collision_exists_callback.hpp"
#include "sire/physics/geometry/collidable_geometry.hpp"
#include "sire/physics/physics_engine.hpp"
#include "sire/physics/utils.hpp"
#include "sire/simulator/simulation_loop.hpp"

#include "log/easyloggingConfig.hpp"

namespace sire::physics::contact::ps_vs_solver2 {
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
      aris::dynamic::s_inv_pm_dot_v3(T_C_vec[preservedPairsIdx[i]].data(), ap_o, ap_c);
      accelExt[accelExtColIdx] = ap_c[2];
      ++accelExtColIdx;
    }
  }
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
/// @param[in] invCpi 6n x 6n 接触点惯量矩阵（contact point inertia matrix）
/// @param[out] a0ext 6n x 1 计算得到的法向外力结果，表示为 [FeA1, FeB1, ... , FeAn, FeBn]
// clang-format on
auto cptAllAccelExtVector(
    aris::dynamic::Model& model,
    const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    const std::vector<std::array<double, 16>>& T_C_vec,
    const std::vector<sire::Size>& preservedPairsIdx,
    const sire::PartId* prtIdVector, double* accelExt) -> void {
  SIRE_PROFILE_SCOPE("ps_vs/cptAllAccelExtVector");
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
      aris::dynamic::s_inv_pm_dot_v3(T_C_vec[preservedPairsIdx[i]].data(), ap_o,
                                     ap_c);
      aris::dynamic::s_vc(3, ap_c, accelExt + accelExtColIdx);
      accelExtColIdx += 3;
    }
  }
}
auto cptInitialCondition(
    sire::physics::PhysicsEngine& engine, sire::core::MaterialManager& manager,
    const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    const std::vector<std::array<double, 16>>& T_C_vec,
    const std::vector<sire::Size>& preservedPairsIdx,
    const std::vector<geometry::CollidableGeometry*>& geomPtrVector,
    double* stiffness, double* damping, double* fri_coef, double* x0,
    double* realDepthX0, double* v0) -> double {
  SIRE_PROFILE_SCOPE("ps_vs/cptInitialCondition");
  sire::Size n{preservedPairsIdx.size()};
  sire::Size n2{2 * n};
  double minStiff{1e20};
  for (int i{0}; i < n; ++i) {
    const core::PropMap& pair_prop =
        manager.getPropMapOrDefault({geomPtrVector[2 * i]->material(),
                                     geomPtrVector[2 * i + 1]->material()});
    stiffness[i] = pair_prop.getPropValueOrDefault("k", 2e8);
    damping[i] = pair_prop.getPropValueOrDefault("d", 5e3);
    fri_coef[i] = pair_prop.getPropValueOrDefault("cof", 0.3);
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
    x0[i] = pair.modifiedDepth / stiffScale;
    // 穿透速度 = - B的速度相对于A
    x0[n + i] = -v_contact[2];
    realDepthX0[i] = pair.depth / stiffScale;
    // 穿透速度 = - B的速度相对于A
    realDepthX0[n + i] = -v_contact[2];
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
  SIRE_PROFILE_SCOPE("ps_vs/cptDAECoeff");
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
  SIRE_PROFILE_SCOPE("ps_vs/findMinRootBisection");
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

  int numberSlices = 16;
  std::vector<double> pois(numberSlices * alphaVec.size());
  for (int i{0}; i < alphaVec.size(); ++i) {
    double maxValue = 8 * sire::PI / alphaVec[i];
    for (int j{0}; j < numberSlices; ++j) {
      pois[numberSlices * i + j] = (j + 1) * maxValue / numberSlices;
    }
  }

  std::sort(pois.begin(), pois.end());
  double lowerBound = 1e-40;
  // TODO: 可以设置为仿真的默认步长，超过默认步长的穿透没必要单独解接触时间了
  double upperBound = 0.1;
  // double minStartDecrease = 0;
  // double maxStartDecrease = 0;
  std::vector<double> x1t(n2 + 1);
  auto depthEnd = x1t.begin() + nContact;
  // auto velocityEnd = x1t.begin() + 2 * nContact;
  bool negativeDepthExists = false;
  DLOG(DEBUG) << "pois: " << pois;

  for (double poi : pois) {
    cptFormulaXComposeAb(n2 + 1, Ab.data(), poi, x01.data(), x1t.data());
    // DLOG(DEBUG) << "poi: " << poi << " "
    //             << (std::find_if(x1t.begin(), depthEnd,
    //                              [](double x) { return x < 0; }) == depthEnd)
    //             << " " << x1t;
    if (std::find_if(x1t.begin(), depthEnd, [](double x) { return x < 0; }) ==
        depthEnd) {
      // if (std::find_if(depthEnd, velocityEnd, [](double x) { return x < 0; })
      // !=
      //   velocityEnd) {
      //   if(minStartDecrease == 0) {
      //     minStartDecrease = poi;
      //   }
      //   maxStartDecrease = poi;
      // }
      lowerBound = poi;  // x向量元素全部大于零
      continue;
    } else {
      upperBound = poi;  // 存在负数
      negativeDepthExists = true;
      break;
    }
  }
  // 说明相邻虚部之间差的比较多，现在只用最小的虚部处理。（找到一个合理的upperbound
  // if (upperBound > 5 * lowerBound) {
  //   DLOG(DEBUG) << "Big image part gap, insert poi manually";
  //   std::vector<double> pois2;
  //   if (maxStartDecrease == 0) {
  //     throw std::runtime_error(
  //             "poi method all point is > 0 and increase");
  //   }
  //   pois2[0] = 2 * minStartDecrease;
  //   pois2[1] = 1.5 * maxStartDecrease;
  //   pois2[2] = 2 * maxStartDecrease;
  //   std::sort(pois2.begin(), pois2.end());
  //   for (double poi : pois2) {
  //     cptFormulaXComposeAb(n2 + 1, Ab.data(), poi, x01.data(), x1t.data());
  //     DLOG(DEBUG) << "poi: " << poi << " "
  //                 << (std::find_if(x1t.begin(), depthEnd,
  //                                  [](double x) { return x < 0; }) ==
  //                                  depthEnd) << " " << x1t;
  //     if (std::find_if(x1t.begin(), depthEnd, [](double x) { return x < 0; })
  //     ==
  //         depthEnd) {
  //       lowerBound = poi;  // x向量元素全部大于零
  //       continue;
  //     } else {
  //       upperBound = poi;  // 存在负数
  //       negativeDepthExists = true;
  //       break;
  //     }
  //   }

  // }
  DLOG(DEBUG) << "lowerBound: " << lowerBound << ", upperBound: " << upperBound;
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
    const std::vector<sire::Size>& preservedPairsIdx,
    geometry::CollidableGeometry** geometryPtrVector) -> void {
  auto modelPtr = engine.currentModel();
  SIRE_ASSERT(modelPtr != nullptr);
  const sire::Size n = preservedPairsIdx.size();
  for (sire::Size i{0}; i < n; ++i) {
    auto& pair = penetration_pairs[preservedPairsIdx[i]];
    geometry::CollidableGeometry* geometry_A_ptr =
        engine.queryGeometryPoolById(pair.id_A);
    geometry::CollidableGeometry* geometry_B_ptr =
        engine.queryGeometryPoolById(pair.id_B);
    SIRE_DEMAND(geometry_A_ptr != nullptr);
    SIRE_DEMAND(geometry_B_ptr != nullptr);
    // 标记 ground 相关的idx，计算cpi的真实大小
    // 默认是两个加速度a
    geometryPtrVector[2 * i] = geometry_A_ptr;
    geometryPtrVector[2 * i + 1] = geometry_B_ptr;
  }
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
auto cptInverseCpiMatrix(
    aris::dynamic::Model& model,
    const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    const std::vector<std::array<double, 16>>& T_C_vec,
    const std::vector<sire::Size>& preservedPairsIdx,
    const sire::PartId* prtIdVector, double* accelExt, double* invCpi) -> void {
  SIRE_PROFILE_SCOPE("ps_vs/cptInverseCpiMatrix");
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
  Size testForceIdxOffset = forcePool.size();
  Size n = preservedPairsIdx.size();
  // Size testForceIdxOffset = 0;
  const double fceValue = 10.0;
  // 初始化结果容器
  const Size cpiWidth = 6 * n;
  std::vector<bool> isGround(n * 2, false);
  for (Size i{0}; i < n; ++i) {
    for (Size j{0}; j < 2; ++j) {
      if (prtIdVector[2 * i + j] == model.ground().id())
        isGround[2 * i + j] = true;
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

  // 给力并计算质量矩阵
  double testFce[3][3] = {{fceValue, 0, 0}, {0, fceValue, 0}, {0, 0, fceValue}};
  // const double* gravityAs = modelPtr->environment().gravity();
  for (Size iContact{0}, cpiLineIdx{0}; iContact < n; ++iContact) {
    // 遍历行时不需要碰撞点到底是哪个 id_A or id_B
    for (Size i2 = 0; i2 < 2; ++i2) {
      if (isGround[2 * iContact + i2]) {
        // 如果是ground相关的碰撞点，记录为0
        std::fill_n(&invCpi[(6 * iContact + 3 * i2) * cpiWidth], 18 * n, 0);
        cpiLineIdx += 3;  // 记录行数
        continue;
      }
      auto& gf = dynamic_cast<aris::dynamic::GeneralForce&>(
          forcePool.at(2 * iContact + i2 + testForceIdxOffset));
      for (Size idir = 0; idir < 3; ++idir) {
        double fs[6];
        sire::core::screw::s_fpm2fs(
            testFce[idir], T_C_vec[preservedPairsIdx[iContact]].data(), fs);
        gf.setFce(fs);
        if (model.forwardDynamics()) {
          std::cout << "forward dynamic failed" << std::endl;
        }
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
            aris::dynamic::s_vs(3, accelExt + cpiColIdx, res);
            aris::dynamic::s_nv(3, 1.0 / fceValue, res);
            std::copy_n(res, 3,
                        &invCpi[(6 * iContact + 3 * i2 + idir) * cpiWidth +
                                jContact * 6 + 3 * j2]);
            cpiColIdx += 3;
          }
        }
        aris::dynamic::s_fill(1, 6, 0, const_cast<double*>(gf.fce()));
        ++cpiLineIdx;  // 记录行数
      }
    }
  }
  for (Size i = 0; i < n * 2; ++i) {
    forcePool.pop_back();
  }
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
  const double fceValue = 100.0;
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
// auto cptInverseCpiMatrix(
//     aris::dynamic::Model& model,
//     const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
//     const std::vector<std::array<double, 16>>& T_C_vec,
//     const std::vector<sire::Size>& preservedPairsIdx,
//     const sire::PartId* prtIdVector, double* accelExt, double* invCpi) ->
//     void {
//   auto init_interaction = [](aris::dynamic::Interaction& interaction,
//                              aris::dynamic::Model* m) -> void {
//     if (interaction.prtNameM().empty() && interaction.prtNameN().empty() &&
//         interaction.makNameI().empty() && interaction.makNameJ().empty())
//       return;

//     auto find_part = [m](std::string_view name) -> aris::dynamic::Part* {
//       auto found = std::find_if(
//           m->partPool().begin(), m->partPool().end(),
//           [name](const auto& part) -> bool { return part.name() == name; });
//       return found == m->partPool().end() ? nullptr : &*found;
//     };

//     auto find_marker = [](aris::dynamic::Part* part,
//                           std::string_view name) -> aris::dynamic::Marker* {
//       auto found = std::find_if(
//           part->markerPool().begin(), part->markerPool().end(),
//           [name](const auto& marker) -> bool { return marker.name() == name;
//           });
//       return found == part->markerPool().end() ? nullptr : &*found;
//     };

//     auto prt_m = find_part(interaction.prtNameM());
//     auto mak_i = find_marker(prt_m, interaction.makNameI());
//     auto prt_n = find_part(interaction.prtNameN());
//     auto mak_j = find_marker(prt_n, interaction.makNameJ());

//     interaction.setMakI(&*mak_i);
//     interaction.setMakJ(&*mak_j);
//   };
//   // 初始化一些重复使用的变量
//   auto& forcePool = model.forcePool();
//   auto& partPool = model.partPool();
//   sire::Size testForceIdxOffset = forcePool.size();

//   const sire::Size n = preservedPairsIdx.size();
//   // 给力并计算质量矩阵
//   const double fceValue = 10.0;
//   double testFce[3] = {0, 0, fceValue};
//   for (Size i{0}; i < n; ++i) {
//     for (Size j{0}; j < 2; ++j) {
//       // add generalForce to forcePool() in Model and init
//       auto& fce = forcePool.add<aris::dynamic::GeneralForce>(
//           std::string("test_f" + j
//                           ? "b"
//                           : "a" + std::to_string(i + testForceIdxOffset)),
//           &partPool.at(prtIdVector[2 * i + j]).markerPool().at(0),
//           &partPool.at(model.ground().id()).markerPool().at(0));
//       fce.resetModel(&model);
//       fce.setFce(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());
//       // force id 可以先不管
//       init_interaction(fce, &model);
//     }
//   }
//   for (Size iContact{0}, cpiLineIdx{0}; iContact < n; ++iContact) {
//     for (Size i2{0}; i2 < 2; ++i2) {
//       auto& gf = dynamic_cast<aris::dynamic::GeneralForce&>(
//           forcePool.at(2 * iContact + i2 + testForceIdxOffset));
//       double fs[6];
//       sire::core::screw::s_fpm2fs(
//           testFce, T_C_vec[preservedPairsIdx[iContact]].data(), fs);
//       gf.setFce(fs);
//       if (model.forwardDynamics())
//         std::cout << "forward dynamic failed" << std::endl;

//       for (Size jContact{0}, cpiColIdx{0}; jContact < n; ++jContact) {
//         double ap_o[3]{0}, res[3]{0};
//         const double* contactPosition =
//             penetration_pairs[preservedPairsIdx[jContact]].p_WC.data();
//         std::array<double, 6> as;
//         for (Size j2 = 0; j2 < 2; ++j2) {
//           auto& prt = partPool[prtIdVector[2 * jContact + j2]];
//           prt.getAs(as.data());
//           aris::dynamic::s_as2ap(prt.vs(), as.data(), contactPosition, ap_o);
//           aris::dynamic::s_inv_pm_dot_v3(
//               T_C_vec[preservedPairsIdx[jContact]].data(), ap_o, res);
//           invCpi[cpiLineIdx * 2 * n + cpiColIdx] = core::screw::s_safe_div(
//               res[2] - accelExt[cpiColIdx], fceValue, 1e-4);
//           ++cpiColIdx;
//         }
//       }
//       aris::dynamic::s_fill(1, 6, 0, const_cast<double*>(gf.fce()));
//       ++cpiLineIdx;
//     }
//   }
//   for (sire::Size i{0}; i < 2 * n; ++i) {
//     forcePool.pop_back();
//   }
// }
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
    std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    std::vector<std::array<double, 16>>& T_C_vec,
    std::vector<sire::Size>& preservedPairsIdx,
    std::vector<sire::PartId>& prtIdVector, std::vector<double>& accelExt,
    std::vector<double>& invCpiResult) -> void {
  cptContactFrame(penetration_pairs, T_C_vec);
  for (sire::Size i{0}; i < penetration_pairs.size(); ++i) {
    if (penetration_pairs[i].depth >= 0) {
      preservedPairsIdx.push_back(i);
    }
  }
  sire::Size n{preservedPairsIdx.size()};
  sire::Size n2{2 * n};
  prtIdVector.resize(n2, 0);
  // preprocessContactInfo(engine, penetration_pairs, preservedPairsIdx,
  //                       prtIdVector.data());
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
auto filterPairsAndPreprocessInfo(
    sire::physics::PhysicsEngine& engine,
    std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    std::vector<common::PenetrationAsPointPair>& contactEnded,
    std::vector<common::PenetrationAsPointPair>& contactNotEnd,
    std::vector<std::array<double, 16>>& T_C_vec,
    std::vector<sire::Size>& preservedPairsIdx,
    std::vector<sire::Size>& pairsNeedModifiedIdx,
    std::vector<sire::Size>& targetConditionIdx) -> void {
  for (sire::Size i{0}; i < contactNotEnd.size(); ++i) {
    auto& pair = contactNotEnd[i];
    if (auto& search =
            std::find_if(penetration_pairs.begin(), penetration_pairs.end(),
                         [&pair](const common::PenetrationAsPointPair& p) {
                           return pair.compareById(p);
                         });
        search == penetration_pairs.end()) {
      // 不存在的接触对加入pairs计算，说明已经穿出，这个时候，不加1e-4的长时间接触保护
      pairsNeedModifiedIdx.push_back(penetration_pairs.size());
      pair.modifiedDepth = (pair.modifiedDepth > 1e-8)
                               ? pair.modifiedDepth
                               : 1e-7;  // 确保不会被过滤掉
      // pair.modifiedDepth = -1;  // 确保不会被过滤掉
      penetration_pairs.push_back(pair);
      targetConditionIdx.push_back(i);
    } else {
      pairsNeedModifiedIdx.push_back(
          std::distance(penetration_pairs.begin(), search));
      // 还没穿出，这个时候就用当前的接触检测的结果就可以
      // 确保不会被过滤掉
      // 修改search的penetration_pairs的depth，随便给一个大于零的，
      // 防止被筛掉，状态已经在别处记录了
      search->modifiedDepth =
          (search->modifiedDepth > 1e-8) ? search->modifiedDepth : 1e-7;
      // pair.modifiedDepth = (pair.modifiedDepth > 0) ? pair.modifiedDepth :
      // 1e-7;  // 确保不会被过滤掉
      targetConditionIdx.push_back(i);
    }
  }
  // DLOG(DEBUG) << "after contact not end, contact pairs: ";
  // for (sire::Size i{0}; i < penetration_pairs.size(); ++i) {
  //   DLOG(DEBUG) << "contact detected id: " << penetration_pairs[i].id_A << "
  //   " << penetration_pairs[i].id_B
  //               << " real depth: " << penetration_pairs[i].depth
  //               << " modified depth: " << penetration_pairs[i].modifiedDepth;
  // }
  cptContactFrame(penetration_pairs, T_C_vec);
  std::vector<int> idxNeedDecrease(pairsNeedModifiedIdx.size(), 0);
  for (sire::Size i{0}; i < penetration_pairs.size(); ++i) {
    bool shouldFilter = false;
    if (penetration_pairs[i].modifiedDepth < 0) {
      // 如果穿透深度小于0，说明接触点已经分离
      shouldFilter = true;
    }
    if (std::abs(penetration_pairs[i].modifiedDepth) < 1e-8) {
      std::array<double, 3> v_contact;
      engine.cptContactVelocityB2A(penetration_pairs[i], T_C_vec[i], v_contact);
      if (v_contact[2] >= 0) {
        // 接触点的法向速度大于0，说明接触点正要分离
        shouldFilter = true;
      }
    }
    if (!shouldFilter) {
      preservedPairsIdx.push_back(i);
    } else {
      // 因为pairsNeedModifiedIdx的idx是基于penetration_pairs的，所以在过滤掉的点
      // 需要将后续的idx减一得到在preservedPairsIdx中的idx
      for (sire::Size j{0}; j < pairsNeedModifiedIdx.size(); ++j) {
        if (pairsNeedModifiedIdx[j] > i) {
          // 需要修改的idx不再需要
          idxNeedDecrease[j] += 1;
        }
      }
    }
  }
  for (sire::Size i{0}; i < pairsNeedModifiedIdx.size(); ++i) {
    // 需要修改的idx在preservedPairsIdx中的idx
    pairsNeedModifiedIdx[i] -= idxNeedDecrease[i];
  }
}

struct PsVsSolver2::Imp {
  std::unique_ptr<core::MaterialManager> material_manager_;
  nlohmann::json records;
  // 消耗系数
  double default_cr_;
  // 摩擦系数
  double default_cof_;
  // 速度阈值 velocity threshold
  double default_tv_;
  double default_k_;
  double default_d_;
  // ContactSolverResult prevResult;
  std::vector<common::PenetrationAsPointPair> contactEnded;
  std::vector<common::PenetrationAsPointPair> contactNotEnd;
  std::vector<double> contactNotEndCondition;  // x0 \dot{x0}
  double prevStiffScale;                       // x0 \dot{x0}

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
        default_tv_(0.1),
        prevStiffScale(1) {}
};
PsVsSolver2::PsVsSolver2() : imp_(std::make_unique<Imp>()) {}
PsVsSolver2::~PsVsSolver2() {};
SIRE_DEFINE_MOVE_CTOR_CPP(PsVsSolver2);
auto PsVsSolver2::resetMaterialManager(core::MaterialManager* manager) -> void {
  imp_->material_manager_.reset(manager);
}
auto PsVsSolver2::materialManager() -> core::MaterialManager& {
  return *imp_->material_manager_;
}
auto PsVsSolver2::setDefaultStiffness(double k) noexcept -> void {
  imp_->default_k_ = k;
}
auto PsVsSolver2::defaultStiffness() noexcept -> double {
  return imp_->default_k_;
}
auto PsVsSolver2::setDefaultCr(double cr) noexcept -> void {
  imp_->default_cr_ = cr;
}
auto PsVsSolver2::defaultCr() noexcept -> double { return imp_->default_cr_; }
auto PsVsSolver2::setDefaultVelocityThreshold(double tv) noexcept -> void {
  imp_->default_tv_ = tv;
}
auto PsVsSolver2::defaultVelocityThreshold() noexcept -> double {
  return imp_->default_tv_;
}
auto PsVsSolver2::debugByRecords() -> nlohmann::json {
  // std::ofstream file("contact_solver_result.json");
  // std::cout << "records: " << imp_->records.dump(2) << std::endl;
  return imp_->records;
}
auto cptNormalContactForceByX0X1tPos(
    Size n, const double* x0, const double* x1t,
    Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd>& cod,
    const double* b, double minTime, double stiffScale,
    std::vector<double>& contactPosFce) -> void {
  SIRE_PROFILE_SCOPE("ps_vs/cptNormalContactForceByX0X1tPos");
  // 使用目标位置计算接触力
  std::vector<double> temp1(x1t, x1t + n);
  aris::dynamic::s_vs(n, x0, temp1.data());
  // DLOG(DEBUG) << "Px1t - Px0: " << temp1;
  aris::dynamic::s_nv(n, stiffScale / minTime, temp1.data());
  // DLOG(DEBUG) << "(Px1t - Px0) * stiffScale / minTime: " << temp1;
  aris::dynamic::s_vs(n, x0 + n, temp1.data());
  // DLOG(DEBUG) << "(Px1t - Px0) * stiffScale / minTime - Vx0: " << temp1;
  aris::dynamic::s_nv(n, 1 / minTime, temp1.data());
  // DLOG(DEBUG) << "((Px1t - Px0) * stiffScale / minTime - Vx0) / minTime: "
  //             << temp1;
  aris::dynamic::s_vs(n, b + n, temp1.data());
  DLOG(DEBUG) << "((Px1t - Px0) * stiffScale / minTime - Vx0) / minTime - ba0: "
              << temp1;
  // DLOG(DEBUG) << "a0Post: " << temp1;
  Eigen::VectorXd a0tVec =
      Eigen::Map<Eigen::VectorXd>(const_cast<double*>(temp1.data()), n);
  Eigen::VectorXd contactPosForce = cod.solve(a0tVec);
  contactPosFce.assign(contactPosForce.data(),
                       contactPosForce.data() + contactPosForce.size());
}
auto cptNormalContactForceByX0X1tVel(
    Size n, const double* x0, const double* x1t,
    Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd>& cod,
    const double* b, double minTime, double stiffScale,
    std::vector<double>& contactVelFce) -> void {
  SIRE_PROFILE_SCOPE("ps_vs/cptNormalContactForceByX0X1tVel");
  // 使用目标速度计算接触力
  std::vector<double> temp2(x1t + n, x1t + 2 * n);
  aris::dynamic::s_vs(n, x0 + n, temp2.data());
  DLOG(DEBUG) << "Vx1t - Vx0: " << temp2;
  aris::dynamic::s_nv(n, 1 / minTime, temp2.data());
  // DLOG(DEBUG) << "(Vx1t - Vx0) / minTime: " << temp2;
  aris::dynamic::s_vs(n, b + n, temp2.data());
  DLOG(DEBUG) << "(Vx1t - Vx0) / minTime - ba0: " << temp2;
  Eigen::VectorXd a0VeltVec =
      Eigen::Map<Eigen::VectorXd>(const_cast<double*>(temp2.data()), n);
  Eigen::VectorXd contactVelForce = cod.solve(a0VeltVec);
  contactVelFce.assign(contactVelForce.data(),
                       contactVelForce.data() + contactVelForce.size());
}

auto PsVsSolver2::cptContactSolverResult(
    const aris::dynamic::Model* current_state,
    std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    std::vector<std::array<double, 16>>& T_C_vec, ContactSolverResult& result)
    -> void {
  SIRE_PROFILE_FUNCTION();
  double nextCtrlSimSuggestDt = result.dt;
  auto enginePtr = physicsEnginePtr();
  SIRE_ASSERT(enginePtr != nullptr);
  auto modelPtr = enginePtr->currentModel();
  SIRE_ASSERT(modelPtr != nullptr);
  auto& partPool = modelPtr->partPool();
  std::vector<sire::Size> preservedPairsIdx;
  std::vector<sire::Size> pairsNeedModifiedIdx;
  std::vector<sire::Size> targetConditionIdx;
  {
    SIRE_PROFILE_SCOPE("ps_vs/filterPairsAndPreprocessInfo");
    filterPairsAndPreprocessInfo(
        *enginePtr, penetration_pairs, imp_->contactEnded, imp_->contactNotEnd,
        T_C_vec, preservedPairsIdx, pairsNeedModifiedIdx, targetConditionIdx);
  }
  sire::Size n{preservedPairsIdx.size()};
  SIRE_PROFILE_PLOT("ps_vs.n_contacts", static_cast<double>(n));

  if (n == 0) {
    imp_->contactNotEnd.clear();
    imp_->contactEnded.clear();
    imp_->contactNotEndCondition.clear();
    sire::simulator::SimulationLoop* simulator_ptr = enginePtr->simLoopPtr();
    simulator_ptr->recorder().recordModelState(*modelPtr);
    std::unique_ptr<core::EventBase> eventPtr{nullptr};
    DLOG(DEBUG) << "nextCtrlSimSuggestDt: " << nextCtrlSimSuggestDt
                << " suggestDt: " << result.dt;
    if (nextCtrlSimSuggestDt - result.dt > 1e-6) {
      // 添加 stepEvents
      eventPtr = simulator_ptr->eventManager().createEventById(1);
      eventPtr->eventProp().addProp("isCtrl", 0.0);
    } else {
      core::EventId nextEventId = simulator_ptr->eventManager().nextEventId();
      eventPtr = simulator_ptr->eventManager().createEventById(nextEventId);
      eventPtr->eventProp().addProp("isCtrl", (nextEventId == 2) ? 1.0 : 0.0);
    }
    eventPtr->eventProp().addProp("dt", result.dt);
    double dt = result.dt;
    simulator_ptr->recorder().recordDt(dt);
    simulator_ptr->integratorPoolPtr()->at(0).updPs(dt);
    double currentTime = simulator_ptr->timer().updateSimTime(dt);
    simulator_ptr->recorder().addRecord(simulator_ptr->timer().simTime());
    DLOG(DEBUG) << "current time: " << simulator_ptr->timer().simTime();
    simulator_ptr->eventManager().updateCtrlSimTime(eventPtr->eventId(),
                                                    currentTime);
    // simulator_ptr->model()->setTime(currentTime);
    simulator_ptr->eventManager().addEvent(std::move(eventPtr));
    return;
  }
  sire::Size n2{2 * n};
  std::vector<geometry::CollidableGeometry*> geomPtrVector(n2, nullptr);
  preprocessContactInfo(*enginePtr, penetration_pairs, preservedPairsIdx,
                        geomPtrVector.data());

  std::vector<sire::Size> prtIdVector(n2, 0);
  std::vector<int> prtIdVector2(n2, 0);
  for (sire::Size i{0}; i < n2; ++i) {
    prtIdVector[i] = geomPtrVector[i]->partId();
    prtIdVector2[i] = static_cast<int>(prtIdVector[i]);
  }

  enginePtr->activateContactForce(false);
  // std::vector<double> allAccelExt(6 * n, 0);
  // cptAllAccelExtVector(*modelPtr, penetration_pairs, T_C_vec,
  // preservedPairsIdx,
  //                      prtIdVector.data(), allAccelExt.data());

  // std::vector<double> allInvCpiResult(36 * n * n, 0);
  // cptInverseCpiMatrix(*modelPtr, penetration_pairs, T_C_vec,
  // preservedPairsIdx,
  //                     prtIdVector.data(), allAccelExt.data(),
  //                     allInvCpiResult.data());
  std::vector<double> T_c(16 * n, 0), contactPoint(3 * n, 0);
  std::vector<double> allInvCpiResult, allAccelExt;
  {
    SIRE_PROFILE_SCOPE("ps_vs/cptContactInverseInertiaMatrix");
    for (sire::Size i{0}; i < n; ++i) {
      std::copy(T_C_vec[preservedPairsIdx[i]].begin(),
                T_C_vec[preservedPairsIdx[i]].end(), T_c.begin() + 16 * i);
      std::copy(penetration_pairs[preservedPairsIdx[i]].p_WC.begin(),
                penetration_pairs[preservedPairsIdx[i]].p_WC.end(),
                contactPoint.begin() + 3 * i);
    }
    dynamic_cast<aris::dynamic::ForwardDynamicSolver&>(
        modelPtr->solverPool()[3])
        .cptContactInverseInertiaMatrix(n, prtIdVector2.data(), T_c.data(),
                                        contactPoint.data(), allInvCpiResult,
                                        allAccelExt);
  }

  // SIRE_ASSERT(aris::dynamic::s_is_equal(36 * n * n, allInvCpiResult.data(),
  // allInvCpiResult2.data(), 1e-7)); DLOG(DEBUG) << "allAccelExt: " <<
  // allAccelExt; DLOG(DEBUG) << "allAccelExt2: " << allAccelExt2;
  // SIRE_ASSERT(aris::dynamic::s_is_equal(6 * n , allAccelExt.data(),
  // allAccelExt2.data(), 1e-7));
  enginePtr->activateContactForce(true);

  std::vector<double> invCpi(n2 * n2, 0);
  for (sire::Size i{0}; i < n; ++i)
    for (sire::Size j{0}; j < n; ++j)
      for (sire::Size i2{0}; i2 < 2; ++i2)
        for (sire::Size j2{0}; j2 < 2; ++j2)
          invCpi[2 * n * (2 * i + i2) + 2 * j + j2] =
              allInvCpiResult[6 * n * (6 * i + 3 * i2 + 2) + 6 * j + 3 * j2 +
                              2];

  std::vector<double> accelExt(n2, 0);
  for (sire::Size i{0}; i < n; ++i)
    for (sire::Size i2{0}; i2 < 2; ++i2)
      accelExt[2 * i + i2] = allAccelExt[6 * i + 3 * i2 + 2];

  DLOG(DEBUG) << "invCpi = " << invCpi << " accelExt = " << accelExt;
  std::vector<double> stiffness(n), damping(n), fri_coef(n), x0(n2),
      realDepthX0(n2), v0(3 * n);
  double stiffScale = cptInitialCondition(
      *enginePtr, *(imp_->material_manager_), penetration_pairs, T_C_vec,
      preservedPairsIdx, geomPtrVector, stiffness.data(), damping.data(),
      fri_coef.data(), x0.data(), realDepthX0.data(), v0.data());
  // std::vector<double> realX0(x0);
  // nlohmann::json realContactCptInfo;
  // for (sire::Size i{0}; i < preservedPairsIdx.size(); ++i) {
  //   nlohmann::json contactCptInfo;
  //   auto& pair = penetration_pairs[preservedPairsIdx[i]];
  //   // 计算接触力
  //   contactCptInfo["pair"] = {
  //       {"id_A", pair.id_A},
  //       {"id_B", pair.id_B},
  //   };
  //   contactCptInfo["depth"] = x0[i] * stiffScale;
  //   contactCptInfo["realDepth"] = realDepthX0[i] * stiffScale;
  //   contactCptInfo["velocity"] = x0[i + n];
  //   realContactCptInfo.push_back(contactCptInfo);
  // }
  // imp_->records["realContactState"].push_back(realContactCptInfo);
  for (sire::Size i{0}; i < pairsNeedModifiedIdx.size(); ++i) {
    sire::Size idx = pairsNeedModifiedIdx[i];
    x0[idx] = imp_->contactNotEndCondition[2 * targetConditionIdx[i]] *
              (imp_->prevStiffScale / stiffScale);
    x0[n + idx] = imp_->contactNotEndCondition[2 * targetConditionIdx[i] + 1];
  }
  // nlohmann::json modifiedContactCptInfo;
  // for (sire::Size i{0}; i < preservedPairsIdx.size(); ++i) {
  //   nlohmann::json contactCptInfo;
  //   auto& pair = penetration_pairs[preservedPairsIdx[i]];
  //   // 计算接触力
  //   contactCptInfo["pair"] = {
  //       {"id_A", pair.id_A},
  //       {"id_B", pair.id_B},
  //   };
  //   contactCptInfo["depth"] = x0[i] * stiffScale;
  //   contactCptInfo["velocity"] = x0[i + n];
  //   modifiedContactCptInfo.push_back(contactCptInfo);
  // }
  // imp_->records["modifiedContactState"].push_back(modifiedContactCptInfo);
  imp_->prevStiffScale = stiffScale;
  imp_->contactNotEnd.clear();
  imp_->contactEnded.clear();
  imp_->contactNotEndCondition.clear();
  // cpi因为要去掉ground，所以可能不是n2的，但是最后相减之后应该是 n 的
  // 对于算出来的cpi，在算逆前先类似得到矩阵A的处理一下（相减）应该就可以，
  // 同时PrtExtForce也不用管。
  // remove ground related cpi and fext;
  // 注意 cpi 可能是奇数，因为要去掉相应的ground，但A一定是偶数矩阵
  std::vector<double> A(n2 * n2), b(n2);
  cptDAECoeff(*enginePtr, n, stiffness.data(), damping.data(), stiffScale,
              accelExt.data(), invCpi.data(), A.data(), b.data());
  // 因为矩阵 A 经常无逆，所以使用其增广形式 [A b; 0 0] 作为状态转移矩阵，x0 =
  // [x0 1] 作为初始状态（求微分方程解的微分部分）
  double minTime =
      findMinRootBisection(n, A.data(), b.data(), x0.data(), 1e-10, 200);
  DLOG(DEBUG) << " minTime: " << minTime << " b: " << b << " A: " << A
              << " x0: " << x0 << " stiffScale: " << stiffScale;
  imp_->records["currentTime"].push_back(modelPtr->time());
  imp_->records["minTime"].push_back(minTime);
  // 没有零点的情况下，取A中的最大值作为参考计算步长（修改为采用suggest_dt作为步长，不变result.dt）
  if (minTime <= 0) {
    minTime = result.dt;
  } else {
    // 判断使用哪个时间
    if (minTime > result.dt) {
      minTime = result.dt;
    } else {
      result.dt = minTime;
    }
  }
  // 计算未穿出的点
  std::vector<double> Ab((n2 + 1) * (n2 + 1), 0), x01(n2 + 1), x1t(n2 + 1);
  sire::core::screw::matrixVectorComposeBack(n2, A.data(), b.data(), Ab.data());
  std::copy(x0.data(), x0.data() + n2, x01.data());
  x01[n2] = 1;
  DLOG(DEBUG) << "Ab: " << Ab << " x01: " << x01 << " minTime: " << minTime;
  int n3 = n2 + 1;
  std::vector<double> temp(n3 * n3);
  // aris::dynamic::s_mc(n3, n3, minTime, Ab.data(), temp.data());              // temp = At
  // core::screw::matrix_exp_pade(n3, temp.data(), temp.data());  // temp = e^At
  // DLOG(DEBUG) << "e^At: " << temp;
  // DLOG(DEBUG) << "n3: " << n3 << " temp: " << temp << " x01: " << x01 << " x1t before mm: " << x1t;
  // aris::dynamic::s_mm(n3, 1, n3, temp.data(), x01.data(), x1t.data());        // e^At * x01
  // DLOG(DEBUG) << "x1t after mm: " << x1t;
  cptFormulaXComposeAb(n2 + 1, Ab.data(), minTime, x01.data(), x1t.data());
  // DLOG(DEBUG) << "x01: " << x01 << " x1t: " << x1t;
  for (sire::Size i{0}; i < n; ++i) {
    if (x1t[i] >= 1e-10) {
      // 由于碰撞点not end，但是计算出来的末位置条件会比较苛刻，调整计算接触力的 
      // 目标条件为 x1t.depth = 1e-4 + x1t.depth.
      imp_->contactNotEnd.push_back(penetration_pairs[preservedPairsIdx[i]]);
      imp_->contactNotEnd.back().modifiedDepth = x1t[i] * stiffScale;
      imp_->contactNotEnd.back().depth = x1t[i] * stiffScale;
      // x1t[i] += 1e-4 / stiffScale;
      imp_->contactNotEndCondition.push_back(x1t[i]);
      imp_->contactNotEndCondition.push_back(x1t[n + i]);
    } else {
      imp_->contactEnded.push_back(penetration_pairs[preservedPairsIdx[i]]);
    }
  }
  DLOG(DEBUG) << "Contact points velocity: " << v0;
  // for (auto& pair : imp_->contactNotEnd) {
  //   DLOG(DEBUG) << "Not end id: " << pair.id_A << " " << pair.id_B
  //               << " real depth: " << pair.depth
  //               << " modified depth: " << pair.modifiedDepth;
  // }
  // DLOG(DEBUG) << imp_->contactNotEnd.size() << " contact(s) not end, "
  //             << "with condition: " << imp_->contactNotEndCondition;
  // // DLOG(DEBUG) << "x1t: " << x1t;
  // for (auto& pair : imp_->contactEnded) {
  //   DLOG(DEBUG) << "Ended id: " << pair.id_A << " " << pair.id_B
  //               << " real depth: " << pair.depth
  //               << " modified depth: " << pair.modifiedDepth;
  // }
  // DLOG(DEBUG) << imp_->contactEnded.size() << " contact(s) ended. ";

  std::vector<double> invM2(9 * n * n, 0);
  for (sire::Size i{0}; i < n; ++i)
    for (sire::Size j{0}; j < n; ++j)
      for (sire::Size k{0}; k < 3; ++k)
        for (sire::Size l{0}; l < 3; ++l)
          invM2[9 * n * i + 3 * n * k + 3 * j + l] =
              allInvCpiResult[6 * n * (6 * i + k) + 6 * j + l + 3] +
              allInvCpiResult[6 * n * (6 * i + k + 3) + 6 * j + l] -
              allInvCpiResult[6 * n * (6 * i + k) + 6 * j + l] -
              allInvCpiResult[6 * n * (6 * i + k + 3) + 6 * j + l + 3];

  std::vector<double> accelExt2(3 * n, 0);
  for (sire::Size i{0}; i < n; ++i)
    for (sire::Size k{0}; k < 3; ++k)
      accelExt2[3 * i + k] =
          allAccelExt[6 * i + k] - allAccelExt[6 * i + k + 3];

  DLOG(DEBUG) << "Real x0: " << realDepthX0;
  std::vector<double> contactFce(3 * n, 0);
  sire::simulator::SimulationLoop* simulator_ptr = enginePtr->simLoopPtr();
  simulator_ptr->recorder().recordModelState(*modelPtr);
  std::unique_ptr<core::EventBase> eventPtr{nullptr};
  DLOG(DEBUG) << "nextCtrlSimSuggestDt: " << nextCtrlSimSuggestDt
              << " suggestDt: " << result.dt;
  if (nextCtrlSimSuggestDt - result.dt > 1e-6) {
    // 添加 stepEvents
    eventPtr = simulator_ptr->eventManager().createEventById(1);
    eventPtr->eventProp().addProp("isCtrl", 0.0);
  } else {
    core::EventId nextEventId = simulator_ptr->eventManager().nextEventId();
    eventPtr = simulator_ptr->eventManager().createEventById(nextEventId);
    eventPtr->eventProp().addProp("isCtrl", (nextEventId == 2) ? 1.0 : 0.0);
  }
  eventPtr->eventProp().addProp("dt", result.dt);

  double dt = result.dt;
  simulator_ptr->recorder().recordDt(dt);
  DLOG(DEBUG) << "dt: " << dt << "x1t: " << x1t;
  // std::vector<double> vTargetPos(x1t.data(), x1t.data() + n);
  // aris::dynamic::s_vs(n, x0.data(), vTargetPos.data());
  // // DLOG(DEBUG) << "Px1t - Px0: " << temp1;
  // aris::dynamic::s_nv(n, stiffScale / minTime, vTargetPos.data());
  // DLOG(DEBUG) << "(Px1t - Px0) * stiffScale / minTime: " << temp1;

  // cptContactForceWithTargetState2(n, fri_coeff, invM2, v0, vTargetPos,
  //                                 accelExt2, minTime, contactFce);
  // cptGlbContactWrench(*modelPtr, *enginePtr, contactFce, penetration_pairs,
  //                     T_C_vec, preservedPairsIdx);
  // // // record vs before updPs
  // std::vector<std::array<double, 6>> partVsBeforeUpdPs(partPool.size());
  // for (sire::Size i{0}; i < partPool.size(); ++i) {
  //   auto& part = partPool.at(i);
  //   part.getVs(partVsBeforeUpdPs[i].data());
  // }
  // simulator_ptr->integratorPoolPtr()->at(0).updPs(dt);
  std::vector<double> vTargetVel(x1t.data() + n, x1t.data() + 2 * n);
  // DLOG(DEBUG) << "vTargetVel: " << vTargetVel << ;
  cptContactForceWithTargetState2(n, fri_coef, invM2, v0, vTargetVel, accelExt2,
                                  minTime, contactFce);
  cptGlbContactWrench(*modelPtr, *enginePtr, contactFce, penetration_pairs,
                      T_C_vec, preservedPairsIdx, geomPtrVector);
  // restore part vs before updVs
  // for (sire::Size i{0}; i < partPool.size(); ++i) {
  //   auto& part = partPool.at(i);
  //   part.setVs(partVsBeforeUpdPs[i].data());
  // }
  simulator_ptr->integratorPoolPtr()->at(0).updPs(dt);

  // DLOG(DEBUG) << "----------- ctrl integrate with dt " << dt << "
  // -----------";
  double currentTime = simulator_ptr->timer().updateSimTime(dt);
  simulator_ptr->recorder().addRecord(simulator_ptr->timer().simTime());
  DLOG(DEBUG) << "current time: " << simulator_ptr->timer().simTime();
  simulator_ptr->eventManager().updateCtrlSimTime(eventPtr->eventId(),
                                                  currentTime);
  simulator_ptr->eventManager().addEvent(std::move(eventPtr));
  simulator_ptr->model()->setTime(currentTime);
}

auto cptContactForceWithTargetState2(
    sire::Size n, std::vector<double>& fri_coef, std::vector<double>& invM,
    std::vector<double>& v0, std::vector<double>& v_target,
    std::vector<double>& b, double h, std::vector<double>& contactFce,
    sire::Size max_iters, double max_err) -> double {
  // std::vector<double> invMD(n * 3 * n * 3, 0);  // Need W size (3n * 3n)
  // // Let's rely on the block structure.
  // // Wait, the arguments are: n (number of pairs), invM (3n x 3n), D (3n x
  // 3n). aris::dynamic::s_mm(3 * n, 3 * n, 3 * n, invM.data(), D.data(),
  // invMD.data());
  SIRE_PROFILE_SCOPE("ps_vs/cptContactForceWithTargetState2");
  // invM already negative, so we can directly use it to compute P
  Eigen::Map<Eigen::MatrixXd> WMat(invM.data(), 3 * n, 3 * n);
  Eigen::MatrixXd P = - h * 0.5 * (WMat + WMat.transpose());
  // Eigen::MatrixXd P = -h * 0.5 * (WMat + WMat.transpose());
  Eigen::Map<Eigen::VectorXd> vFree(v0.data(), 3 * n), bVec(b.data(), 3 * n),
      vTarget(v_target.data(), n);
  vFree -= h * bVec;

  // std::vector<int> idx_n(n, 0), idx_t(2 * n, 0);
  // for (int i = 0; i < n; ++i) {
  //   idx_n[i] = 3 * i + 2;  // Normal force index
  //   idx_t[2 * i] = 3 * i + 0;  // Tangential force index
  //   idx_t[2 * i + 1] = 3 * i + 1;  // Tangential force index
  // }
  // 获取法向 (n) 与切向 (t) 的索引数组
  std::vector<int> idx_n, idx_t;
  idx_n.reserve(n);
  idx_t.reserve(2 * n);
  for (int i = 0; i < n; ++i) {
    idx_t.push_back(3 * i + 0);  // X 轴切向
    idx_t.push_back(3 * i + 1);  // Y 轴切向
    idx_n.push_back(3 * i + 2);  // Z 轴法向
  }

  int dim_n = idx_n.size();
  int dim_t = idx_t.size();
  Eigen::MatrixXd P_nn(dim_n, dim_n), P_nt(dim_n, dim_t), P_tt(dim_t, dim_t);
  Eigen::VectorXd q_n(dim_n), q_t(dim_t);

  for (int i = 0; i < dim_n; ++i) {
    q_n(i) = vFree(idx_n[i]);
    for (int j = 0; j < dim_n; ++j) P_nn(i, j) = P(idx_n[i], idx_n[j]);
    for (int j = 0; j < dim_t; ++j) P_nt(i, j) = P(idx_n[i], idx_t[j]);
  }
  for (int i = 0; i < dim_t; ++i) {
    q_t(i) = vFree(idx_t[i]);
    for (int j = 0; j < dim_t; ++j) P_tt(i, j) = P(idx_t[i], idx_t[j]);
  }

  Eigen::LDLT<Eigen::MatrixXd> P_nn_ldlt = P_nn.ldlt();

  Eigen::SparseMatrix<double> P_tt_sparse =
      Eigen::MatrixXd(P_tt.triangularView<Eigen::Upper>()).sparseView();
  P_tt_sparse.makeCompressed();

  std::vector<Eigen::Triplet<double>> triplets;
  for (int i = 0; i < n; ++i) {
    triplets.push_back(Eigen::Triplet<double>(3 * i + 1, 2 * i + 0, -1.0));
    triplets.push_back(Eigen::Triplet<double>(3 * i + 2, 2 * i + 1, -1.0));
  }
  Eigen::SparseMatrix<double> A_sparse(3 * n, 2 * n);
  A_sparse.setFromTriplets(triplets.begin(), triplets.end());
  A_sparse.makeCompressed();

  std::vector<clarabel::SupportedConeT<double>> cones;
  for (int i = 0; i < n; ++i)
    cones.push_back(clarabel::SecondOrderConeT<double>(3));

  clarabel::DefaultSettings<double> settings =
      clarabel::DefaultSettingsBuilder<double>::default_settings().build();
  settings.verbose = false;

  Eigen::VectorXd q_tt_current = q_t;
  Eigen::VectorXd b_eigen = Eigen::VectorXd::Zero(3 * n);
  clarabel::DefaultSolver<double> solver(P_tt_sparse, q_tt_current, A_sparse,
                                         b_eigen, cones, settings);

  Eigen::VectorXd fn_val = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd ft_val = Eigen::VectorXd::Zero(2 * n);

  // int outer_iters_used = 0;
  // double clarabel_inner_iter_sum = 0.0;
  // double clarabel_solve_time_sum = 0.0;
  // double clarabel_solve_time_max = 0.0;
  // clarabel::SolverStatus clarabel_last_status =
  // clarabel::SolverStatus::Unsolved;

  // Basic Iteration
  double error = -1;
  for (int iter = 1; iter <= max_iters; ++iter) {
    // outer_iters_used = iter;
    // Eigen::VectorXd rhs = vTarget + q_n - P_nt * ft_val;
    Eigen::VectorXd rhs = - vTarget - q_n - P_nt * ft_val;
    Eigen::VectorXd fn_new = P_nn_ldlt.solve(rhs);
    fn_new = fn_new.cwiseMax(0.0);
    fn_val = fn_new;

    q_tt_current.noalias() = P_nt.transpose() * fn_val + q_t;
    b_eigen.setZero();
    for (int i = 0; i < n; ++i) b_eigen[3 * i] = fri_coef[i] * fn_val(i);

    solver.update_q(q_tt_current);
    solver.update_b(b_eigen);
    solver.solve();

    // auto info = solver.info();
    // clarabel_last_status = info.status;
    // clarabel_inner_iter_sum += static_cast<double>(info.iterations);
    // clarabel_solve_time_sum += info.solve_time;
    // if (info.solve_time > clarabel_solve_time_max) {
    //   clarabel_solve_time_max = info.solve_time;
    // }
    // SIRE_PROFILE_PLOT("ps_vs.contact.outer_iter", static_cast<double>(iter));
    // SIRE_PROFILE_PLOT("ps_vs.contact.clarabel_inner_iter",
    //                   static_cast<double>(info.iterations));
    // SIRE_PROFILE_PLOT("ps_vs.contact.clarabel_solve_time_ms",
    //                   info.solve_time * 1e3);

    Eigen::VectorXd ft_new(2 * n);
    for (int i = 0; i < 2 * n; ++i) ft_new(i) = solver.solution().x[i];
    error = (ft_new - ft_val).norm();
    // SIRE_PROFILE_PLOT("ps_vs.contact.outer_error", error);
    if (error < max_err) {
      ft_val = ft_new;
      // Eigen::VectorXd f_opt = Eigen::VectorXd::Zero(3 * n);
      for (int i = 0; i < n; ++i) {
        contactFce[3 * i] = ft_val(2 * i);
        contactFce[3 * i + 1] = ft_val(2 * i + 1);
        contactFce[3 * i + 2] = fn_val(i);
      }
      break;
    }
    ft_val = ft_new;
  }

  // const double outer_iters_d = static_cast<double>(outer_iters_used);
  // const double clarabel_inner_iter_mean =
  //     (outer_iters_used > 0) ? (clarabel_inner_iter_sum / outer_iters_d) :
  //     0.0;
  // const double clarabel_solve_time_mean_ms =
  //     (outer_iters_used > 0) ? ((clarabel_solve_time_sum / outer_iters_d) *
  //     1e3)
  //                            : 0.0;
  // const double clarabel_solve_time_max_ms = clarabel_solve_time_max * 1e3;
  // SIRE_PROFILE_PLOT("ps_vs.contact.outer_iters_used", outer_iters_d);
  // SIRE_PROFILE_PLOT("ps_vs.contact.clarabel_inner_iter_mean",
  //                   clarabel_inner_iter_mean);
  // SIRE_PROFILE_PLOT("ps_vs.contact.clarabel_solve_time_mean_ms",
  //                   clarabel_solve_time_mean_ms);
  // SIRE_PROFILE_PLOT("ps_vs.contact.clarabel_solve_time_max_ms",
  //                   clarabel_solve_time_max_ms);
  return error;
}

auto cptGlbContactWrench(
    aris::dynamic::Model& model, sire::physics::PhysicsEngine& engine,
    const std::vector<double>& contactFce,
    const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    const std::vector<std::array<double, 16>>& T_C_vec,
    const std::vector<sire::Size>& preservedPairsIdx,
    const std::vector<geometry::CollidableGeometry*>& geomPtrVector) -> void {
  SIRE_PROFILE_SCOPE("ps_vs/cptGlbContactWrench");
  engine.resetPartContactForce();
  const sire::Size contact_force_offset = engine.contactForceIdx();
  auto& force_pool = model.forcePool();
  sire::Size n{preservedPairsIdx.size()};
  for (int i = 0; i < n; ++i) {
    // std::cout << "fn=" << fn[i] << " ";
    const auto& pair = penetration_pairs[preservedPairsIdx[i]];
    // f of contact based on contact frame;
    double f_Bc_C[3]{contactFce[3 * i], contactFce[3 * i + 1],
                     contactFce[3 * i + 2]};
    // 将接触坐标系下的力转换到世界坐标系
    double fs[6];
    core::screw::s_fpm2fs(f_Bc_C, T_C_vec.at(preservedPairsIdx[i]).data(), fs);
    DLOG(DEBUG) << "Contact force: " << f_Bc_C[0] << " " << f_Bc_C[1] << " "
                << f_Bc_C[2] << ", in fs: " << fs[0] << " " << fs[1] << " "
                << fs[2] << " " << fs[3] << " " << fs[4] << " " << fs[5];
    aris::dynamic::GeneralForce& force_A =
        dynamic_cast<aris::dynamic::GeneralForce&>(force_pool.at(
            geomPtrVector[2 * i]->partId() + contact_force_offset));
    aris::dynamic::GeneralForce& force_B =
        dynamic_cast<aris::dynamic::GeneralForce&>(force_pool.at(
            geomPtrVector[2 * i + 1]->partId() + contact_force_offset));
    double fs_A[6]{0};
    aris::dynamic::s_vc(6, force_A.fce(), fs_A);
    aris::dynamic::s_vs(6, fs, fs_A);
    force_A.setFce(fs_A);
    double fs_B[6]{0};
    aris::dynamic::s_vc(6, force_B.fce(), fs_B);
    aris::dynamic::s_va(6, fs, fs_B);
    force_B.setFce(fs_B);
  }
}

ARIS_REGISTRATION {
  typedef sire::physics::collision::CollisionFilter& (
      PsVsSolver2::*CollisionFilterPoolFunc)();
  typedef sire::core::MaterialManager& (PsVsSolver2::*MaterialManagerFunc)();
  aris::core::class_<PsVsSolver2>("PsVsSolver2")
      .inherit<ContactSolver>()
      .prop("material_manager", &PsVsSolver2::resetMaterialManager,
            MaterialManagerFunc(&PsVsSolver2::materialManager))
      .prop("default_k", &PsVsSolver2::setDefaultStiffness,
            &PsVsSolver2::defaultStiffness)
      .prop("default_cr", &PsVsSolver2::setDefaultCr, &PsVsSolver2::defaultCr);
}
}  // namespace sire::physics::contact::ps_vs_solver2