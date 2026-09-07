#include "sire/physics/contact/ps_vs_solver3.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
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

namespace sire::physics::contact::ps_vs_solver3 {
using MatrixXdRM = Eigen::Matrix<double, -1, -1, Eigen::RowMajor>;

// 前向声明：解析法向力求解（不用 Clarabel，COD 分解）
auto cptContactForceWithTargetState(
    sire::Size n, std::vector<double>& fri_coef, std::vector<double>& invM_3n,
    std::vector<double>& v0, std::vector<double>& v_target,
    std::vector<double>& b, double h, std::vector<double>& contactFce,
    sire::Size max_iters, double max_err) -> double;

/// @brief 最大耗散原理版本：摩擦方向迭代至与终点切向速度自洽
/// 与 cptContactForceWithTargetState 的区别：
///   - 旧版：摩擦方向锁定在步初 v_t(0)；新版：迭代使方向对齐 v_t'(f)
///   - 旧版：单次 COD 求解；新版：外层固定点迭代 + 每轮重求 fn
///   - 新版返回值 = 最终摩擦方向变化量（收敛指标），旧版始终返回 0
auto cptContactForceWithTargetState3(
    sire::Size n, std::vector<double>& fri_coef, std::vector<double>& invM_3n,
    std::vector<double>& v0, std::vector<double>& v_target,
    std::vector<double>& b, double h, std::vector<double>& contactFce,
    sire::Size max_iters, double max_err) -> double;

/// @brief 逐接触点 SOR + 粘/滑二分法（v4）
auto cptContactForceWithTargetState4(
    sire::Size n, std::vector<double>& fri_coef, std::vector<double>& invM_3n,
    std::vector<double>& v0, std::vector<double>& v_target,
    std::vector<double>& b, double h, std::vector<double>& contactFce,
    sire::Size max_iters, double max_err) -> double;

// TODO: 临时将 DLOG 改为 LOG(INFO) 以在 Release 模式下输出调试信息后恢复
// #undef DLOG
// #define DLOG(level) LOG(INFO)

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
    minStiff = minStiff < stiffness[i] ? minStiff : stiffness[i];
    damping[i] = pair_prop.getPropValueOrDefault("d", 5e3);
    fri_coef[i] = pair_prop.getPropValueOrDefault("cof", 0.3);
  }
  double stiffScale = std::pow(10, -std::round(std::log10(minStiff) / 2));
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
namespace {
// Dense Taylor output on a local segment. The baseline scan grid is retained:
// uncertain signs use exponential action rather than skipping a candidate.
struct PolynomialStats {
  int prepares{0}, normSkips{0}, errorFailures{0}, degreeFailures{0};
  int uncertainSigns{0}, arithmeticFailures{0}, retrySkips{0}, fixedSkips{0};
};
class ContactTimePolynomial {
 public:
  ContactTimePolynomial(sire::Size n, const double* matrix, PolynomialStats& stats)
      : n_(n), matrix_(matrix), stats_(stats) {
    for (sire::Size i = 0; i < n; ++i) {
      double row = 0;
      for (sire::Size j = 0; j < n; ++j) row += std::abs(matrix[i*n+j]);
      norm_ = std::max(norm_, row);
    }
  }
  void invalidate() { valid_ = false; }
  bool tryScan(double start, double time, double maxHorizon,
               const std::vector<double>& state, sire::Size contacts,
               std::vector<double>& out) {
    const double interval = time - start;
    if (valid_ && time >= start_ && time - start_ <= horizon_) {
      if (evaluate(time, contacts, out)) return true;
      // Do not rebuild the same uncertain point. Exponential action decides it.
      retryBelow_ = std::min(retryBelow_, interval);
      invalidate();
      return false;
    }
    invalidate();
    if (interval >= retryBelow_ || stats_.prepares >= 16) {
      ++stats_.retrySkips;
      return false;
    }
    // Matrix is fixed for this solve. Reject impossible horizons before
    // entering a preparation zone or allocating coefficients.
    double horizon = maxHorizon;
    if (!std::isfinite(norm_) || interval * norm_ > 16) {
      ++stats_.normSkips;
      return false;
    }
    if (norm_ > 0)
      horizon = std::min(horizon, std::nextafter(16 / norm_, 0.0));
    if (horizon < interval) { ++stats_.normSkips; return false; }
    // Try intermediate lengths instead of jumping from a long segment to
    // one sample. Bound failed work per scan AND across the whole solve.
    for (int attempt = 0; attempt < 4 && stats_.prepares < 16; ++attempt) {
      if (attempt == 3) horizon = interval;
      if (prepare(start, horizon, state)) {
        if (evaluate(time, contacts, out)) return true;
        break;
      }
      if (horizon <= interval) break;
      horizon = std::max(interval, horizon / 2);
    }
    // This is a performance heuristic, not a claim that a changed state
    // cannot succeed. Larger later intervals take the reliable baseline;
    // a shorter final interval can still retry if the work budget permits.
    retryBelow_ = std::min(retryBelow_, interval);
    invalidate();
    return false;
  }
 private:
  bool prepare(double start, double horizon, const std::vector<double>& state) {
    SIRE_PROFILE_SCOPE("ps_vs/contactEndTime/polynomialPrepare");
    ++stats_.prepares;
    valid_ = false;
    const double rho = horizon * norm_;
    if (!(horizon > 0) || !std::isfinite(rho)) { ++stats_.arithmeticFailures; return false; }
    // Large norm alone is not proof of difficult dynamics, but limits the
    // usefulness of this inexpensive remainder estimate. Defer to baseline.
    if (rho > 16) { ++stats_.normSkips; return false; }
    coefficients_.resize((maxDegree + 2) * n_);
    std::copy(state.begin(), state.end(), coefficients_.begin());
    const double eps = std::numeric_limits<double>::epsilon();
    const double gamma = 4 * (static_cast<double>(n_) + 2) * eps;
    double previousNorm = 0, coefficientError = 0;
    for (double value : state) previousNorm = std::max(previousNorm, std::abs(value));
    const double budget = 64 * eps * std::max(1.0, previousNorm);
    error_ = 0;
    for (int k = 1; k <= maxDegree + 1; ++k) {
      double termNorm = 0;
      for (sire::Size i = 0; i < n_; ++i) {
        double value = 0;
        for (sire::Size j = 0; j < n_; ++j)
          value += matrix_[i*n_+j] * coefficients_[(k-1)*n_+j];
        value *= horizon / k;
        if (!std::isfinite(value)) { ++stats_.arithmeticFailures; return false; }
        coefficients_[k*n_+i] = value;
        termNorm = std::max(termNorm, std::abs(value));
      }
      coefficientError = (rho / k) * coefficientError +
                         gamma * (rho / k) * previousNorm;
      const double ratio = rho / (k + 1);
      // The k-th coefficient is the first omitted term. Subsequent terms
      // are bounded by a geometric majorant of their factorial recurrence.
      if (ratio < 1 && (termNorm + coefficientError) / (1-ratio) <= budget) {
        degree_ = k - 1;
        error_ += (termNorm + coefficientError) / (1-ratio);
        start_ = start; horizon_ = horizon;
        valid_ = std::isfinite(error_) && error_ <= budget;
        if (!valid_) ++stats_.errorFailures;
        return valid_;
      }
      error_ += coefficientError;
      // This accumulated bound cannot decrease at later degrees.
      if (!std::isfinite(error_) || error_ > budget) {
        ++stats_.errorFailures;
        return false;
      }
      previousNorm = termNorm;
    }
    ++stats_.degreeFailures;
    return false;
  }
  bool evaluate(double time, sire::Size contacts, std::vector<double>& out) const {
    if (!valid_ || time < start_ || time - start_ > horizon_) return false;
    SIRE_PROFILE_SCOPE("ps_vs/contactEndTime/polynomialEvaluate");
    const double u = (time - start_) / horizon_;
    const double gamma = 4 * (degree_ + 1) * std::numeric_limits<double>::epsilon();
    for (sire::Size i = 0; i < n_; ++i) {
      double value = coefficients_[degree_*n_+i], magnitude = std::abs(value);
      for (int k = degree_; k-- > 0;) {
        value = value * u + coefficients_[k*n_+i];
        magnitude = magnitude * u + std::abs(coefficients_[k*n_+i]);
      }
      out[i] = value;
      if (!std::isfinite(value)) { ++stats_.arithmeticFailures; return false; }
      // Include an inflated roundoff estimate, not just truncation error.
      // Near-zero contacts always use the baseline sign calculation.
      if (i < contacts && std::abs(value) <= error_ + gamma * magnitude) {
        ++stats_.uncertainSigns;
        return false;
      }
    }
    return true;
  }
 private:
  static constexpr int maxDegree = 32;
  sire::Size n_;
  const double* matrix_;
  PolynomialStats& stats_;
  double retryBelow_{std::numeric_limits<double>::infinity()};
  double norm_{0}, start_{0}, horizon_{0}, error_{0};
  int degree_{0};
  bool valid_{false};
  std::vector<double> coefficients_;
};
}  // namespace

// clang-format off
/// @brief 搜索本步最早分离时间；固定间隔扫描复用完整状态转移矩阵，
/// 变间隔、末尾补齐和 Brent 求根局部推进，负穿深从初值复核。
// clang-format on
auto findSinglePointContactEndTime(sire::Size nContact, double suggestDt,
                                   const double* A, const double* b,
                                   const double* x0, const std::string& method) -> double {
  SIRE_PROFILE_SCOPE("ps_vs/findSinglePointContactEndTime");
  if (method != "exponential" && method != "polynomial")
    throw std::invalid_argument("Unknown contact time method");
  const bool usePolynomial = method == "polynomial";
  if (!std::isfinite(suggestDt) || suggestDt <= 0) {
    throw std::invalid_argument("Contact time requires a positive finite step");
  }
  if (nContact == 0) return -1;
  const sire::Size n2 = 2 * nContact, n3 = n2 + 1;
  std::vector<double> Ab(n3 * n3, 0), x01(n3), xt(n3);
  core::screw::matrixVectorComposeBack(n2, A, b, Ab.data());
  std::copy(x0, x0 + n2, x01.begin());
  x01[n2] = 1;

  // Eigenvalues determine only the sampling scale. Both direct evaluation
  // and fixed-step propagation retain the full coupling and affine forcing.
  double fastestRate = 0, fastestFrequency = 0;
  {
    SIRE_PROFILE_SCOPE("ps_vs/contactEndTime/spectrum");
    Eigen::MatrixXd matrix = Eigen::Map<const MatrixXdRM>(A, n2, n2);
    Eigen::EigenSolver<Eigen::MatrixXd> spectrum(matrix, false);
    if (spectrum.info() != Eigen::Success) {
      throw std::runtime_error("Cannot determine contact time sampling scale");
    }
    for (Eigen::Index i = 0; i < spectrum.eigenvalues().size(); ++i) {
      const auto eigenvalue = spectrum.eigenvalues()[i];
      fastestRate = std::max(fastestRate, std::abs(eigenvalue));
      fastestFrequency = std::max(fastestFrequency, std::abs(eigenvalue.imag()));
    }
  }
  double maxStep = suggestDt / 32;
  if (fastestFrequency > 0) {
    maxStep = std::min(maxStep, sire::PI / (16 * fastestFrequency));
  }
  double scanStep = maxStep;
  if (fastestRate > 0) scanStep = std::min(scanStep, 1 / (32 * fastestRate));
  // Resolve a fixed fraction of the current search horizon, with an absolute
  // 10 ns cap for ordinary simulation steps.  Using a fixed ultra-small
  // tolerance makes event-refined steps do just as much root work even when
  // their entire horizon is only 1e-7 s.  The machine floor is expressed at
  // the scale of the global simulation clock, where the returned dt is added.
  const double timeTolerance = std::max(
      64 * std::numeric_limits<double>::epsilon() *
          std::max(1.0, std::abs(suggestDt)),
      std::min(1e-8, suggestDt * 1e-4));
  int scanEvaluations = 0, rootEvaluations = 0, interpolationSteps = 0;
  int bisectionSteps = 0, confirmations = 0;
  int polynomialHits = 0, polynomialFallbacks = 0, polynomialSegments = 0;
  PolynomialStats polynomialStats;
  auto finish = [&](double result) {
    if (usePolynomial) {
      SIRE_PROFILE_PLOT("contact_time.poly_prepare_calls", static_cast<double>(polynomialStats.prepares));
      SIRE_PROFILE_PLOT("contact_time.poly_norm_skips", static_cast<double>(polynomialStats.normSkips));
      SIRE_PROFILE_PLOT("contact_time.poly_error_failures", static_cast<double>(polynomialStats.errorFailures));
      SIRE_PROFILE_PLOT("contact_time.poly_degree_failures", static_cast<double>(polynomialStats.degreeFailures));
      SIRE_PROFILE_PLOT("contact_time.poly_uncertain_signs", static_cast<double>(polynomialStats.uncertainSigns));
      SIRE_PROFILE_PLOT("contact_time.poly_arithmetic_failures", static_cast<double>(polynomialStats.arithmeticFailures));
      SIRE_PROFILE_PLOT("contact_time.poly_retry_skips", static_cast<double>(polynomialStats.retrySkips));
      SIRE_PROFILE_PLOT("contact_time.poly_fixed_skips", static_cast<double>(polynomialStats.fixedSkips));
    }
    SIRE_PROFILE_PLOT("contact_time.polynomial_hits", static_cast<double>(polynomialHits));
    SIRE_PROFILE_PLOT("contact_time.polynomial_fallbacks", static_cast<double>(polynomialFallbacks));
    SIRE_PROFILE_PLOT("contact_time.polynomial_segments", static_cast<double>(polynomialSegments));
    SIRE_PROFILE_PLOT("contact_time.n_contacts", static_cast<double>(nContact));
    SIRE_PROFILE_PLOT("contact_time.suggest_dt", suggestDt);
    SIRE_PROFILE_PLOT("contact_time.max_rate", fastestRate);
    SIRE_PROFILE_PLOT("contact_time.max_frequency", fastestFrequency);
    SIRE_PROFILE_PLOT("contact_time.max_scan_step", maxStep);
    SIRE_PROFILE_PLOT("contact_time.time_tolerance", timeTolerance);
    SIRE_PROFILE_PLOT("contact_time.scan_evaluations", static_cast<double>(scanEvaluations));
    SIRE_PROFILE_PLOT("contact_time.root_evaluations", static_cast<double>(rootEvaluations));
    SIRE_PROFILE_PLOT("contact_time.interpolation_steps", static_cast<double>(interpolationSteps));
    SIRE_PROFILE_PLOT("contact_time.bisection_steps", static_cast<double>(bisectionSteps));
    SIRE_PROFILE_PLOT("contact_time.confirmations", static_cast<double>(confirmations));
    SIRE_PROFILE_PLOT("contact_time.separation_found", result > 0 ? 1.0 : 0.0);
    return result;
  };
  core::screw::MatrixExpMultiplyWorkspace action(n3, Ab.data());
  core::screw::MatrixExpPadeWorkspace fixedPade(n3);
  // Polynomial data and all matrix workspaces are local to this solve.
  std::unique_ptr<ContactTimePolynomial> polynomial;
  if (usePolynomial) polynomial = std::make_unique<ContactTimePolynomial>(n3, Ab.data(), polynomialStats);
  auto minimumDepth = [&](const std::vector<double>& state) {
    return *std::min_element(state.begin(), state.begin() + nContact);
  };
  auto hasNegativeDepth = [&]() {
    for (double value : xt) {
      if (!std::isfinite(value)) {
        throw std::runtime_error("Non-finite state while finding contact end time");
      }
    }
    for (sire::Size i = 0; i < nContact; ++i) {
      if (xt[i] < 0) return true;
    }
    return false;
  };
  auto anyNegativeDepth = [&](double time) {
    ++confirmations;
    {
      SIRE_PROFILE_SCOPE("ps_vs/contactEndTime/exponentialAction");
      action.apply(x01.data(), xt.data(), time);
    }
    return hasNegativeDepth();
  };

  // Do not treat a new impact's initial zero depth as separation. Resolve
  // fast damped transients, then grow the scan intervals to the frequency
  // cap. Always include suggestDt, including for purely real eigenvalues.
  {
    SIRE_PROFILE_SCOPE("ps_vs/contactEndTime/scan");
    // Local to this call: A, b and the initial state may change next step.
    // Keep input/output vectors separate because s_mm is not in-place safe.
    std::vector<double> scanState = x01;
    std::vector<double> fixedTransition;
    double lower = 0;
    while (lower < suggestDt) {
      ++scanEvaluations;
      const double upper = std::min(suggestDt, lower + scanStep);
      if (upper <= lower) {
        throw std::runtime_error("Contact time scan cannot advance");
      }
      bool negativeDepth;
      bool polynomialAccepted = false;
      const bool fullFixedInterval = scanStep == maxStep && lower + scanStep <= suggestDt;
      if (polynomial && fullFixedInterval && !fixedTransition.empty()) {
        // Once paid for, the fixed transition is cheaper than preparing a
        // replacement. The shortened final interval may still use polynomial.
        ++polynomialStats.fixedSkips;
        polynomial->invalidate();
      } else if (polynomial) {
        const int before = polynomialStats.prepares;
        polynomialAccepted = polynomial->tryScan(
            lower, upper, std::min(maxStep * 4, suggestDt - lower),
            scanState, nContact, xt);
        if (polynomialStats.prepares != before) ++polynomialSegments;
        if (polynomialAccepted) ++polynomialHits;
        else ++polynomialFallbacks;
      }
      // Keep the original sampling times. Only full maxStep intervals reuse
      // exp(Ab*maxStep); other intervals use a local exponential action.
      if (polynomialAccepted) {
        negativeDepth = hasNegativeDepth();
      } else if (fullFixedInterval) {
        if (fixedTransition.empty()) {
          SIRE_PROFILE_SCOPE("ps_vs/contactEndTime/fixedStepExponential");
          fixedTransition.resize(n3 * n3);
          fixedPade.apply(Ab.data(), fixedTransition.data(), maxStep);
        }
        {
          SIRE_PROFILE_SCOPE("ps_vs/contactEndTime/fixedStepAdvance");
          aris::dynamic::s_mm(n3, 1, n3, fixedTransition.data(), scanState.data(),
                              xt.data());
        }
        negativeDepth = hasNegativeDepth();
      } else {
        {
          SIRE_PROFILE_SCOPE("ps_vs/contactEndTime/scanAction");
          action.apply(scanState.data(), xt.data(), upper - lower);
        }
        negativeDepth = hasNegativeDepth();
      }
      // Confirm every candidate from x(0), including variable/final intervals.
      // If rejected, xt holds the directly recomputed state and the swap below
      // resets accumulated propagation error before continuing the scan.
      if (negativeDepth) {
        negativeDepth = anyNegativeDepth(upper);
        if (polynomial) polynomial->invalidate();
      }
      if (negativeDepth) {
        SIRE_PROFILE_SCOPE("ps_vs/contactEndTime/brent");
        double left = lower, right = upper;
        std::vector<double> leftState = scanState;

        // Brent-Dekker on min_i depth_i(t): its sign is exactly the previous
        // any-negative predicate. a,b are the bracket endpoints with b the
        // better residual; c,d track previous trials for progress safeguards.
        // Keep a separate chronological bracket for forward-only propagation.
        double a = left, b = right;
        double fa = minimumDepth(leftState), fb = minimumDepth(xt);
        if (std::abs(fa) < std::abs(fb)) {
          std::swap(a, b);
          std::swap(fa, fb);
        }
        double c = a, fc = fa, d = c;
        bool lastWasBisection = true;
        bool zeroProbeUsed = false;
        for (int iteration = 0; iteration < 80 && right - left > timeTolerance;
             ++iteration) {
          double trial = left + (right - left) / 2;
          bool bisect = true;
          // A zero endpoint may be the exact crossing or a flat zero-depth
          // contact. Probe towards the negative endpoint once; never accept
          // zero itself as separation or repeatedly crawl across a plateau.
          if (iteration < 32 && !zeroProbeUsed && (fa == 0.0 || fb == 0.0)) {
            const double zeroTime = fa == 0.0 ? a : b;
            trial = zeroTime + timeTolerance / 2;
            bisect = !(trial > left && trial < right);
            zeroProbeUsed = true;
          }
          // Zero depth is not a separation event: a newly touching or
          // persistently zero-depth contact must not terminate the search.
          // Reserve the last 48 iterations for bisection. Since the initial
          // width <= suggestDt/32 and tol >= 32*eps*suggestDt, that is enough
          // to reach the original width tolerance if interpolation stalls.
          if (iteration < 32 && fa != 0.0 && fb != 0.0 && fa != fb) {
            const double scale = std::max(std::abs(fa),
                                          std::max(std::abs(fb), std::abs(fc)));
            const double va = fa / scale, vb = fb / scale, vc = fc / scale;
            double candidate;
            if (va != vc && vb != vc && va != vb) {
              // Inverse quadratic interpolation in coordinates relative to b.
              candidate = b + (a - b) * vb * vc / ((va - vb) * (va - vc))
                            + (c - b) * va * vb / ((vc - va) * (vc - vb));
            } else {
              candidate = b - (b - a) * vb / (vb - va);
            }
            const double guard = a + (b - a) / 4;
            const double history = lastWasBisection ? std::abs(b - c)
                                                    : std::abs(c - d);
            if (std::isfinite(candidate) &&
                candidate > std::min(guard, b) && candidate < std::max(guard, b) &&
                history > timeTolerance && std::abs(candidate - b) < history / 2) {
              // Avoid a rounded repeat of b while retaining a strict bracket.
              trial = std::abs(candidate - b) < timeTolerance / 2
                          ? b + std::copysign(timeTolerance / 2, a - b)
                          : candidate;
              bisect = !(trial > left && trial < right);
            }
          }
          if (bisect) {
            trial = left + (right - left) / 2;
            ++bisectionSteps;
          } else {
            ++interpolationSteps;
          }
          ++rootEvaluations;
          {
            SIRE_PROFILE_SCOPE("ps_vs/contactEndTime/rootAction");
            action.apply(leftState.data(), xt.data(), trial - left);
          }
          const bool trialNegative = hasNegativeDepth();
          const double ftrial = minimumDepth(xt);
          if (trialNegative) {
            right = trial;
          } else {
            left = trial;
            leftState.swap(xt);
          }
          d = c;
          c = b;
          fc = fb;
          if ((fa < 0.0) != trialNegative) {
            b = trial;
            fb = ftrial;
          } else {
            a = trial;
            fa = ftrial;
          }
          if (std::abs(fa) < std::abs(fb)) {
            std::swap(a, b);
            std::swap(fa, fb);
          }
          lastWasBisection = bisect;
        }
        // The positive upper bracket avoids a zero step and reaches separation.
        return finish(right);
      }
      scanState.swap(xt);
      lower = upper;
      scanStep = std::min(maxStep, scanStep * 1.5);
    }
  }
  return finish(-1);
}

auto findMinRootOriginal(sire::Size nContact, double suggestDt, const double* A,
                         const double* b, const double* x0, double tolerance,
                         sire::Size maxIter) -> double {
  SIRE_PROFILE_SCOPE("ps_vs/findMinRootOriginal");
  const sire::Size n2 = 2 * nContact;
  const sire::Size n3 = n2 + 1;
  std::vector<double> Ab(n3 * n3, 0), x01(n3);
  sire::core::screw::matrixVectorComposeBack(n2, A, b, Ab.data());
  std::copy(x0, x0 + n2, x01.data());
  x01[n2] = 1;

  Eigen::MatrixXd Ab_eig =
      Eigen::Map<MatrixXdRM>(const_cast<double*>(Ab.data()), n3, n3);
  Eigen::EigenSolver<Eigen::MatrixXd> es(Ab_eig, false);
  Eigen::VectorXd absImgPrt = es.eigenvalues().imag().cwiseAbs();
  std::vector<double> alphaVec;
  std::copy_if(absImgPrt.data(), absImgPrt.data() + n3,
               std::back_inserter(alphaVec),
               [tolerance](double i) { return i > tolerance; });
  std::sort(alphaVec.begin(), alphaVec.end(), std::greater<double>());
  alphaVec.erase(std::unique(alphaVec.begin(), alphaVec.end(),
                             [tolerance](double a, double b) {
                               return std::abs(a - b) < tolerance;
                             }),
                 alphaVec.end());
  if (alphaVec.size() == 0) {
    DLOG(DEBUG) << "Contact without split, no imaginary part";
    return -1;
  }

  int numberSlices = 16;
  std::vector<double> pois(numberSlices * alphaVec.size());
  for (int i{0}; i < (int)alphaVec.size(); ++i) {
    double maxValue = 8 * sire::PI / alphaVec[i];
    for (int j{0}; j < numberSlices; ++j)
      pois[numberSlices * i + j] = (j + 1) * maxValue / numberSlices;
  }
  std::sort(pois.begin(), pois.end());

  double lowerBound = 1e-20;
  double upperBound = 0.1;
  std::vector<double> x1t(n3);
  auto depthEnd = x1t.begin() + nContact;
  bool negativeDepthExists = false;
  DLOG(DEBUG) << "pois: " << pois;

  for (double poi : pois) {
    cptFormulaXComposeAb(n3, Ab.data(), poi, x01.data(), x1t.data());
    if (std::find_if(x1t.begin(), depthEnd, [](double x) { return x < 0; }) ==
        depthEnd) {
      lowerBound = poi;
      continue;
    } else {
      upperBound = poi;
      negativeDepthExists = true;
      break;
    }
  }

  DLOG(DEBUG) << "lowerBound: " << lowerBound << " upperBound: " << upperBound;
  if (!negativeDepthExists) {
    DLOG(WARNING) << "Contact without split, negative depth not exists";
    return -1;
  }

  double m{-1};
  int i{0};
  for (; i < maxIter; ++i) {
    if (upperBound - lowerBound < 1e-7) break;
    m = (lowerBound + upperBound) / 2;
    cptFormulaXComposeAb(n3, Ab.data(), m, x01.data(), x1t.data());
    if (std::find_if(x1t.begin(), depthEnd, [tolerance](double x) {
          return std::abs(x) < tolerance;
        }) != depthEnd) {
      if (std::find_if(x1t.begin(), depthEnd, [tolerance](double x) {
            return x < -tolerance;
          }) == depthEnd) {
        break;
      }
    }
    if (std::find_if(x1t.begin(), depthEnd, [](double x) { return x < 0; }) ==
        depthEnd) {
      lowerBound = m;
    } else {
      upperBound = m;
    }
  }
  if (i == maxIter) m = -1;
  return m;
}

// clang-format off
/// @brief Eigendecomposition 版本：一次 EigenSolver 分解 + O(n²) 求值。
/// 速度比原始版快 ~50x，数值精度足够支撑 tolerance=1e-10。
// clang-format on
auto findMinRootBisection(sire::Size nContact, double suggestDt,
                          const double* A, const double* b, const double* x0,
                          double tolerance, sire::Size maxIter) -> double {
  SIRE_PROFILE_SCOPE("ps_vs/findMinRootBisection");
  const sire::Size n2 = 2 * nContact;
  const sire::Size n3 = n2 + 1;
  std::vector<double> Ab(n3 * n3, 0), x01(n3);
  sire::core::screw::matrixVectorComposeBack(n2, A, b, Ab.data());
  std::copy(x0, x0 + n2, x01.data());
  x01[n2] = 1;

  Eigen::MatrixXd AbMat = Eigen::Map<MatrixXdRM>(Ab.data(), n3, n3);
  Eigen::EigenSolver<Eigen::MatrixXd> es(AbMat, true);
  Eigen::VectorXcd lambdas = es.eigenvalues();

  Eigen::VectorXd absImgPrt = lambdas.imag().cwiseAbs();
  std::vector<double> alphaVec;
  std::copy_if(absImgPrt.data(), absImgPrt.data() + n3,
               std::back_inserter(alphaVec),
               [tolerance](double i) { return i > tolerance; });
  std::sort(alphaVec.begin(), alphaVec.end(), std::greater<double>());
  alphaVec.erase(std::unique(alphaVec.begin(), alphaVec.end(),
                             [tolerance](double a, double b) {
                               return std::abs(a - b) < tolerance;
                             }),
                 alphaVec.end());
  if (alphaVec.size() == 0) {
    DLOG(DEBUG) << "Contact without split, no imaginary part";
    return -1;
  }

  Eigen::MatrixXcd V = es.eigenvectors();
  Eigen::VectorXcd c = V.colPivHouseholderQr().solve(
      Eigen::Map<Eigen::VectorXd>(x01.data(), n3).cast<std::complex<double>>());

  auto anyDepthNegative = [&](double t) -> bool {
    for (sire::Size i{0}; i < nContact; ++i) {
      std::complex<double> sum{0, 0};
      for (sire::Size j{0}; j < n3; ++j)
        sum += V(i, j) * std::exp(lambdas(j) * t) * c(j);
      if (sum.real() < 0) return true;
    }
    return false;
  };

  auto evalDepths = [&](double t, std::vector<double>& depths) -> void {
    for (sire::Size i{0}; i < nContact; ++i) {
      std::complex<double> sum{0, 0};
      for (sire::Size j{0}; j < n3; ++j)
        sum += V(i, j) * std::exp(lambdas(j) * t) * c(j);
      depths[i] = sum.real();
    }
  };

  int numberSlices = 16;
  std::vector<double> pois(numberSlices * alphaVec.size());
  for (int i{0}; i < (int)alphaVec.size(); ++i) {
    double maxValue = 8 * sire::PI / alphaVec[i];
    for (int j{0}; j < numberSlices; ++j)
      pois[numberSlices * i + j] = (j + 1) * maxValue / numberSlices;
  }
  std::sort(pois.begin(), pois.end());

  double lowerBound = 1e-40;
  double upperBound = 0.1;
  bool negativeDepthExists = false;

  for (double poi : pois) {
    if (anyDepthNegative(poi)) {
      upperBound = poi;
      negativeDepthExists = true;
      break;
    }
    lowerBound = poi;
  }

  if (!negativeDepthExists) {
    DLOG(WARNING) << "Contact without split, negative depth not exists";
    return -1;
  }

  double m{-1};
  int i{0};
  std::vector<double> depths(nContact);
  for (; i < maxIter; ++i) {
    if (upperBound - lowerBound < 1e-7) break;
    m = (lowerBound + upperBound) / 2;
    evalDepths(m, depths);
    auto depthEnd = depths.begin();
    if (std::find_if(depthEnd, depths.end(), [tolerance](double x) {
          return std::abs(x) < tolerance;
        }) != depths.end()) {
      if (std::find_if(depthEnd, depths.end(), [tolerance](double x) {
            return x < -tolerance;
          }) == depths.end()) {
        break;
      }
    }
    if (std::find_if(depthEnd, depths.end(), [](double x) { return x < 0; }) ==
        depths.end()) {
      lowerBound = m;
    } else {
      upperBound = m;
    }
  }
  if (i == maxIter) m = -1;
  return m;
}

// clang-format off
/// @brief RealSchur 版本：使用正交 U（κ(U)=1），闭式求 e^{Tt}。
/// 处理 Jordan 块（invCpi 零空间导致不可对角化），精度与速度兼得。
// clang-format on
auto findMinRootSchur(sire::Size nContact, double suggestDt, const double* A,
                      const double* b, const double* x0, double tolerance,
                      sire::Size maxIter) -> double {
  SIRE_PROFILE_SCOPE("ps_vs/findMinRootSchur");
  const sire::Size n2 = 2 * nContact;
  const sire::Size n3 = n2 + 1;
  // 因为矩阵 A 经常无逆，所以使用其增广形式 [A b; 0 0] 作为状态转移矩阵，x0 =
  // [x0 1] 作为初始状态（求微分方程解的微分部分）
  std::vector<double> Ab(n3 * n3, 0), x01(n3);
  sire::core::screw::matrixVectorComposeBack(n2, A, b, Ab.data());
  std::copy(x0, x0 + n2, x01.data());
  x01[n2] = 1;

  // Use RealSchur decomposition: Ab = U * T * U^T (U orthogonal, T
  // quasi-triangular). Orthogonal U has κ(U)=1, robust for Jordan blocks
  // when invCpi nullspace makes Ab non-diagonalizable.
  // For any t: x(t) = U * exp(T*t) * (U^T * x01).
  Eigen::MatrixXd AbMat = Eigen::Map<MatrixXdRM>(Ab.data(), n3, n3);
  Eigen::RealSchur<Eigen::MatrixXd> schur(AbMat);
  const Eigen::MatrixXd& U = schur.matrixU();
  const Eigen::MatrixXd& T = schur.matrixT();
  Eigen::VectorXd UTx01 =
      U.transpose() * Eigen::Map<Eigen::VectorXd>(x01.data(), n3);

  // Extract eigenvalues (1×1 or 2×2 diagonal blocks of T) for imaginary-part
  // detection. For 2×2 blocks [a b; c d], eigenvalues are (a+d)/2 ±
  // sqrt(((a-d)/2)² + bc).
  std::vector<double> alphaVec;
  for (sire::Size j{0}; j < n3; ++j) {
    if (j + 1 < n3 && T(j + 1, j) != 0.0) {
      double a = T(j, j), b = T(j, j + 1), c = T(j + 1, j), d = T(j + 1, j + 1);
      double disc = ((a - d) / 2.0) * ((a - d) / 2.0) + b * c;
      if (disc < 0) {
        double imag = std::sqrt(-disc);
        if (imag > tolerance) alphaVec.push_back(imag);
      }
      ++j;  // skip the second row of the 2×2 block
    }
    // 1×1 blocks: real eigenvalues, no imaginary part to extract
  }
  std::sort(alphaVec.begin(), alphaVec.end(), std::greater<double>());
  alphaVec.erase(std::unique(alphaVec.begin(), alphaVec.end(),
                             [tolerance](double a, double b) {
                               return std::abs(a - b) < tolerance;
                             }),
                 alphaVec.end());
  if (alphaVec.size() == 0) {
    DLOG(DEBUG) << "Contact without split, no imaginary part";
    return -1;
  }

  int numberSlices = 16;
  std::vector<double> pois(numberSlices * alphaVec.size());
  for (int i{0}; i < (int)alphaVec.size(); ++i) {
    double maxValue = 8 * sire::PI / alphaVec[i];
    for (int j{0}; j < numberSlices; ++j)
      pois[numberSlices * i + j] = (j + 1) * maxValue / numberSlices;
  }
  // 从 pois 中筛选出小于等于 suggestDt 的部分
  pois.erase(std::remove_if(pois.begin(), pois.end(),
                            [suggestDt](double x) { return x > suggestDt; }),
             pois.end());
  std::sort(pois.begin(), pois.end());

  // Precompute exp(T*t) for a given t. For 1×1 blocks: exp(λ*t).
  // For 2×2 blocks: closed-form matrix exponential.
  auto expTt = [&](double t, Eigen::MatrixXd& expT) -> void {
    expT.setZero(n3, n3);
    for (sire::Size j{0}; j < n3; ++j) {
      if (j + 1 < n3 && T(j + 1, j) != 0.0) {
        double a = T(j, j), b = T(j, j + 1), c = T(j + 1, j),
               d = T(j + 1, j + 1);
        double tr2 = (a + d) / 2.0;
        double disc = ((a - d) / 2.0) * ((a - d) / 2.0) + b * c;
        double mu = std::sqrt(std::abs(disc));
        double etr2 = std::exp(tr2 * t);
        if (mu < 1e-15 * (std::abs(a) + std::abs(d) + 1.0)) {
          // Near-defective 2×2 block (mu ≈ 0, repeated real eigenvalues):
          // exp([a b; c a]*t) = e^{at} * [1 + (a-d)t/2,  bt;  ct,  1 -
          // (a-d)t/2] Since a ≈ d, simplify to: e^{at} * [1, bt; ct, 1]
          expT(j, j) = etr2;
          expT(j, j + 1) = etr2 * b * t;
          expT(j + 1, j) = etr2 * c * t;
          expT(j + 1, j + 1) = etr2;
        } else if (disc >= 0) {
          double ch = std::cosh(mu * t), sh = std::sinh(mu * t);
          expT(j, j) = etr2 * (ch + (a - d) / (2.0 * mu) * sh);
          expT(j, j + 1) = etr2 * (b / mu) * sh;
          expT(j + 1, j) = etr2 * (c / mu) * sh;
          expT(j + 1, j + 1) = etr2 * (ch - (a - d) / (2.0 * mu) * sh);
        } else {
          double cs = std::cos(mu * t), sn = std::sin(mu * t);
          expT(j, j) = etr2 * (cs + (a - d) / (2.0 * mu) * sn);
          expT(j, j + 1) = etr2 * (b / mu) * sn;
          expT(j + 1, j) = etr2 * (c / mu) * sn;
          expT(j + 1, j + 1) = etr2 * (cs - (a - d) / (2.0 * mu) * sn);
        }
        ++j;
      } else {
        expT(j, j) = std::exp(T(j, j) * t);
      }
    }
  };

  // O(n²) evaluation of x(t) = U * exp(T*t) * (U^T * x01) for any t.
  // Fast: only multiply by exp(T*t) (quasi-triangular) and U.
  std::vector<double> expT_buf(n3 * n3);
  auto evalX = [&](double t, Eigen::VectorXd& xt) -> void {
    Eigen::MatrixXd expT = Eigen::Map<MatrixXdRM>(expT_buf.data(), n3, n3);
    expTt(t, expT);
    xt = U * (expT * UTx01);
  };

  // Returns true if any depth component < 0 at time t.
  Eigen::VectorXd xt_buf(n3);
  auto anyDepthNegative = [&](double t) -> bool {
    evalX(t, xt_buf);
    for (sire::Size i{0}; i < nContact; ++i)
      if (xt_buf(i) < 0) return true;
    return false;
  };

  // Returns the depth components at time t.
  auto evalDepths = [&](double t, std::vector<double>& depths) -> void {
    evalX(t, xt_buf);
    for (sire::Size i{0}; i < nContact; ++i) depths[i] = xt_buf(i);
  };

  double lowerBound = 1e-40;
  double upperBound = 0.1;
  bool negativeDepthExists = false;
  DLOG(DEBUG) << "pois: " << pois;
  for (double poi : pois) {
    if (anyDepthNegative(poi)) {
      upperBound = poi;
      negativeDepthExists = true;
      break;
    }
    lowerBound = poi;
  }

  DLOG(DEBUG) << "lowerBound: " << lowerBound << ", upperBound: " << upperBound;
  if (!negativeDepthExists) {
    DLOG(WARNING) << "Contact without split, negative depth not exists";
    return -1;
  }

  // bisection with O(n²) evaluation per iteration
  double m{-1};
  int i{0};
  std::vector<double> depths(nContact);
  for (; i < maxIter; ++i) {
    if (upperBound - lowerBound < 1e-7) break;
    m = (lowerBound + upperBound) / 2;
    evalDepths(m, depths);
    if (std::find_if(depths.begin(), depths.end(), [tolerance](double x) {
          return std::abs(x) < tolerance;
        }) != depths.end()) {
      if (std::find_if(depths.begin(), depths.end(), [tolerance](double x) {
            return x < -tolerance;
          }) == depths.end()) {
        break;
      }
    }
    if (std::find_if(depths.begin(), depths.end(),
                     [](double x) { return x < 0; }) == depths.end()) {
      lowerBound = m;
    } else {
      upperBound = m;
    }
  }
  if (i == maxIter) m = -1;
  DLOG(DEBUG) << "depths at m: " << depths;
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
// // TODO: 筛选接触点 — 判断两个接触点是否太近（法向差<0.5° 且 位置差<2cm）
// auto contactPairsTooClosedFlag(
//     const sire::physics::common::PenetrationAsPointPair& pairA,
//     const sire::physics::common::PenetrationAsPointPair& pairB) -> bool {
//   // 0.5 degree and 5cm difference.
//   return pairA.nhat_AB_W.cross(pairB.nhat_AB_W).norm() < 1e-2 &&
//          (pairA.p_WC - pairB.p_WC).cwiseAbs().sum() < 2e-2;
// }
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
// TODO: 筛选接触点 — filterPairsAndPreprocessInfo (简单版): 只按 depth>=0 过滤
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
// TODO: 筛选接触点 — filterPairsAndPreprocessInfo (完整版): 过滤
// modifiedDepth<0 或法向速度>=0 的接触点
auto filterPairsAndPreprocessInfo(
    sire::physics::PhysicsEngine& engine,
    std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    std::vector<common::PenetrationAsPointPair>& contactEnded,
    std::vector<common::PenetrationAsPointPair>& contactNotEnd,
    std::vector<std::array<double, 16>>& T_C_vec,
    std::vector<sire::Size>& preservedPairsIdx,
    std::vector<sire::Size>& pairsNeedModifiedIdx,
    std::vector<sire::Size>& targetConditionIdx) -> void {
  // for (sire::Size i{0}; i < contactNotEnd.size(); ++i) {
  //   auto& pair = contactNotEnd[i];
  //   if (auto& search =
  //           std::find_if(penetration_pairs.begin(), penetration_pairs.end(),
  //                        [&pair](const common::PenetrationAsPointPair& p) {
  //                          return pair.compareById(p);
  //                        });
  //       search == penetration_pairs.end()) {
  //     //
  //     不存在的接触对加入pairs计算，说明已经穿出，这个时候，不加1e-4的长时间接触保护
  //     pairsNeedModifiedIdx.push_back(penetration_pairs.size());
  //     pair.modifiedDepth = (pair.modifiedDepth > 1e-8)
  //                              ? pair.modifiedDepth
  //                              : 1e-7;  // 确保不会被过滤掉
  //     // pair.modifiedDepth = -1;  // 确保不会被过滤掉
  //     penetration_pairs.push_back(pair);
  //     targetConditionIdx.push_back(i);
  //   } else {
  //     pairsNeedModifiedIdx.push_back(
  //         std::distance(penetration_pairs.begin(), search));
  //     // 还没穿出，这个时候就用当前的接触检测的结果就可以
  //     // 确保不会被过滤掉
  //     // 修改search的penetration_pairs的depth，随便给一个大于零的，
  //     // 防止被筛掉，状态已经在别处记录了
  //     search->modifiedDepth =
  //         (search->modifiedDepth > 1e-8) ? search->modifiedDepth : 1e-7;
  //     // pair.modifiedDepth = (pair.modifiedDepth > 0) ? pair.modifiedDepth :
  //     // 1e-7;  // 确保不会被过滤掉
  //     targetConditionIdx.push_back(i);
  //   }
  // }
  // DLOG(DEBUG) << "after contact not end, contact pairs: ";
  // for (sire::Size i{0}; i < penetration_pairs.size(); ++i) {
  //   DLOG(DEBUG) << "contact detected id: " << penetration_pairs[i].id_A << "
  //   " << penetration_pairs[i].id_B
  //               << " real depth: " << penetration_pairs[i].depth
  //               << " modified depth: " << penetration_pairs[i].modifiedDepth;
  // }
  cptContactFrame(penetration_pairs, T_C_vec);
  // std::vector<int> idxNeedDecrease(pairsNeedModifiedIdx.size(), 0);
  for (sire::Size i{0}; i < penetration_pairs.size(); ++i) {
    // TODO: 筛选接触点 — 过滤
    // modifiedDepth<0（已分离）或法向速度>=0（正在分离）的接触点
    bool shouldFilter = false;
    if (penetration_pairs[i].modifiedDepth < 0) {
      // 如果穿透深度小于0，说明接触点已经分离
      shouldFilter = true;
    }
    // TODO: 注意如果接触点过多过少要改这个
    if (std::abs(penetration_pairs[i].modifiedDepth) < 1e-6) {
      std::array<double, 3> v_contact;
      engine.cptContactVelocityB2A(penetration_pairs[i], T_C_vec[i], v_contact);
      if (v_contact[2] >= 0) {
        // 接触点的法向速度大于0，说明接触点正要分离
        shouldFilter = true;
      }
    }
    if (!shouldFilter) {
      preservedPairsIdx.push_back(i);
    }
    // else {
    //   //
    //   因为pairsNeedModifiedIdx的idx是基于penetration_pairs的，所以在过滤掉的点
    //   // 需要将后续的idx减一得到在preservedPairsIdx中的idx
    //   for (sire::Size j{0}; j < pairsNeedModifiedIdx.size(); ++j) {
    //     if (pairsNeedModifiedIdx[j] > i) {
    //       // 需要修改的idx不再需要
    //       idxNeedDecrease[j] += 1;
    //     }
    //   }
    // }
  }
  // for (sire::Size i{0}; i < pairsNeedModifiedIdx.size(); ++i) {
  //   // 需要修改的idx在preservedPairsIdx中的idx
  //   pairsNeedModifiedIdx[i] -= idxNeedDecrease[i];
  // }
}

// TODO: 筛选接触点 — modifyPenetrationDepth: 将 modifiedDepth 截断为 1e-6
auto modifyPenetrationDepth(
    sire::physics::PhysicsEngine& engine,
    std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    std::vector<sire::Size>& preservedPairsIdx) -> void {
  for (sire::Size i{0}; i < preservedPairsIdx.size(); ++i) {
    auto& pair = penetration_pairs[preservedPairsIdx[i]];
    if (std::abs(pair.modifiedDepth) > 1e-6) {
      pair.modifiedDepth = 1e-6;
    }
    // 修改穿透深度为一个小于零的值，确保不会被过滤掉
  }
}

struct PsVsSolver3::Imp {
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
PsVsSolver3::PsVsSolver3() : imp_(std::make_unique<Imp>()) {}
PsVsSolver3::~PsVsSolver3() {};
SIRE_DEFINE_MOVE_CTOR_CPP(PsVsSolver3);
auto PsVsSolver3::resetMaterialManager(core::MaterialManager* manager) -> void {
  imp_->material_manager_.reset(manager);
}
auto PsVsSolver3::materialManager() -> core::MaterialManager& {
  return *imp_->material_manager_;
}
auto PsVsSolver3::setDefaultStiffness(double k) noexcept -> void {
  imp_->default_k_ = k;
}
auto PsVsSolver3::defaultStiffness() noexcept -> double {
  return imp_->default_k_;
}
auto PsVsSolver3::setDefaultCr(double cr) noexcept -> void {
  imp_->default_cr_ = cr;
}
auto PsVsSolver3::defaultCr() noexcept -> double { return imp_->default_cr_; }
auto PsVsSolver3::setDefaultVelocityThreshold(double tv) noexcept -> void {
  imp_->default_tv_ = tv;
}
auto PsVsSolver3::defaultVelocityThreshold() noexcept -> double {
  return imp_->default_tv_;
}
auto PsVsSolver3::debugByRecords() -> nlohmann::json {
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
// 既然记录在多接触点的情况下并不好用，那就不记录了，每帧消除穿深，
// 但是保留一个k d = Fext 的深度用来帮助穿透回正。
auto PsVsSolver3::cptContactSolverResult(
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
    // TODO: 筛选接触点 — 调用 filterPairsAndPreprocessInfo +
    // modifyPenetrationDepth
    SIRE_PROFILE_SCOPE("ps_vs/filterPairsAndPreprocessInfo");
    if (singlePointContactMode()) {
      prepareSinglePointContacts(penetration_pairs, T_C_vec, preservedPairsIdx);
    } else {
      filterPairsAndPreprocessInfo(
          *enginePtr, penetration_pairs, imp_->contactEnded, imp_->contactNotEnd,
          T_C_vec, preservedPairsIdx, pairsNeedModifiedIdx, targetConditionIdx);
      modifyPenetrationDepth(*enginePtr, penetration_pairs, preservedPairsIdx);
    }
  }
  sire::Size n{preservedPairsIdx.size()};
  SIRE_PROFILE_PLOT("ps_vs.n_contacts", static_cast<double>(n));
  SIRE_PROFILE_PLOT(
      "ps_vs.curr_time",
      static_cast<double>(enginePtr->simLoopPtr()->timer().simTime()));

  if (n == 0) {
    // imp_->contactNotEnd.clear();
    // imp_->contactEnded.clear();
    // imp_->contactNotEndCondition.clear();
    sire::simulator::SimulationLoop* simulator_ptr = enginePtr->simLoopPtr();
    simulator_ptr->recorder().recordModelState(*modelPtr);
    // Clear stale contact results: when n==0 no recordContactPairResults is
    // called, so the previous step's data would remain in the record and
    // lastContactPairResults() would return stale data.
    simulator_ptr->recorder().recordContactPairResults({});
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
    DLOG(DEBUG) << "current time: " << simulator_ptr->timer().simTime();
    simulator_ptr->eventManager().updateCtrlSimTime(eventPtr->eventId(),
                                                    currentTime);
    if (singlePointContactMode()) simulator_ptr->model()->setTime(currentTime);
    simulator_ptr->eventManager().addEvent(std::move(eventPtr));
    return;
  }
  for (sire::Size i{0}; i < n; ++i) {
    DLOG(DEBUG) << "contact detected id: "
                << penetration_pairs[preservedPairsIdx[i]].id_A << " "
                << penetration_pairs[preservedPairsIdx[i]].id_B
                << " real depth: "
                << penetration_pairs[preservedPairsIdx[i]].depth
                << " modified depth: "
                << penetration_pairs[preservedPairsIdx[i]].modifiedDepth
                << " contact position: "
                << penetration_pairs[preservedPairsIdx[i]].p_WC[0] << " "
                << penetration_pairs[preservedPairsIdx[i]].p_WC[1] << " "
                << penetration_pairs[preservedPairsIdx[i]].p_WC[2]
                << " contact normal: "
                << penetration_pairs[preservedPairsIdx[i]].nhat_AB_W[0] << " "
                << penetration_pairs[preservedPairsIdx[i]].nhat_AB_W[1] << " "
                << penetration_pairs[preservedPairsIdx[i]].nhat_AB_W[2];
    // DLOG(DEBUG) << "contact frame: " << T_C_vec[preservedPairsIdx[i]][0] << "
    // "
    //             << T_C_vec[preservedPairsIdx[i]][1] << " "
    //             << T_C_vec[preservedPairsIdx[i]][2] << " "
    //             << T_C_vec[preservedPairsIdx[i]][3] << " "
    //             << T_C_vec[preservedPairsIdx[i]][4] << " "
    //             << T_C_vec[preservedPairsIdx[i]][5] << " "
    //             << T_C_vec[preservedPairsIdx[i]][6] << " "
    //             << T_C_vec[preservedPairsIdx[i]][7] << " "
    //             << T_C_vec[preservedPairsIdx[i]][8] << " "
    //             << T_C_vec[preservedPairsIdx[i]][9] << " "
    //             << T_C_vec[preservedPairsIdx[i]][10] << " "
    //             << T_C_vec[preservedPairsIdx[i]][11] << " "
    //             << T_C_vec[preservedPairsIdx[i]][12] << " "
    //             << T_C_vec[preservedPairsIdx[i]][13] << " "
    //             << T_C_vec[preservedPairsIdx[i]][14] << " "
    //             << T_C_vec[preservedPairsIdx[i]][15];
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

  std::vector<double> accelExt3(n2, 0);
  // auto& fce =
  //     dynamic_cast<aris::dynamic::GeneralForce&>(modelPtr->forcePool()[0]);
  // DLOG(DEBUG) << "ball force: " << fce.fce()[0] << " " << fce.fce()[1] << " "
  //             << fce.fce()[2] << " " << fce.fce()[3] << " " << fce.fce()[4]
  //             << " " << fce.fce()[5];
  cptAccelExtVector(*modelPtr, penetration_pairs, T_C_vec, preservedPairsIdx,
                    prtIdVector.data(), accelExt3.data());

  DLOG(DEBUG) << "invCpi = " << invCpi << " accelExt = " << accelExt
              << " accelExt3 = " << accelExt3;
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
  // for (sire::Size i{0}; i < pairsNeedModifiedIdx.size(); ++i) {
  //   sire::Size idx = pairsNeedModifiedIdx[i];
  //   x0[idx] = imp_->contactNotEndCondition[2 * targetConditionIdx[i]] *
  //             (imp_->prevStiffScale / stiffScale);
  //   x0[n + idx] = imp_->contactNotEndCondition[2 * targetConditionIdx[i] +
  //   1];
  // }
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
  // Initial conditions always come from the current model and collision data.
  // imp_->contactNotEnd.clear();
  // imp_->contactEnded.clear();
  // imp_->contactNotEndCondition.clear();
  // cpi因为要去掉ground，所以可能不是n2的，但是最后相减之后应该是 n 的
  // 对于算出来的cpi，在算逆前先类似得到矩阵A的处理一下（相减）应该就可以，
  // 同时PrtExtForce也不用管。
  // remove ground related cpi and fext;
  // 注意 cpi 可能是奇数，因为要去掉相应的ground，但A一定是偶数矩阵
  auto& ball = partPool[1];
  DLOG(DEBUG) << "ball as: " << ball.as()[0] << " " << ball.as()[1] << " "
              << ball.as()[2] << " " << ball.as()[3] << " " << ball.as()[4]
              << " " << ball.as()[5];
  DLOG(DEBUG) << "ball vs: " << ball.vs()[0] << " " << ball.vs()[1] << " "
              << ball.vs()[2] << " " << ball.vs()[3] << " " << ball.vs()[4]
              << " " << ball.vs()[5];
  std::vector<double> A(n2 * n2), b(n2);
  cptDAECoeff(*enginePtr, n, stiffness.data(), damping.data(), stiffScale,
              accelExt.data(), invCpi.data(), A.data(), b.data());
  // 因为矩阵 A 经常无逆，所以使用其增广形式 [A b; 0 0] 作为状态转移矩阵，x0 =
  // [x0 1] 作为初始状态（求微分方程解的微分部分）
  double minTime = singlePointContactMode()
                       ? findSinglePointContactEndTime(
                             n, result.dt, A.data(), b.data(), x0.data(), contactTimeMethod())
                       : -1;
  DLOG(DEBUG) << " minTime: " << minTime << " b: " << b << " A: " << A
              << " x0: " << x0 << " stiffScale: " << stiffScale;
  // Solver diagnostics must not accumulate for the lifetime of an RL job.
  auto* debug_loop = enginePtr->simLoopPtr();
  if (debug_loop != nullptr && debug_loop->recorder().historyEnabled()) {
    auto& times = imp_->records["currentTime"];
    if (times.size() < 4096) {
      times.push_back(modelPtr->time());
      imp_->records["minTime"].push_back(minTime);
    }
  }
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
  // aris::dynamic::s_mc(n3, n3, minTime, Ab.data(), temp.data()); // temp = At
  // core::screw::matrix_exp_pade(n3, temp.data(), temp.data());  // temp = e^At
  // DLOG(DEBUG) << "e^At: " << temp;
  // DLOG(DEBUG) << "n3: " << n3 << " temp: " << temp << " x01: " << x01 << "
  // x1t before mm: " << x1t; aris::dynamic::s_mm(n3, 1, n3, temp.data(),
  // x01.data(), x1t.data());        // e^At * x01 DLOG(DEBUG) << "x1t after mm:
  // " << x1t;
  cptFormulaXComposeAb(n2 + 1, Ab.data(), minTime, x01.data(), x1t.data());
  // DLOG(DEBUG) << "x01: " << x01 << " x1t: " << x1t;
  // for (sire::Size i{0}; i < n; ++i) {
  //   if (x1t[i] >= 1e-10) {
  //     // 由于碰撞点not
  //     end，但是计算出来的末位置条件会比较苛刻，调整计算接触力的
  //     // 目标条件为 x1t.depth = 1e-4 + x1t.depth.
  //     imp_->contactNotEnd.push_back(penetration_pairs[preservedPairsIdx[i]]);
  //     imp_->contactNotEnd.back().modifiedDepth = x1t[i] * stiffScale;
  //     imp_->contactNotEnd.back().depth = x1t[i] * stiffScale;
  //     // x1t[i] += 1e-4 / stiffScale;
  //     imp_->contactNotEndCondition.push_back(x1t[i]);
  //     imp_->contactNotEndCondition.push_back(x1t[n + i]);
  //   } else {
  //     imp_->contactEnded.push_back(penetration_pairs[preservedPairsIdx[i]]);
  //   }
  // }
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
  std::unique_ptr<core::EventBase> eventPtr{nullptr};
  DLOG(DEBUG) << "nextCtrlSimSuggestDt: " << nextCtrlSimSuggestDt
              << " suggestDt: " << result.dt;
  if (singlePointContactMode() ? result.dt < nextCtrlSimSuggestDt
                               : nextCtrlSimSuggestDt - result.dt > 1e-6) {
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
  DLOG(DEBUG) << "vTargetVel: " << vTargetVel << " v0: " << v0;
  double error =
      cptContactForceWithTargetState4(n, fri_coef, invM2, v0, vTargetVel,
                                      accelExt2, minTime, contactFce, 30, 1e-8);
  // double error =
  //     cptContactForceWithTargetState3(n, fri_coef, invM2, v0, vTargetVel,
  //                                     accelExt2, minTime, contactFce, 30,
  //                                     1e-6);
  // double error =
  //     cptContactForceWithTargetState(n, fri_coef, invM2, v0, vTargetVel,
  //                                    accelExt2, minTime, contactFce, 30,
  //                                    1e-6);
  DLOG(DEBUG) << "Error of contact force: " << error;
  {
    SIRE_PROFILE_SCOPE("ps_vs/afterCptContactForce");
    // cptGlbContactWrench(*modelPtr, *enginePtr, contactFce, penetration_pairs,
    //                     T_C_vec, preservedPairsIdx, geomPtrVector);
    enginePtr->resetPartContactForce();
    const sire::Size contact_force_offset = enginePtr->contactForceIdx();
    auto& force_pool = modelPtr->forcePool();
    sire::Size n{preservedPairsIdx.size()};
    std::vector<sire::simulator::ContactPairResult> pairResults;
    pairResults.reserve(n);
    for (int i = 0; i < n; ++i) {
      sire::simulator::ContactPairResult r;
      r.geomIdA = geomPtrVector[2 * i]->geometryId();
      r.geomIdB = geomPtrVector[2 * i + 1]->geometryId();
      // std::cout << "fn=" << fn[i] << " ";
      const auto& pair = penetration_pairs[preservedPairsIdx[i]];
      // f of contact based on contact frame;
      double f_Bc_C[3]{contactFce[3 * i], contactFce[3 * i + 1],
                       contactFce[3 * i + 2]};
      // 将接触坐标系下的力转换到世界坐标系
      double fs[6];
      core::screw::s_fpm2fs(f_Bc_C, T_C_vec.at(preservedPairsIdx[i]).data(),
                            fs);
      // contact force in contact frame → world frame
      r.force_W[0] = fs[0];
      r.force_W[1] = fs[1];
      r.force_W[2] = fs[2];
      r.point_W[0] = pair.p_WC[0];
      r.point_W[1] = pair.p_WC[1];
      r.point_W[2] = pair.p_WC[2];
      pairResults.push_back(r);

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
    simulator_ptr->recorder().recordModelState(*modelPtr);
    simulator_ptr->recorder().recordPenetrationPairs(penetration_pairs);
    simulator_ptr->recorder().recordContactPairResults(pairResults);
    // restore part vs before updVs
    // for (sire::Size i{0}; i < partPool.size(); ++i) {
    //   auto& part = partPool.at(i);
    //   part.setVs(partVsBeforeUpdPs[i].data());
    // }
    simulator_ptr->integratorPoolPtr()->at(0).updPs(dt);

    // DLOG(DEBUG) << "----------- ctrl integrate with dt " << dt << "
    // -----------";
    double currentTime = simulator_ptr->timer().updateSimTime(dt);
    DLOG(DEBUG) << "current time: " << simulator_ptr->timer().simTime();
    simulator_ptr->eventManager().updateCtrlSimTime(eventPtr->eventId(),
                                                    currentTime);
    simulator_ptr->eventManager().addEvent(std::move(eventPtr));
    simulator_ptr->model()->setTime(currentTime);
  }
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
  SIRE_PROFILE_FUNCTION();
  std::vector<double> v0_original = v0;
  // invM already negative, so we can directly use it to compute P
  Eigen::Map<MatrixXdRM> WMat(invM.data(), 3 * n, 3 * n);
  Eigen::MatrixXd P = -h * 0.5 * (WMat + WMat.transpose());
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

  // Use COD instead of LDLT because P_nn can be rank-deficient when
  // multiple contact points share the same rigid body (e.g., 4 spheres on
  // one box → only 3 independent normal force modes).
  // Note: COD is re-created inside the loop for active-submatrix solves
  // (iterative negative-fn removal). The pre-loop decomposition is kept
  // for clarity but not used directly.

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

  int outer_iters_used = 0;
  // double clarabel_inner_iter_sum = 0.0;
  // double clarabel_solve_time_sum = 0.0;
  // double clarabel_solve_time_max = 0.0;
  // clarabel::SolverStatus clarabel_last_status =
  // clarabel::SolverStatus::Unsolved;

  // Basic Iteration
  double error = -1;
  for (int iter = 1; iter <= max_iters; ++iter) {
    outer_iters_used = iter;
    // Eigen::VectorXd rhs = vTarget + q_n - P_nt * ft_val;
    Eigen::VectorXd rhs = -vTarget - q_n - P_nt * ft_val;

    // 迭代处理负法向力：与方法1/3一致，把fn<0的接触点从P_nn子矩阵中
    // 去掉后重解，确保力在剩余接触点间正确重分配。
    // 对比cwiseMax(0)：近奇异矩阵中正负力对相互抵消（fn=[+F,-F,...]），
    // cwiseMax(0)只截负不调正 → 合力F+G≠0 → 弹飞。
    Eigen::VectorXd fn_new(n);
    {
      std::vector<bool> active(n, true);
      bool has_negative = true;
      for (int pass = 0; pass <= n && has_negative; ++pass) {
        has_negative = false;
        std::vector<int> idx;
        for (int i = 0; i < n; ++i)
          if (active[i]) idx.push_back(i);
        int na = static_cast<int>(idx.size());
        if (na == 0) {
          fn_new.setZero();
          break;
        }
        Eigen::MatrixXd P_sub(na, na);
        Eigen::VectorXd rhs_sub(na);
        for (int i = 0; i < na; ++i) {
          rhs_sub(i) = rhs(idx[i]);
          for (int j = 0; j < na; ++j) P_sub(i, j) = P_nn(idx[i], idx[j]);
        }

        // Tikhonov regularization: breaks degeneracy from redundant contacts
        // (identical pos/normal → near-singular P_sub).
        // MuJoCo's QP formulation naturally distributes forces in the nullspace
        // via the quadratic objective min(½fᵀAf).  In our explicit linear
        // solve, adding λI provides equivalent effect: redundant contacts share
        // force evenly instead of COD's arbitrary min-norm assignment. λ =
        // 1e-3×max_diag: large enough to resolve degeneracy (~1e-8×σ₁), small
        // enough to be negligible for well-conditioned contacts.
        double lambda = 1e-3 * P_sub.diagonal().cwiseAbs().maxCoeff();
        for (int i = 0; i < na; ++i) P_sub(i, i) += lambda;

        Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod_sub(P_sub);
        cod_sub.setThreshold(1e-8 * P_sub.cwiseAbs().maxCoeff());
        Eigen::VectorXd fn_sub = cod_sub.solve(rhs_sub);

        fn_new.setZero();
        for (int i = 0; i < na; ++i) {
          fn_new(idx[i]) = fn_sub(i);
          if (fn_sub(i) < 0) {
            has_negative = true;
            active[idx[i]] = false;
          }
        }
      }
    }
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
    SIRE_PROFILE_PLOT("ps_vs.contact.outer_error", error);
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

  const double outer_iters_d = static_cast<double>(outer_iters_used);
  // const double clarabel_inner_iter_mean =
  //     (outer_iters_used > 0) ? (clarabel_inner_iter_sum / outer_iters_d) :
  //     0.0;
  // const double clarabel_solve_time_mean_ms =
  //     (outer_iters_used > 0) ? ((clarabel_solve_time_sum / outer_iters_d) *
  //     1e3)
  //                            : 0.0;
  // const double clarabel_solve_time_max_ms = clarabel_solve_time_max * 1e3;
  SIRE_PROFILE_PLOT("ps_vs.contact.outer_iters_used", outer_iters_d);
  // SIRE_PROFILE_PLOT("ps_vs.contact.clarabel_inner_iter_mean",
  //                   clarabel_inner_iter_mean);
  // SIRE_PROFILE_PLOT("ps_vs.contact.clarabel_solve_time_mean_ms",
  //                   clarabel_solve_time_mean_ms);
  // SIRE_PROFILE_PLOT("ps_vs.contact.clarabel_solve_time_max_ms",
  //                   clarabel_solve_time_max_ms);
  // TODO: 临时 dump 数据到 result.json，用于 Python 对比验证，排查完成后删除
  // {
  //   nlohmann::json debugData;
  //   debugData["n"] = static_cast<int>(n);
  //   debugData["h"] = h;
  //   debugData["fri_coef"] = fri_coef;
  //   debugData["invM"] = invM;
  //   debugData["v0"] = v0_original;
  //   debugData["v_target"] = v_target;
  //   debugData["b"] = b;
  //   debugData["cResult"] = contactFce;
  //   std::ofstream f("D:\\code\\sire\\scripts\\tools\\result1.json");
  //   f << debugData.dump(2);
  // }
  return error;
}

/// @brief 解析法向力求解（不用 Clarabel）：构建摩擦矩阵 D，算 invMD = invM *
/// D， 用 COD 分解求解法向力 fn = invMD
/// \ a_target，负法向力时去掉对应摩擦项重解。 参考
/// analytical_tangent_force_solver 的实现。
auto cptContactForceWithTargetState(
    sire::Size n, std::vector<double>& fri_coef, std::vector<double>& invM_3n,
    std::vector<double>& v0, std::vector<double>& v_target,
    std::vector<double>& b, double h, std::vector<double>& contactFce,
    sire::Size max_iters, double max_err) -> double {
  SIRE_PROFILE_FUNCTION();

  // ---- Step 1: 构建 n×3n 的法向逆惯量矩阵 (提取 z 行) ----
  // invM_3n 是 3n×3n，每 3 行对应一个接触点的 x,y,z 方向。
  // 我们只需要 z 方向 (行索引 3*i+2)。
  DLOG(DEBUG) << "invM (3n*3n): " << invM_3n;
  std::vector<double> invM(n * 3 * n);
  for (sire::Size i{0}; i < n; ++i)
    for (sire::Size j{0}; j < n; ++j)
      for (sire::Size l{0}; l < 3; ++l)
        invM[3 * n * i + 3 * j + l] =
            invM_3n[9 * n * i + 3 * n * 2 + 3 * j + l];
  DLOG(DEBUG) << "invM (n*3n): " << invM;
  // ---- Step 2: 构建摩擦方向矩阵 D (3n × n) ----
  // D 的每列对应一个接触点，3 行 = (tangent_x, tangent_y, normal_z)
  std::vector<double> D(3 * n * n, 0);
  for (sire::Size i{0}; i < n; ++i) {
    double* vc = v0.data() + 3 * i;
    double vt = std::sqrt(vc[0] * vc[0] + vc[1] * vc[1]);
    double vt_reg = std::max(vt, 1e-8);
    double mu_eff = fri_coef[i];
    D[3 * i * n + i] = -(vc[0] / vt_reg) * mu_eff;
    D[(3 * i + 1) * n + i] = -(vc[1] / vt_reg) * mu_eff;
    D[(3 * i + 2) * n + i] = 1.0;
  }

  // ---- Step 3: 计算 invMD = invM * D (n × n) ----
  std::vector<double> invMD(n * n, 0);
  aris::dynamic::s_mm(n, n, 3 * n, invM.data(), D.data(), invMD.data());
  DLOG(DEBUG) << "invMD: " << invMD;
  // ---- Step 4: 目标加速度 a_target = (vContactTarget - vContact0) / h - b_n
  // ---- v_target 来自 DAE 状态（= -v_contact_z），需先转回接触坐标系约定
  // v0[3*i+2] 已经是接触坐标系约定（v_contact_z）
  std::vector<double> a_target(n);
  for (sire::Size i{0}; i < n; ++i) {
    // double v_contact_target = -v_target[i];  // DAE → 接触坐标系
    // 统统转换到相对于接触深度的速度约定
    double dv = v_target[i] + v0[3 * i + 2];
    a_target[i] = dv / h - b[3 * i + 2];
  }
  DLOG(DEBUG) << "b: " << b;
  DLOG(DEBUG) << "a_target: " << a_target;

  // ---- Step 5: COD 求解法向力 ----
  using MatrixXdRM = Eigen::Matrix<double, -1, -1, Eigen::RowMajor>;
  MatrixXdRM invMDMat = Eigen::Map<MatrixXdRM>(invMD.data(), n, n);
  Eigen::CompleteOrthogonalDecomposition<MatrixXdRM> cod(invMDMat);
  Eigen::VectorXd aVec = Eigen::Map<Eigen::VectorXd>(a_target.data(), n);
  Eigen::VectorXd fn = cod.solve(aVec);

  std::vector<double> contactNormalFce(fn.data(), fn.data() + n);

  // ---- Step 6: 迭代处理负法向力，去掉对应摩擦项重解 ----
  // 因为 invM 有接触耦合，去掉一列后重新求解可能导致之前为正的 fn 变负，
  // 需要循环直到没有新的负法向力出现。
  bool hasNegative = true;
  int maxPasses = static_cast<int>(n) + 1;  // 最多 n+1 轮，防止死循环
  for (int pass = 0; pass < maxPasses && hasNegative; ++pass) {
    hasNegative = false;
    for (sire::Size i{0}; i < n; ++i) {
      if (contactNormalFce[i] < 0) {
        hasNegative = true;
        // 将负法向力对应的摩擦方向整列清零
        D[3 * i * n + i] = 0;
        D[(3 * i + 1) * n + i] = 0;
        D[(3 * i + 2) * n + i] = 0;
      }
    }
    if (hasNegative) {
      // 重建 invMD 并重解
      aris::dynamic::s_mm(n, n, 3 * n, invM.data(), D.data(), invMD.data());
      invMDMat = Eigen::Map<MatrixXdRM>(invMD.data(), n, n);
      Eigen::CompleteOrthogonalDecomposition<MatrixXdRM> codPass(invMDMat);
      fn = codPass.solve(aVec);
      contactNormalFce.assign(fn.data(), fn.data() + n);
    }
  }

  // ---- Step 7: 映射到 3D 接触力 contactFce = D * fn ----
  std::fill(contactFce.begin(), contactFce.end(), 0.0);
  aris::dynamic::s_mm(3 * n, 1, n, D.data(), contactNormalFce.data(),
                      contactFce.data());

  // 确保法向力非负，零或负时整组力归零（避免法向为0但切向非零的物理矛盾）
  for (sire::Size i{0}; i < n; ++i) {
    if (contactFce[3 * i + 2] <= 0) {
      contactFce[3 * i + 0] = 0;
      contactFce[3 * i + 1] = 0;
      contactFce[3 * i + 2] = 0;
    }
  }

  return 0.0;  // 无迭代，直接解析求解
}

/// @brief 最大耗散原理版本（v3）：摩擦方向与终点切向速度迭代自洽。
/// 公式推导见文档：
///   D(d) = [d_x*μ, d_y*μ, 1]^T  (每个接触点一列)
///   H(d) = M^{-1}_{n,:} · D(d)
///   f_n = H(d)^{-1} · a_target
///   f = D(d) · f_n
///   v' = v0 + h·(M^{-1}·f + b)
///   d^{new} = -normalize(v'_t)
/// 迭代至 d 收敛。
auto cptContactForceWithTargetState3(
    sire::Size n, std::vector<double>& fri_coef, std::vector<double>& invM_3n,
    std::vector<double>& v0, std::vector<double>& v_target,
    std::vector<double>& b, double h, std::vector<double>& contactFce,
    sire::Size max_iters, double max_err) -> double {
  SIRE_PROFILE_FUNCTION();

  // ---- Step 1: 构建 n×3n 的法向逆惯量矩阵 (提取 z 行) ----
  std::vector<double> invM_z(n * 3 * n);
  for (sire::Size i{0}; i < n; ++i)
    for (sire::Size j{0}; j < n; ++j)
      for (sire::Size l{0}; l < 3; ++l)
        invM_z[3 * n * i + 3 * j + l] =
            invM_3n[9 * n * i + 3 * n * 2 + 3 * j + l];

  // ---- Step 2: 法向目标加速度（与摩擦方向无关，只算一次） ----
  std::vector<double> a_target(n);
  for (sire::Size i{0}; i < n; ++i) {
    double dv = v_target[i] + v0[3 * i + 2];
    a_target[i] = dv / h - b[3 * i + 2];
  }
  DLOG(DEBUG) << "v3 a_target: " << a_target;

  // ---- Step 3: 初始化摩擦方向 d_i = -normalize(v_{t,i}(0)) ----
  std::vector<double> fric_dir(2 * n);  // [dx0, dy0, dx1, dy1, ...]
  for (sire::Size i{0}; i < n; ++i) {
    double vtx = v0[3 * i + 0];
    double vty = v0[3 * i + 1];
    double vt = std::sqrt(vtx * vtx + vty * vty);
    double vt_reg = std::max(vt, 1e-8);
    fric_dir[2 * i + 0] = -(vtx / vt_reg);
    fric_dir[2 * i + 1] = -(vty / vt_reg);
  }

  using MatrixXdRM = Eigen::Matrix<double, -1, -1, Eigen::RowMajor>;
  std::vector<double> D(3 * n * n, 0);
  std::vector<double> invMD(n * n, 0);
  std::vector<double> contactNormalFce(n);
  std::vector<double> v_final(3 * n);
  std::vector<double> invM_f(3 * n);

  double error = -1.0;
  int outer_used = 0;
  for (outer_used = 0; outer_used < max_iters; ++outer_used) {
    // ---- 构建 D(d)，摩擦列 = [d_x·μ, d_y·μ, 1]^T ----
    std::fill(D.begin(), D.end(), 0.0);
    for (sire::Size i{0}; i < n; ++i) {
      D[3 * i * n + i] = fric_dir[2 * i + 0] * fri_coef[i];
      D[(3 * i + 1) * n + i] = fric_dir[2 * i + 1] * fri_coef[i];
      D[(3 * i + 2) * n + i] = 1.0;
    }

    // ---- H = invM_z * D, COD 求解 f_n ----
    aris::dynamic::s_mm(n, n, 3 * n, invM_z.data(), D.data(), invMD.data());
    MatrixXdRM invMDMat = Eigen::Map<MatrixXdRM>(invMD.data(), n, n);
    Eigen::CompleteOrthogonalDecomposition<MatrixXdRM> cod(invMDMat);
    Eigen::VectorXd aVec = Eigen::Map<Eigen::VectorXd>(a_target.data(), n);
    Eigen::VectorXd fn = cod.solve(aVec);
    contactNormalFce.assign(fn.data(), fn.data() + n);

    // ---- 处理负法向力：迭代清零对应列 ----
    bool hasNegative = true;
    int maxPasses = static_cast<int>(n) + 1;
    for (int pass = 0; pass < maxPasses && hasNegative; ++pass) {
      hasNegative = false;
      for (sire::Size i{0}; i < n; ++i) {
        if (contactNormalFce[i] < 0) {
          hasNegative = true;
          D[3 * i * n + i] = 0;
          D[(3 * i + 1) * n + i] = 0;
          D[(3 * i + 2) * n + i] = 0;
        }
      }
      if (hasNegative) {
        aris::dynamic::s_mm(n, n, 3 * n, invM_z.data(), D.data(), invMD.data());
        invMDMat = Eigen::Map<MatrixXdRM>(invMD.data(), n, n);
        Eigen::CompleteOrthogonalDecomposition<MatrixXdRM> codPass(invMDMat);
        fn = codPass.solve(aVec);
        contactNormalFce.assign(fn.data(), fn.data() + n);
      }
    }

    // ---- f = D * f_n ----
    std::fill(contactFce.begin(), contactFce.end(), 0.0);
    aris::dynamic::s_mm(3 * n, 1, n, D.data(), contactNormalFce.data(),
                        contactFce.data());

    // ---- v' = v0 + h·(M^{-1}·f + b) ----
    aris::dynamic::s_mm(3 * n, 1, 3 * n, invM_3n.data(), contactFce.data(),
                        invM_f.data());
    for (sire::Size i{0}; i < 3 * n; ++i)
      v_final[i] = v0[i] + h * (invM_f[i] + b[i]);

    // ---- 更新摩擦方向：d_new = -normalize(v'_t) ----
    double max_dir_change = 0.0;
    for (sire::Size i{0}; i < n; ++i) {
      if (contactNormalFce[i] <= 0) continue;  // 无接触力，保留旧方向
      double vtx = v_final[3 * i + 0];
      double vty = v_final[3 * i + 1];
      double vt2 = vtx * vtx + vty * vty;
      if (vt2 < 1e-16) continue;  // 粘滞，保留旧方向
      double vt = std::sqrt(vt2);
      double new_dx = -(vtx / vt);
      double new_dy = -(vty / vt);
      double change = std::abs(new_dx - fric_dir[2 * i + 0]) +
                      std::abs(new_dy - fric_dir[2 * i + 1]);
      max_dir_change = std::max(max_dir_change, change);
      fric_dir[2 * i + 0] = new_dx;
      fric_dir[2 * i + 1] = new_dy;
    }

    error = max_dir_change;
    DLOG(DEBUG) << "v3 iter=" << outer_used << " dir_change=" << error;
    if (error < max_err) break;
  }

  // ---- 最终 clamp ----
  for (sire::Size i{0}; i < n; ++i) {
    if (contactFce[3 * i + 2] <= 0) {
      contactFce[3 * i + 0] = 0;
      contactFce[3 * i + 1] = 0;
      contactFce[3 * i + 2] = 0;
    }
  }

  DLOG(DEBUG) << "v3 outer_iters=" << outer_used << " final_error=" << error;
  return error;
}

// ============================================================================
// v4: 逐接触点 SOR + 粘/滑状态分类 + 滑移二分法
// 将全局 3n 变量 SOCP 分解为 n 个 3 变量逐接触点子问题，
// 每个子问题用粘滞/滑移显式分类 + 极坐标二分法求解。
// 对标 RaiSim 论文的 per-contact bisection 结构，子问题修改了法向约束常数项。
// ============================================================================

namespace {

/// @brief 滑移单接触子问题：在圆锥曲线 C 上二分搜索最优冲量
/// @param W 3×3 逆表观惯性矩阵 M_{i,i}^{-1}
/// @param c 3 维权速度常数项 (v0_i + h·b_i + h·ΣW_ij·f_j)
/// @param v_tgt 法向目标速度
/// @param h 接触时间步长
/// @param mu 摩擦系数
/// @return 最优接触力 f (3 维)
auto solveSlipBisection(const Eigen::Matrix3d& W, const Eigen::Vector3d& c,
                        double v_tgt, double h, double mu) -> Eigen::Vector3d {
  const double W31 = W(2, 0), W32 = W(2, 1), W33 = W(2, 2);
  const double cn = c(2);
  const double numer = (v_tgt - cn) / h;

  // r(θ) = numer / (W33/μ + W31·cosθ + W32·sinθ)
  auto r_of_theta = [&](double theta) -> double {
    double denom = W33 / mu + W31 * std::cos(theta) + W32 * std::sin(theta);
    if (std::abs(denom) < 1e-30) return -1.0;
    return numer / denom;
  };

  // f(θ, r)
  auto f_of = [&](double theta, double r) -> Eigen::Vector3d {
    return {r * std::cos(theta), r * std::sin(theta), r / mu};
  };

  // 能量：E(f) = ½h·fᵀWf + cᵀf
  auto energy = [&](double theta) -> double {
    double r = r_of_theta(theta);
    if (r <= 0) return 1e100;
    Eigen::Vector3d f = f_of(theta, r);
    return 0.5 * h * f.dot(W * f) + c.dot(f);
  };

  // 数值梯度符号（沿 θ 方向）
  auto grad_sign = [&](double theta) -> int {
    double eps = 1e-10;
    double ep = energy(theta + eps);
    double em = energy(theta - eps);
    if (ep >= 1e99 || em >= 1e99) return 0;  // 无效区域
    return (ep > em) ? 1 : -1;
  };

  // ---- Step 1: 初始猜测 θ₀ ----
  // 零速度解 f_v0 = -W^{-1}·c_mod，其切向分量方向给出初始 θ
  Eigen::Vector3d c_mod = c;
  c_mod(2) -= v_tgt / h;
  Eigen::Vector3d f_v0 = -W.ldlt().solve(c_mod);
  double theta0 = std::atan2(f_v0(1), f_v0(0));

  // 验证初始 r 为正（在摩擦锥边界上可达）
  {
    double r0 = r_of_theta(theta0);
    if (r0 <= 0) {
      // 扫描找到第一个合法方向
      for (int k = 0; k < 16; ++k) {
        double test_theta = static_cast<double>(k) * 2.0 * sire::PI / 16.0;
        double test_r = r_of_theta(test_theta);
        if (test_r > 0) {
          theta0 = test_theta;
          break;
        }
      }
    }
  }
  double r0 = r_of_theta(theta0);
  if (r0 <= 0) {
    // 无可达滑移解 → 回退到零力（应由外层张开处理）
    return Eigen::Vector3d::Zero();
  }

  // ---- Step 2: 增量步进，定位包含最小值的区间 ----
  const int D0 = grad_sign(theta0);
  if (D0 == 0) {
    // 起点的梯度无效，直接返回投影解
    return f_of(theta0, r0);
  }

  double alpha = -0.15 * D0;  // 初始步长
  const double beta2 = 0.5;   // 回退因子
  const double beta3 = 2.0;   // 加速因子

  double theta_left = theta0;
  double theta_right = theta0;
  bool interval_found = false;

  for (int step = 0; step < 30 && !interval_found; ++step) {
    double theta_new = theta_right + alpha;
    double r_new = r_of_theta(theta_new);

    // 物理可行性检查：r > 0（在锥外边界上）
    if (r_new <= 0) {
      alpha *= beta2;
      continue;
    }

    int D_new = grad_sign(theta_new);
    if (D_new == 0) {
      alpha *= beta2;
      continue;
    }

    if (D_new == D0) {
      // 未跨过零点，继续步进
      theta_left = theta_new;
      alpha *= beta3;
    } else {
      theta_right = theta_new;
      interval_found = true;
    }
  }

  if (!interval_found) {
    // 未找到区间 → 返回初始投影解
    return f_of(theta0, r0);
  }

  // ---- Step 3: 二分法求精 ----
  const double tol = 1e-12;
  for (int bisect = 0; bisect < 50; ++bisect) {
    if (theta_right - theta_left < tol) break;

    double theta_mid = (theta_left + theta_right) / 2.0;
    int D_mid = grad_sign(theta_mid);

    if (D_mid == 0) break;
    if (D_mid == D0) {
      theta_left = theta_mid;
    } else {
      theta_right = theta_mid;
    }
  }

  double theta_opt = (theta_left + theta_right) / 2.0;
  double r_opt = r_of_theta(theta_opt);
  if (r_opt <= 0) r_opt = 1e-12;  // 安全下限

  return f_of(theta_opt, r_opt);
}

}  // namespace

/// @brief 逐接触点 SOR + 粘/滑二分法（v4）
/// 对标 RaiSim 的 per-contact bisection + 非线性块 SOR 结构。
/// 与 RaiSim 的区别：法向约束从"零法向速度"改为"目标法向速度(v_target)"。
///
/// 求解流程：
///   for iter in 0..max_iters:
///     for each contact i (正向/反向交替扫描):
///       c_i = v0_i + h·b_i + h·Σ_{j≠i} W_ij · f_j    ← 自由速度+耦合
///
///       if v_target_i > c_i_n:                           ← 张开
///         f_i* = 0
///       else:
///         f_stick = -W_ii^{-1} · (c_i - [0,0,v_tgt_i/h])
///         if f_stick_z ≥ 0 && ||f_stick_t|| ≤ μ·f_stick_z:  ← 粘滞
///           f_i* = f_stick
///         else:                                          ← 滑移
///           f_i* = solveSlipBisection(...)
///
///       f_i ← α·f_i* + (1-α)·f_i                       ← SOR 松弛更新
///     α ← α_min + γ·(α - α_min)                          ← 衰减
///     翻转扫描方向
///
/// @warning 此函数与 v1/v2/v3 参数接口一致，可直接替换调用。
auto cptContactForceWithTargetState4(
    sire::Size n, std::vector<double>& fri_coef, std::vector<double>& invM_3n,
    std::vector<double>& v0, std::vector<double>& v_target,
    std::vector<double>& b, double h, std::vector<double>& contactFce,
    sire::Size max_iters, double max_err) -> double {
  SIRE_PROFILE_FUNCTION();

  const int dim3 = 3 * static_cast<int>(n);
  using MatrixXdRM = Eigen::Matrix<double, -1, -1, Eigen::RowMajor>;

  Eigen::Map<MatrixXdRM> W(invM_3n.data(), dim3, dim3);
  W = -W;
  Eigen::Map<Eigen::VectorXd> v0_vec(v0.data(), dim3);
  Eigen::Map<Eigen::VectorXd> v_tgt(v_target.data(), n);
  v_tgt = -v_tgt;  // DAE → 接触坐标系约定
  Eigen::Map<Eigen::VectorXd> b_vec(b.data(), dim3);
  b_vec = -b_vec;  // DAE → 接触坐标系约定

  // 当前力估计（从零初始化；可从 warm-start 载入）
  Eigen::VectorXd f = Eigen::VectorXd::Zero(dim3);

  // SOR 超参数
  double alpha_val = 1.0;
  const double alpha_min = 0.7;
  const double gamma = 0.99;

  double error = -1.0;
  int outer_used = 0;

  for (outer_used = 0; outer_used < static_cast<int>(max_iters); ++outer_used) {
    bool forward = (outer_used % 2 == 0);  // 正/反向交替扫描
    double max_change = 0.0;

    for (int sweep = 0; sweep < static_cast<int>(n); ++sweep) {
      int i = forward ? sweep : (static_cast<int>(n) - 1 - sweep);

      // ---- 计算 c_i ----
      Eigen::Vector3d c_i =
          v0_vec.segment<3>(3 * i) + h * b_vec.segment<3>(3 * i);
      for (int j = 0; j < static_cast<int>(n); ++j) {
        if (j == i) continue;
        c_i.noalias() += h * W.block<3, 3>(3 * i, 3 * j) * f.segment<3>(3 * j);
      }

      Eigen::Matrix3d W_ii = W.block<3, 3>(3 * i, 3 * i);
      Eigen::Vector3d f_star;

      // ---- 判断接触状态 ----
      if (v_tgt(i) <= c_i(2)) {
        // 张开：无约束法向速度已超过目标 → 不需力
        f_star.setZero();
      } else {
        // 尝试粘滞解
        Eigen::Vector3d c_mod = c_i;
        c_mod(2) -= v_tgt(i);  // ✅ 只减 v_tgt
        Eigen::Vector3d f_stick = -W_ii.ldlt().solve(c_mod);
        f_stick /= h;  // ✅ 整体除以 h

        double fn = f_stick(2);
        double ft_norm = f_stick.head<2>().norm();

        if (fn > 0 && ft_norm <= fri_coef[i] * fn * (1.0 + 1e-12)) {
          // 粘滞：冲量在摩擦锥内 → 直接取闭式解
          f_star = f_stick;
        } else {
          // 滑移 → 二分法
          f_star = solveSlipBisection(W_ii, c_i, v_tgt(i), h, fri_coef[i]);
        }
      }

      // ---- SOR 松弛更新 ----
      Eigen::Vector3d f_old = f.segment<3>(3 * i);
      f.segment<3>(3 * i) = alpha_val * f_star + (1.0 - alpha_val) * f_old;

      double change = (f.segment<3>(3 * i) - f_old).norm();
      max_change = std::max(max_change, change);
    }

    // 衰减松弛因子
    alpha_val = alpha_min + gamma * (alpha_val - alpha_min);

    // Contact force grows roughly as 1/h when the same velocity correction is
    // requested over a shorter interval.  Use a relative force update so a
    // refined event step does not make the fixed absolute tolerance
    // artificially harder to reach.
    const double force_scale = std::max(1.0, f.cwiseAbs().maxCoeff());
    error = max_change / force_scale;
    SIRE_PROFILE_PLOT("ps_vs.v4.outer_iter", static_cast<double>(outer_used));
    SIRE_PROFILE_PLOT("ps_vs.v4.error", error);

    if (error < max_err) break;
  }
  std::copy(f.data(), f.data() + dim3, contactFce.data());
  for (int i = 0; i < static_cast<int>(n); ++i) {
    if (contactFce[3 * i + 2] <= 0) {
      DLOG(WARNING) << "unexpected negative contact force at contact " << i
                    << ": f = [" << contactFce[3 * i + 0] << ", "
                    << contactFce[3 * i + 1] << ", " << contactFce[3 * i + 2]
                    << "]";
    }
  }

  SIRE_PROFILE_PLOT("ps_vs.v4.outer_used", static_cast<double>(outer_used));
  DLOG(DEBUG) << "v4 outer_iters=" << outer_used << " final_error=" << error;
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
      PsVsSolver3::*CollisionFilterPoolFunc)();
  typedef sire::core::MaterialManager& (PsVsSolver3::*MaterialManagerFunc)();
  aris::core::class_<PsVsSolver3>("PsVsSolver3")
      .inherit<ContactSolver>()
      .prop("material_manager", &PsVsSolver3::resetMaterialManager,
            MaterialManagerFunc(&PsVsSolver3::materialManager))
      .prop("default_k", &PsVsSolver3::setDefaultStiffness,
            &PsVsSolver3::defaultStiffness)
      .prop("default_cr", &PsVsSolver3::setDefaultCr, &PsVsSolver3::defaultCr);
}
}  // namespace sire::physics::contact::ps_vs_solver3
