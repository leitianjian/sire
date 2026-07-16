#include "sire/physics/contact/analytical_implicit_friction_solver.hpp"

#include <array>
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>
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
#include "sire/core/profiler.hpp"
#include "sire/core/prop_map.hpp"
#include "sire/physics/collision/collision_detection.hpp"
#include "sire/physics/geometry/collidable_geometry.hpp"
#include "sire/physics/physics_engine.hpp"
#include "sire/physics/utils.hpp"
#include "sire/simulator/simulation_loop.hpp"

// Reuse DAE helpers from analytical_tangent_force namespace
#include "sire/physics/contact/analytical_tangent_force_solver.hpp"

#include "log/easyloggingConfig.hpp"

namespace sire::physics::contact::analytical_implicit_friction {

using MatrixXdRM = Eigen::Matrix<double, -1, -1, Eigen::RowMajor>;

// =========================================================================
//  Imp struct
// =========================================================================
struct AnalyticalImplicitFrictionSolver::Imp {
  std::unique_ptr<core::MaterialManager> material_manager_;
  nlohmann::json records;
  double default_cr_;
  double default_cof_;
  double default_tv_;
  double default_k_;
  double default_d_;
  std::vector<common::PenetrationAsPointPair> contactEnded;
  std::vector<common::PenetrationAsPointPair> contactNotEnd;
  std::vector<double> contactNotEndCondition;
  double prevStiffScale;

  Imp()
      : material_manager_(std::make_unique<core::MaterialManager>()),
        default_cr_(0.2),
        default_k_(2.8e8),
        default_cof_(0.3),
        default_tv_(0.1),
        prevStiffScale(1) {}
};

// =========================================================================
//  Constructor / Destructor / Move
// =========================================================================
AnalyticalImplicitFrictionSolver::AnalyticalImplicitFrictionSolver()
    : imp_(std::make_unique<Imp>()) {}
AnalyticalImplicitFrictionSolver::~AnalyticalImplicitFrictionSolver() {}
SIRE_DEFINE_MOVE_CTOR_CPP(AnalyticalImplicitFrictionSolver);

auto AnalyticalImplicitFrictionSolver::resetMaterialManager(
    core::MaterialManager* manager) -> void {
  imp_->material_manager_.reset(manager);
}
auto AnalyticalImplicitFrictionSolver::materialManager()
    -> core::MaterialManager& {
  return *imp_->material_manager_;
}
auto AnalyticalImplicitFrictionSolver::setDefaultStiffness(double k) noexcept
    -> void {
  imp_->default_k_ = k;
}
auto AnalyticalImplicitFrictionSolver::defaultStiffness() noexcept -> double {
  return imp_->default_k_;
}
auto AnalyticalImplicitFrictionSolver::setDefaultCr(double cr) noexcept
    -> void {
  imp_->default_cr_ = cr;
}
auto AnalyticalImplicitFrictionSolver::defaultCr() noexcept -> double {
  return imp_->default_cr_;
}
auto AnalyticalImplicitFrictionSolver::setDefaultVelocityThreshold(
    double tv) noexcept -> void {
  imp_->default_tv_ = tv;
}
auto AnalyticalImplicitFrictionSolver::defaultVelocityThreshold() noexcept
    -> double {
  return imp_->default_tv_;
}
auto AnalyticalImplicitFrictionSolver::debugByRecords() -> nlohmann::json {
  return imp_->records;
}

// =========================================================================
//  Local helper: cptGlbContactWrench (not exposed in analytical_tangent_force header)
// =========================================================================

inline auto cptGlbContactWrench(
    aris::dynamic::Model& model, sire::physics::PhysicsEngine& engine,
    const std::vector<double>& contactFce,
    const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    const std::vector<std::array<double, 16>>& T_C_vec,
    const std::vector<sire::Size>& preservedPairsIdx,
    const std::vector<geometry::CollidableGeometry*>& geomPtrVector) -> void {
  engine.resetPartContactForce();
  const sire::Size contact_force_offset = engine.contactForceIdx();
  auto& force_pool = model.forcePool();
  sire::Size n{preservedPairsIdx.size()};
  for (int i = 0; i < n; ++i) {
    double f_Bc_C[3]{contactFce[3 * i], contactFce[3 * i + 1],
                     contactFce[3 * i + 2]};
    double fs[6];
    core::screw::s_fpm2fs(f_Bc_C, T_C_vec.at(preservedPairsIdx[i]).data(), fs);
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
/// @brief  Solve 2×2 constrained QP: min 0.5*ft^T*P*ft + q^T*ft  s.t. ||ft|| <= c
/// Uses eigendecomposition + 1D Newton for the Lagrange multiplier.
inline Eigen::Vector2d solveLocalFrictionQP(const Eigen::Matrix2d& P,
                                              const Eigen::Vector2d& q,
                                              double c) {
  // Unconstrained optimum
  Eigen::Vector2d ft_unc = -P.inverse() * q;
  double norm_unc = ft_unc.norm();
  if (norm_unc <= c + 1e-12) return ft_unc;

  // Constraint active: find λ > 0 s.t. ||(P + λI)^{-1} * q|| = c
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(P);
  double d0 = eig.eigenvalues()(0);  // smaller eigenvalue
  double d1 = eig.eigenvalues()(1);
  // w = V^T * q  (q projected onto eigenvector basis)
  Eigen::Vector2d w = eig.eigenvectors().transpose() * q;

  double lambda = std::max(0.0, q.norm() / c - d0);
  for (int k = 0; k < 20; ++k) {
    double inv0 = 1.0 / (d0 + lambda);
    double inv1 = 1.0 / (d1 + lambda);
    double f_val = w(0) * w(0) * inv0 * inv0 + w(1) * w(1) * inv1 * inv1 - c * c;
    double f_der = -2.0 * (w(0) * w(0) * inv0 * inv0 * inv0 +
                           w(1) * w(1) * inv1 * inv1 * inv1);
    if (std::abs(f_val) < 1e-10 * c * c) break;
    double step = f_val / f_der;
    lambda -= step;
    if (lambda < -0.9 * d0) lambda = -0.9 * d0;
    if (lambda < 0.0) lambda = 0.0;
    if (std::abs(step) < 1e-12) break;
  }

  double inv0 = 1.0 / (d0 + lambda);
  double inv1 = 1.0 / (d1 + lambda);
  // ft = -V * diag(1/(d+λ)) * V^T * q = -V * diag(1/(d+λ)) * w
  Eigen::Vector2d w_scaled(w(0) * inv0, w(1) * inv1);
  return -eig.eigenvectors() * w_scaled;
}

// =========================================================================
//  Main solver entry point
// =========================================================================
auto AnalyticalImplicitFrictionSolver::cptContactSolverResult(
    const aris::dynamic::Model* current_state,
    std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    std::vector<std::array<double, 16>>& T_C_vec, ContactSolverResult& result)
    -> void {
  SIRE_PROFILE_FUNCTION();
  auto enginePtr = physicsEnginePtr();
  SIRE_ASSERT(enginePtr != nullptr);
  auto modelPtr = enginePtr->currentModel();
  SIRE_ASSERT(modelPtr != nullptr);
  auto& partPool = modelPtr->partPool();
  double nextCtrlSimSuggestDt = result.dt;

  // ========== Filter contacts & compute contact frames ==========
  cptContactFrame(penetration_pairs, T_C_vec);
  std::vector<sire::Size> preservedPairsIdx;
  for (sire::Size i{0}; i < penetration_pairs.size(); ++i) {
    if (penetration_pairs[i].modifiedDepth >= 0)
      preservedPairsIdx.push_back(i);
  }
  sire::Size n{preservedPairsIdx.size()};
  sire::Size n2{2 * n};
  SIRE_PROFILE_PLOT("impl_fric.n_contacts", static_cast<double>(n));

  // Build geomPtrVector
  std::vector<geometry::CollidableGeometry*> geomPtrVector(n2, nullptr);
  for (sire::Size i{0}; i < n; ++i) {
    auto& pair = penetration_pairs[preservedPairsIdx[i]];
    geomPtrVector[2 * i] = enginePtr->queryGeometryPoolById(pair.id_A);
    geomPtrVector[2 * i + 1] = enginePtr->queryGeometryPoolById(pair.id_B);
    SIRE_DEMAND(geomPtrVector[2 * i] != nullptr);
    SIRE_DEMAND(geomPtrVector[2 * i + 1] != nullptr);
  }
  std::vector<sire::PartId> prtIdVector(n2);
  for (sire::Size i{0}; i < n2; ++i)
    prtIdVector[i] = geomPtrVector[i]->partId();

  // ---- n==0 early return ----
  if (n == 0) {
    imp_->contactNotEnd.clear();
    imp_->contactEnded.clear();
    imp_->contactNotEndCondition.clear();
    sire::simulator::SimulationLoop* simulator_ptr = enginePtr->simLoopPtr();
    simulator_ptr->recorder().recordModelState(*modelPtr);
    std::unique_ptr<core::EventBase> eventPtr{nullptr};
    if (nextCtrlSimSuggestDt - result.dt > 1e-6) {
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
    simulator_ptr->eventManager().updateCtrlSimTime(eventPtr->eventId(),
                                                    currentTime);
    simulator_ptr->eventManager().addEvent(std::move(eventPtr));
    return;
  }

  // ---- Resize result ----
  result.resize(partPool.size() * 6, penetration_pairs.size());
  {
    std::vector<bool> isPreserved(penetration_pairs.size(), false);
    for (sire::Size i{0}; i < n; ++i)
      isPreserved[preservedPairsIdx[i]] = true;
    sire::Size preservedCnt{0};
    for (sire::Size i{0}; i < penetration_pairs.size(); ++i) {
      auto& pair = penetration_pairs[i];
      if (isPreserved[i]) {
        sire::PartId idA = prtIdVector[2 * preservedCnt];
        sire::PartId idB = prtIdVector[2 * preservedCnt + 1];
        ++preservedCnt;
        result.prtsA[i] = idA;
        result.prtsB[i] = idB;
        result.contactPairIdxMap_.insert(
            {sire::core::SortedPair<sire::PartId>(idA, idB), i});
      } else {
        auto* gA = enginePtr->queryGeometryPoolById(pair.id_A);
        auto* gB = enginePtr->queryGeometryPoolById(pair.id_B);
        result.prtsA[i] = gA->partId();
        result.prtsB[i] = gB->partId();
        result.contactPairIdxMap_.insert(
            {sire::core::SortedPair<sire::PartId>(gA->partId(), gB->partId()),
             i});
      }
    }
  }

  // ---- Full 6n×6n inverse inertia & external acceleration ----
  std::vector<double> allInvCpiResult, allAccelExt6n;
  {
    SIRE_PROFILE_SCOPE("impl_fric/cptContactInverseInertiaMatrix");
    std::vector<int> prtIdVector2(n2, 0);
    for (sire::Size i{0}; i < n2; ++i)
      prtIdVector2[i] = static_cast<int>(prtIdVector[i]);
    std::vector<double> T_c(16 * n, 0), contactPoint(3 * n, 0);
    for (sire::Size i{0}; i < n; ++i) {
      std::copy(T_C_vec[preservedPairsIdx[i]].begin(),
                T_C_vec[preservedPairsIdx[i]].end(), T_c.begin() + 16 * i);
      std::copy(penetration_pairs[preservedPairsIdx[i]].p_WC.begin(),
                penetration_pairs[preservedPairsIdx[i]].p_WC.end(),
                contactPoint.begin() + 3 * i);
    }
    enginePtr->activateContactForce(false);
    dynamic_cast<aris::dynamic::ForwardDynamicSolver&>(
        modelPtr->solverPool()[3])
        .cptContactInverseInertiaMatrix(n, prtIdVector2.data(), T_c.data(),
                                        contactPoint.data(), allInvCpiResult,
                                        allAccelExt6n);
    enginePtr->activateContactForce(true);
  }

  // ---- Extract normal (z) invCpi & accelExt for DAE ----
  std::vector<double> invCpi(n2 * n2, 0);
  std::vector<double> accelExt(n2, 0);
  {
    for (sire::Size i{0}; i < n; ++i) {
      for (sire::Size j{0}; j < n; ++j) {
        for (sire::Size i2{0}; i2 < 2; ++i2) {
          for (sire::Size j2{0}; j2 < 2; ++j2) {
            invCpi[2 * n * (2 * i + i2) + 2 * j + j2] =
                allInvCpiResult[6 * n * (6 * i + 3 * i2 + 2) + 6 * j +
                                3 * j2 + 2];
          }
        }
      }
      for (sire::Size i2{0}; i2 < 2; ++i2)
        accelExt[2 * i + i2] = allAccelExt6n[6 * i + 3 * i2 + 2];
    }
  }

  // ---- Prepare DAE initial conditions ----
  std::vector<double> stiffness(n), damping(n), x0(n2), v0(3 * n);
  double stiffScale;
  std::vector<double> realX0;
  {
    stiffScale = analytical_tangent_force::cptInitialCondition(
        *enginePtr, *(imp_->material_manager_), penetration_pairs, T_C_vec,
        preservedPairsIdx, geomPtrVector.data(), stiffness.data(),
        damping.data(), x0.data(), v0.data());
    realX0.assign(x0.begin(), x0.end());
    // TODO: contact state continuation (pairsNeedModifiedIdx logic)
    //       skipped for initial test — single contact pair doesn't need it.
    imp_->prevStiffScale = stiffScale;
    imp_->contactNotEnd.clear(); imp_->contactEnded.clear();
    imp_->contactNotEndCondition.clear();
  }

  // ---- DAE: build A, b, find minTime, compute x1t ----
  std::vector<double> A(n2 * n2), b(n2);
  analytical_tangent_force::cptDAECoeff(*enginePtr, n, stiffness.data(),
                                         damping.data(), stiffScale,
                                         accelExt.data(), invCpi.data(),
                                         A.data(), b.data());

  double minTime = analytical_tangent_force::findMinRootSchur(
      n, result.dt, A.data(), b.data(), x0.data(), 1e-10, 200);

  imp_->records["currentTime"].push_back(modelPtr->time());
  imp_->records["minTime"].push_back(minTime);

  if (minTime <= 0) {
    minTime = result.dt;
  } else {
    if (minTime > result.dt)
      minTime = result.dt;
    else
      result.dt = minTime;
  }

  // ---- Compute x1t (target state at minTime) ----
  std::vector<double> Ab((n2 + 1) * (n2 + 1), 0), x01(n2 + 1), x1t(n2 + 1);
  sire::core::screw::matrixVectorComposeBack(n2, A.data(), b.data(),
                                             Ab.data());
  std::copy(x0.data(), x0.data() + n2, x01.data());
  x01[n2] = 1;
  analytical_tangent_force::cptFormulaXComposeAb(n2 + 1, Ab.data(), minTime,
                                                  x01.data(), x1t.data());

  // Track not-ended contacts
  for (sire::Size i{0}; i < n; ++i) {
    if (x1t[i] >= 1e-10) {
      imp_->contactNotEnd.push_back(penetration_pairs[preservedPairsIdx[i]]);
      imp_->contactNotEndCondition.push_back(x1t[i]);
      imp_->contactNotEndCondition.push_back(x1t[n + i]);
    } else {
      imp_->contactEnded.push_back(penetration_pairs[preservedPairsIdx[i]]);
    }
  }

  // ---- Extract full 3n×3n inverse inertia ----
  std::vector<double> invM(9 * n * n, 0);
  for (sire::Size i{0}; i < n; ++i) {
    for (sire::Size j{0}; j < n; ++j) {
      for (sire::Size k{0}; k < 3; ++k) {
        for (sire::Size l{0}; l < 3; ++l) {
          invM[9 * n * i + 3 * n * k + 3 * j + l] =
              allInvCpiResult[6 * n * (6 * i + k) + 6 * j + l + 3] +
              allInvCpiResult[6 * n * (6 * i + k + 3) + 6 * j + l] -
              allInvCpiResult[6 * n * (6 * i + k) + 6 * j + l] -
              allInvCpiResult[6 * n * (6 * i + k + 3) + 6 * j + l + 3];
        }
      }
    }
  }

  // External acceleration (3n), B−A
  std::vector<double> accelExt3n(3 * n, 0);
  for (sire::Size i{0}; i < n; ++i)
    for (sire::Size k{0}; k < 3; ++k)
      accelExt3n[3 * i + k] =
          allAccelExt6n[6 * i + k] - allAccelExt6n[6 * i + k + 3];

  // ---- Friction coefficients ----
  std::vector<double> fri_coef(n);
  for (sire::Size i{0}; i < n; ++i) {
    const core::PropMap& pair_prop =
        imp_->material_manager_->getPropMapOrDefault(
            {geomPtrVector[2 * i]->material(),
             geomPtrVector[2 * i + 1]->material()});
    fri_coef[i] = pair_prop.getPropValueOrDefault("cof", imp_->default_cof_);
  }

  // =====================================================================
  //  Implicit Coulomb projection iteration
  //  (replaces D-matrix + COD from analytical_tangent_force)
  // =====================================================================
  MatrixXdRM WMat =
      Eigen::Map<MatrixXdRM>(invM.data(), 3 * n, 3 * n);
  Eigen::MatrixXd P = -minTime * 0.5 * (WMat + WMat.transpose());

  Eigen::VectorXd vFree(3 * n), bVec(3 * n), vTarget(n);
  for (int i = 0; i < 3 * n; ++i) vFree(i) = v0[i];
  for (int i = 0; i < 3 * n; ++i) bVec(i) = accelExt3n[i];
  for (int i = 0; i < n; ++i) vTarget(i) = x1t[n + i];  // velocity target
  vFree -= minTime * bVec;

  // Index sets
  std::vector<int> idx_n(n), idx_t(2 * n);
  for (int i = 0; i < n; ++i) {
    idx_t[2 * i]     = 3 * i + 0;
    idx_t[2 * i + 1] = 3 * i + 1;
    idx_n[i]         = 3 * i + 2;
  }

  // Sub-blocks
  int dim_n = n, dim_t = 2 * n;
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

  // Regularize P_nn (same as ps_vs_solver2)
  const double reg_eps = 1e-8 * P_nn.diagonal().cwiseAbs().maxCoeff();
  Eigen::MatrixXd P_nn_reg = P_nn;
  for (int i = 0; i < dim_n; ++i) P_nn_reg(i, i) += reg_eps;
  Eigen::LDLT<Eigen::MatrixXd> P_nn_ldlt(P_nn_reg);

  // ---- Fixed-point iteration with block Gauss-Seidel QP ----
  const int    max_iters = 30;
  const double max_err   = 1e-2;
  const int    gs_iters  = 3;  // inner Gauss-Seidel sweeps per outer iter
  Eigen::VectorXd fn_val = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd ft_val = Eigen::VectorXd::Zero(2 * n);

  for (int iter = 1; iter <= max_iters; ++iter) {
    // --- Solve for normal force (tangent coupling via P_nt * ft) ---
    Eigen::VectorXd rhs = -vTarget - q_n - P_nt * ft_val;
    Eigen::VectorXd fn_new = P_nn_ldlt.solve(rhs);
    fn_new = fn_new.cwiseMax(0.0);  // unilateral constraint

    // --- Update q_tt = P_nt^T * fn + q_t ---
    Eigen::VectorXd q_tt = P_nt.transpose() * fn_new + q_t;

    // --- Block Gauss-Seidel: solve coupled QP for ft ---
    Eigen::VectorXd ft_new = ft_val;  // warm start
    for (int gs = 0; gs < gs_iters; ++gs) {
      for (int i = 0; i < n; ++i) {
        // Extract P_ii (2×2 diagonal block of P_tt)
        Eigen::Matrix2d P_ii;
        P_ii(0, 0) = P_tt(2 * i, 2 * i);
        P_ii(0, 1) = P_tt(2 * i, 2 * i + 1);
        P_ii(1, 0) = P_tt(2 * i + 1, 2 * i);
        P_ii(1, 1) = P_tt(2 * i + 1, 2 * i + 1);

        // Compute q_eff_i = q_tt_i + sum_{j≠i} P_ij * ft_j
        Eigen::Vector2d q_eff;
        q_eff(0) = q_tt(2 * i);
        q_eff(1) = q_tt(2 * i + 1);
        for (int j = 0; j < n; ++j) {
          if (j == i) continue;
          q_eff(0) += P_tt(2 * i, 2 * j) * ft_new(2 * j) +
                      P_tt(2 * i, 2 * j + 1) * ft_new(2 * j + 1);
          q_eff(1) += P_tt(2 * i + 1, 2 * j) * ft_new(2 * j) +
                      P_tt(2 * i + 1, 2 * j + 1) * ft_new(2 * j + 1);
        }

        double c_i = fri_coef[i] * std::max(fn_new(i), 0.0);
        Eigen::Vector2d ft_i =
            solveLocalFrictionQP(P_ii, q_eff, c_i);
        ft_new(2 * i)     = ft_i(0);
        ft_new(2 * i + 1) = ft_i(1);
      }
    }

    double err = (fn_new - fn_val).norm() + (ft_new - ft_val).norm();
    fn_val = fn_new;
    ft_val = ft_new;
    if (err < max_err) break;
  }

  // ---- Assemble contact force vector ----
  std::vector<double> contactFce(3 * n, 0);
  for (int i = 0; i < n; ++i) {
    contactFce[3 * i]     = ft_val(2 * i);
    contactFce[3 * i + 1] = ft_val(2 * i + 1);
    contactFce[3 * i + 2] = fn_val(i);
  }

  // =====================================================================
  //  Apply forces, integrate, manage events (same as analytical_tangent_force)
  // =====================================================================
  {
    SIRE_PROFILE_SCOPE("impl_fric/eventIntegration");
    sire::simulator::SimulationLoop* simulator_ptr = enginePtr->simLoopPtr();
    SIRE_ASSERT(simulator_ptr != nullptr);

    cptGlbContactWrench(
        *modelPtr, *enginePtr, contactFce, penetration_pairs, T_C_vec,
        preservedPairsIdx, geomPtrVector);

    simulator_ptr->recorder().recordModelState(*modelPtr);
    simulator_ptr->recorder().recordPenetrationPairs(penetration_pairs);

    std::unique_ptr<core::EventBase> eventPtr{nullptr};
    DLOG(DEBUG) << "nextCtrlSimSuggestDt: " << nextCtrlSimSuggestDt
                << " suggestDt: " << result.dt;
    if (nextCtrlSimSuggestDt - result.dt > 1e-6) {
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
    simulator_ptr->eventManager().updateCtrlSimTime(eventPtr->eventId(),
                                                    currentTime);
    simulator_ptr->model()->setTime(currentTime);
    simulator_ptr->eventManager().addEvent(std::move(eventPtr));
  }
}

// =========================================================================
//  ARIS Registration
// =========================================================================
ARIS_REGISTRATION {
  typedef sire::physics::collision::CollisionFilter& (
      AnalyticalImplicitFrictionSolver::*CollisionFilterPoolFunc)();
  typedef sire::core::MaterialManager& (
      AnalyticalImplicitFrictionSolver::*MaterialManagerFunc)();
  aris::core::class_<AnalyticalImplicitFrictionSolver>(
      "AnalyticalImplicitFrictionSolver")
      .inherit<ContactSolver>()
      .prop("material_manager",
            &AnalyticalImplicitFrictionSolver::resetMaterialManager,
            MaterialManagerFunc(
                &AnalyticalImplicitFrictionSolver::materialManager))
      .prop("default_k",
            &AnalyticalImplicitFrictionSolver::setDefaultStiffness,
            &AnalyticalImplicitFrictionSolver::defaultStiffness)
      .prop("default_cr", &AnalyticalImplicitFrictionSolver::setDefaultCr,
            &AnalyticalImplicitFrictionSolver::defaultCr);
}
}  // namespace sire::physics::contact::analytical_implicit_friction
