#include "sire/physics/contact/ps_vs_solver_v5.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <thread>
#include <typeinfo>
#include <vector>

#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/Dense>

#include <aris/core/reflection.hpp>
#include <aris/dynamic/math_matrix.hpp>
#include <aris/dynamic/model.hpp>
#include <aris/dynamic/model_force.hpp>
#include <aris/dynamic/model_interaction.hpp>
#include <aris/server/control_server.hpp>

#include "log/easyloggingConfig.hpp"
#include "sire/core/constants.hpp"
#include "sire/core/material_manager.hpp"
#include "sire/core/profiler.hpp"
#include "sire/core/force_screw.hpp"
#include "sire/physics/collision/collision_detection.hpp"
#include "sire/physics/geometry/collidable_geometry.hpp"
#include "sire/physics/physics_engine.hpp"
#include "sire/simulator/simulation_loop.hpp"

// Reuse preprocessing functions from v3
#include "sire/physics/contact/ps_vs_solver3.hpp"

// Forward-declare v3 internals defined in ps_vs_solver3.cpp but missing from header:
namespace sire::physics::contact::ps_vs_solver3 {
auto preprocessContactInfo(
    sire::physics::PhysicsEngine& engine,
    const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    const std::vector<sire::Size>& preservedPairsIdx,
    geometry::CollidableGeometry** geometryPtrVector) -> void;
auto cptAllAccelExtVector(
    aris::dynamic::Model& model,
    const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    const std::vector<std::array<double, 16>>& T_C_vec,
    const std::vector<sire::Size>& preservedPairsIdx,
    const sire::PartId* prtIdVector, double* accelExt) -> void;
auto modifyPenetrationDepth(
    sire::physics::PhysicsEngine& engine,
    std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    std::vector<sire::Size>& preservedPairsIdx) -> void;
// v3 cpp defines 12-param; header only has 11-param:
auto cptInitialCondition(
    sire::physics::PhysicsEngine& engine, sire::core::MaterialManager& manager,
    const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    const std::vector<std::array<double, 16>>& T_C_vec,
    const std::vector<sire::Size>& preservedPairsIdx,
    const std::vector<geometry::CollidableGeometry*>& geomPtrVector,
    double* stiffness, double* damping, double* fri_coef, double* x0,
    double* realDepthX0, double* v0) -> double;
}  // namespace sire::physics::contact::ps_vs_solver3

// TODO: temp debug logging
#undef DLOG
#define DLOG(level) LOG(INFO)

namespace sire::physics::contact::ps_vs_solver_v5 {

using MatrixXdRM = Eigen::Matrix<double, -1, -1, Eigen::RowMajor>;

// ============================================================================
// Internal ADMM helpers
// ============================================================================

namespace {

/// @brief Project a 3D vector onto the Coulomb friction cone Kμ.
/// Kμ = {(t, n) : ‖t‖ ≤ μ·n, n ≥ 0}.
/// Closed-form: for ‖v_T‖ > μ·v_n and v_n ≥ －‖v_T‖/μ,
/// β = (μ·‖v_T‖ + v_n) / (1 + μ²),  t = μ·β·v_T/‖v_T‖,  n = β.
auto projCone3(const Eigen::Vector3d& v, double mu) -> Eigen::Vector3d {
  double vn = v(2);
  double vt_norm = v.head<2>().norm();

  // Already in cone
  if (vn >= 0.0 && vt_norm <= mu * vn) return v;

  // In dual cone → origin
  if (vt_norm <= -vn / mu) return Eigen::Vector3d::Zero();

  // Project to lateral surface of Kμ
  double beta = (mu * vt_norm + vn) / (1.0 + mu * mu);
  if (beta <= 0.0) return Eigen::Vector3d::Zero();

  Eigen::Vector3d result;
  result.head<2>() = (mu * beta / vt_norm) * v.head<2>();
  result(2) = beta;
  return result;
}

auto projConeAll(const Eigen::VectorXd& f_vec, int nc,
                 const std::vector<double>& mu_vec) -> Eigen::VectorXd {
  Eigen::VectorXd y(f_vec.size());
  for (int i = 0; i < nc; ++i)
    y.segment<3>(3 * i) = projCone3(f_vec.segment<3>(3 * i), mu_vec[i]);
  return y;
}

/// @brief Γ_new(σ) = [0, 0, μ·‖σ_T‖ - σ_N] per contact (DAE-modified).
auto computeDeSaxceCorrection(const Eigen::VectorXd& sigma, int nc,
                              const std::vector<double>& mu_vec)
    -> Eigen::VectorXd {
  Eigen::VectorXd gamma(3 * nc);
  gamma.setZero();
  for (int i = 0; i < nc; ++i) {
    double sigma_N = sigma(3 * i + 2);
    double sigma_T_norm = sigma.segment<2>(3 * i).norm();
    gamma(3 * i + 2) = mu_vec[i] * sigma_T_norm - sigma_N;
  }
  return gamma;
}

/// @brief Γ(σ) = [0, 0, μ·‖σ_T‖] per contact (paper-original, no -σ_N).
auto computeDeSaxceOriginal(const Eigen::VectorXd& sigma, int nc,
                            const std::vector<double>& mu_vec)
    -> Eigen::VectorXd {
  Eigen::VectorXd gamma(3 * nc);
  gamma.setZero();
  for (int i = 0; i < nc; ++i) {
    gamma(3 * i + 2) = mu_vec[i] * sigma.segment<2>(3 * i).norm();
  }
  return gamma;
}

}  // namespace

// ============================================================================
// Standalone ADMM solver (called from cptContactForces)
// ============================================================================

auto cptContactForceWithTargetState5(
    sire::Size n, std::vector<double>& fri_coef,
    std::vector<double>& invM_3n, std::vector<double>& v0,
    std::vector<double>& v_target, std::vector<double>& b, double h,
    std::vector<double>& contactFce, sire::Size max_iters,
    double max_err) -> double {
  SIRE_PROFILE_FUNCTION();

  const int nc = static_cast<int>(n);
  const int dim3 = 3 * nc;
  if (nc == 0) {
    std::fill(contactFce.begin(), contactFce.end(), 0.0);
    return 0.0;
  }

  Eigen::Map<MatrixXdRM> W_map(invM_3n.data(), dim3, dim3);
  Eigen::Map<Eigen::VectorXd> v0_vec(v0.data(), dim3);
  Eigen::Map<Eigen::VectorXd> v_tgt_vec(v_target.data(), nc);
  Eigen::Map<Eigen::VectorXd> b_vec(b.data(), dim3);

  Eigen::MatrixXd H = -h * W_map;
  H = 0.5 * (H + H.transpose());

  // a_target (v1 convention)
  Eigen::VectorXd v_tgt(nc);
  for (int i = 0; i < nc; ++i) {
    double dv = v_tgt_vec(i) + v0_vec(3 * i + 2);
    v_tgt(i) = dv - h * b_vec(3 * i + 2);
  }
  Eigen::VectorXd v_pos = -v_tgt;

  // Opening: a_pos <= 0 means the solver would need to PULL to meet target.
  for (int i = 0; i < nc; ++i) {
    if (v_pos(i) <= 0.0) {
      double a_pos_i = v_pos(i);
      v_pos(i) = 0.0;
      // DLOG(DEBUG) << "v5 c" << i << " opening, a_pos=" << a_pos_i << " → force=0";
    }
  }

  // H_N: normal rows of H (nc × 3nc)
  Eigen::MatrixXd H_N(nc, dim3);
  H_N.setZero();
  for (int i = 0; i < nc; ++i) H_N.row(i) = H.row(3 * i + 2);

  // Free velocity, matching v2's vFree = v0 - h*b
  Eigen::VectorXd g = v0_vec - h * b_vec;

  // ADMM parameters
  const double eta_v5 = 1e-6;
  double rho = std::sqrt(eta_v5 * H.norm());

  // State
  Eigen::VectorXd f = Eigen::VectorXd::Zero(dim3);
  Eigen::VectorXd y = Eigen::VectorXd::Zero(dim3);
  Eigen::VectorXd z = Eigen::VectorXd::Zero(dim3);
  Eigen::VectorXd nu = Eigen::VectorXd::Zero(nc);
  Eigen::VectorXd sigma = g;
  Eigen::VectorXd s = Eigen::VectorXd::Zero(dim3);

  double error = -1.0;
  int total_iters = 0;

  for (int outer = 0; outer < static_cast<int>(max_iters); ++outer) {
    s = computeDeSaxceCorrection(sigma, nc, fri_coef);

    // Build augmented Hessian
    Eigen::MatrixXd H_aug =
        H + (eta_v5 + rho) * Eigen::MatrixXd::Identity(dim3, dim3);
    // v5 H_aug

    Eigen::LDLT<Eigen::MatrixXd> ldlt_Haug;
    ldlt_Haug.compute(H_aug);
    bool h_aug_ok = (ldlt_Haug.info() == Eigen::Success);

    Eigen::MatrixXd H_N_Hinv;
    if (h_aug_ok) H_N_Hinv = ldlt_Haug.solve(H_N.transpose());

    Eigen::MatrixXd Schur(nc, nc);
    Eigen::LDLT<Eigen::MatrixXd> ldlt_Schur;
    bool schur_ok = false;
    if (h_aug_ok) {
      Schur = H_N * H_N_Hinv;
      ldlt_Schur.compute(Schur);
      schur_ok = (ldlt_Schur.info() == Eigen::Success);
    }

    for (int inner = 0; inner < 200; ++inner) {  // v5
      ++total_iters;
      Eigen::VectorXd f_old = f;

      Eigen::VectorXd rhs = rho * y + z - (g + s) + eta_v5 * f_old;

      if (h_aug_ok && schur_ok) {
        Eigen::VectorXd w = ldlt_Haug.solve(rhs);
        nu = ldlt_Schur.solve(H_N * w - v_pos);
        f = w - H_N_Hinv * nu;  // f = H_aug⁻¹·rhs - H_aug⁻¹·H_Nᵀ·ν
      } else {
        // Regularized KKT fallback
        int kkt_dim = dim3 + nc;
        Eigen::MatrixXd KKT(kkt_dim, kkt_dim);
        KKT.setZero();
        KKT.topLeftCorner(dim3, dim3) =
            H +
            (eta_v5 + rho + 1e-6) * Eigen::MatrixXd::Identity(dim3, dim3);  // v5 fallback
        KKT.topRightCorner(dim3, nc) = H_N.transpose();
        KKT.bottomLeftCorner(nc, dim3) = H_N;

        Eigen::VectorXd kkt_rhs(kkt_dim);
        kkt_rhs.head(dim3) = rhs;
        kkt_rhs.tail(nc) = v_pos;

        Eigen::VectorXd sol = KKT.ldlt().solve(kkt_rhs);
        f = sol.head(dim3);
        nu = sol.tail(nc);
      }

      // Track y_old for correct dual residual
      Eigen::VectorXd y_old = y;

      y = projConeAll(f - z / rho, nc, fri_coef);
      z -= rho * (f - y);

      double prim_resid = (f - y).lpNorm<Eigen::Infinity>();
      double dual_resid =
          (eta_v5 * (f - f_old) + rho * (y - y_old))
              .lpNorm<Eigen::Infinity>();

      if (prim_resid < max_err && dual_resid < max_err) break;  // v5

      // Adaptive rho
      if (prim_resid > 10.0 * dual_resid) {
        rho *= 2.0;
      } else if (dual_resid > 10.0 * prim_resid) {
        rho *= 0.5;
        rho = std::max(rho, 1e-6);
      } else {
        // Refactor only when rho changed significantly
        continue;
      }
      H_aug = H + (eta_v5 + rho) * Eigen::MatrixXd::Identity(dim3, dim3);
      ldlt_Haug.compute(H_aug);
      h_aug_ok = (ldlt_Haug.info() == Eigen::Success);
      if (h_aug_ok) {  // v5 refactor
        H_N_Hinv = ldlt_Haug.solve(H_N.transpose());
        Schur = H_N * H_N_Hinv;
        ldlt_Schur.compute(Schur);
        schur_ok = (ldlt_Schur.info() == Eigen::Success);
      }
    }

    sigma = g + H * f;
    Eigen::VectorXd s_new =
        computeDeSaxceCorrection(sigma, nc, fri_coef);
    error = (s_new - s).lpNorm<Eigen::Infinity>();
    s = s_new;

    SIRE_PROFILE_PLOT("ps_vs_v5.outer", static_cast<double>(outer));
    SIRE_PROFILE_PLOT("ps_vs_v5.error", error);
    if (error < max_err) break;
  }

  Eigen::Map<Eigen::VectorXd> fce(contactFce.data(), dim3);
  fce = y;
  for (int i = 0; i < nc; ++i) {
    if (contactFce[3 * i + 2] <= 0.0)
      contactFce[3 * i + 0] = contactFce[3 * i + 1] =
          contactFce[3 * i + 2] = 0.0;
  }

  SIRE_PROFILE_PLOT("ps_vs_v5.iters", static_cast<double>(total_iters));
  return error;
}

// ============================================================================
// PsVsSolverV5 class implementation
// ============================================================================

struct PsVsSolverV5::Imp {
  std::unique_ptr<core::MaterialManager> material_manager_;
  nlohmann::json records;
  std::filesystem::path trace_dir_;
  std::mutex trace_mutex_;
  std::size_t trace_frame_{0};
  double default_cr_;
  double default_cof_;
  double default_tv_;
  double default_k_;
  double default_d_;

  Imp()
      : material_manager_(std::make_unique<core::MaterialManager>()),
        default_cr_(0.2),
        default_k_(2.8e8),
        default_cof_(0.3),
        default_tv_(0.1),
        default_d_(5e3) {
    const char* trace_root = std::getenv("SIRE_SOLVER_TRACE_DIR");
    if (trace_root == nullptr || trace_root[0] == '\0') return;

    const auto now = std::chrono::system_clock::now();
    const auto now_time = std::chrono::system_clock::to_time_t(now);
    const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
                            now.time_since_epoch()) %
                        1000;
    std::tm local_time{};
#ifdef _WIN32
    localtime_s(&local_time, &now_time);
#else
    localtime_r(&now_time, &local_time);
#endif
    std::ostringstream session_name;
    session_name << "trajectory_" << std::put_time(&local_time, "%Y%m%d_%H%M%S")
                 << '_' << std::setw(3) << std::setfill('0') << millis.count();

    std::error_code ec;
    const std::filesystem::path root(trace_root);
    std::filesystem::create_directories(root, ec);
    if (ec) {
      LOG(ERROR) << "solver trace: cannot create root directory " << root.string()
                 << ": " << ec.message();
      return;
    }

    trace_dir_ = root / session_name.str();
    for (std::size_t suffix = 1; std::filesystem::exists(trace_dir_); ++suffix) {
      trace_dir_ = root / (session_name.str() + "_" + std::to_string(suffix));
    }
    std::filesystem::create_directory(trace_dir_, ec);
    if (ec) {
      LOG(ERROR) << "solver trace: cannot create session directory "
                 << trace_dir_.string() << ": " << ec.message();
      trace_dir_.clear();
      return;
    }

    nlohmann::json manifest{{"format", "sire-contact-solver-trace-v1"},
                            {"frame_pattern", "frame_*.json"},
                            {"note", "one file per contact-solver invocation"}};
    std::ofstream manifest_file(trace_dir_ / "manifest.json");
    manifest_file << manifest.dump(2) << '\n';
    LOG(INFO) << "solver trace enabled: " << trace_dir_.string();
  }

  auto recordSolverFrame(
      double sim_time, sire::Size n, const std::vector<double>& fri_coef,
      const std::vector<double>& invM, const std::vector<double>& v0,
      const std::vector<double>& v_target, const std::vector<double>& b,
      double h, const std::vector<double>& result, double solver_error,
      sire::Size solver_max_iters, double solver_max_error,
      const char* solver_type) -> void {
    if (trace_dir_.empty()) return;

    std::lock_guard<std::mutex> lock(trace_mutex_);
    const std::size_t frame = trace_frame_++;
    nlohmann::json data{{"trace_format", "sire-contact-solver-frame-v1"},
                        {"frame", frame},
                        {"sim_time", sim_time},
                        {"solver_type", solver_type},
                        {"solver_error", solver_error},
                        {"solver_max_iters", solver_max_iters},
                        {"solver_max_error", solver_max_error},
                        {"n", static_cast<int>(n)},
                        {"h", h},
                        {"fri_coef", fri_coef},
                        {"invM", invM},
                        {"v0", v0},
                        {"v_target", v_target},
                        {"b", b},
                        {"cResult", result},
                        {"cADMMResult", result}};

    std::ostringstream filename;
    filename << "frame_" << std::setw(8) << std::setfill('0') << frame
             << ".json";
    const auto final_path = trace_dir_ / filename.str();
    auto temp_path = final_path;
    temp_path += ".tmp";

    std::ofstream output(temp_path, std::ios::out | std::ios::trunc);
    if (!output) {
      LOG(ERROR) << "solver trace: cannot open " << temp_path.string();
      return;
    }
    output << data.dump(2) << '\n';
    output.close();
    if (!output) {
      LOG(ERROR) << "solver trace: failed writing " << temp_path.string();
      return;
    }

    std::error_code ec;
    std::filesystem::rename(temp_path, final_path, ec);
    if (ec) {
      LOG(ERROR) << "solver trace: cannot publish " << final_path.string()
                 << ": " << ec.message();
    } else {
      DLOG(DEBUG) << "solver trace frame=" << frame
                  << " sim_time=" << sim_time << " file="
                  << final_path.string();
    }
  }
};

PsVsSolverV5::PsVsSolverV5() : imp_(std::make_unique<Imp>()) {}
PsVsSolverV5::~PsVsSolverV5() {}
SIRE_DEFINE_MOVE_CTOR_CPP(PsVsSolverV5);

auto PsVsSolverV5::resetMaterialManager(core::MaterialManager* manager)
    -> void {
  imp_->material_manager_.reset(manager);
}
auto PsVsSolverV5::materialManager() -> core::MaterialManager& {
  return *imp_->material_manager_;
}
auto PsVsSolverV5::setDefaultStiffness(double k) noexcept -> void {
  imp_->default_k_ = k;
}
auto PsVsSolverV5::defaultStiffness() noexcept -> double {
  return imp_->default_k_;
}
auto PsVsSolverV5::setDefaultCr(double cr) noexcept -> void {
  imp_->default_cr_ = cr;
}
auto PsVsSolverV5::defaultCr() noexcept -> double {
  return imp_->default_cr_;
}
auto PsVsSolverV5::setDefaultVelocityThreshold(double tv) noexcept -> void {
  imp_->default_tv_ = tv;
}
auto PsVsSolverV5::defaultVelocityThreshold() noexcept -> double {
  return imp_->default_tv_;
}
auto PsVsSolverV5::debugByRecords() -> nlohmann::json {
  return imp_->records;
}

auto PsVsSolverV5::cptContactSolverResult(
    const aris::dynamic::Model* current_state,
    std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    std::vector<std::array<double, 16>>& T_C_vec,
    ContactSolverResult& result) -> void {
  SIRE_PROFILE_FUNCTION();
  double suggest_dt = result.dt;
  auto enginePtr = physicsEnginePtr();
  SIRE_ASSERT(enginePtr != nullptr);
  auto modelPtr = enginePtr->currentModel();
  SIRE_ASSERT(modelPtr != nullptr);

  using namespace ::sire::physics::contact::ps_vs_solver3;

  sire::Size n = penetration_pairs.size();
  double nextCtrlSimSuggestDt = result.dt;

  // ---- Filter contact pairs ----
  std::vector<sire::Size> preservedPairsIdx, pairsNeedModifiedIdx, targetConditionIdx;
  filterPairsAndPreprocessInfo(*enginePtr, penetration_pairs,
      std::vector<common::PenetrationAsPointPair>{},
      std::vector<common::PenetrationAsPointPair>{},
      T_C_vec, preservedPairsIdx, pairsNeedModifiedIdx, targetConditionIdx);
  modifyPenetrationDepth(*enginePtr, penetration_pairs, preservedPairsIdx);
  n = preservedPairsIdx.size();
  DLOG(DEBUG) << "v5 n_contacts=" << n << " suggest_dt=" << suggest_dt;

  // ---- Event creation (must happen before updPs; same as v3) ----
  auto simulator_ptr = enginePtr->simLoopPtr();
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

  if (n == 0) {
    simulator_ptr->recorder().recordModelState(*modelPtr);
    simulator_ptr->recorder().recordContactPairResults({});
    double dt = result.dt;
    simulator_ptr->recorder().recordDt(dt);
    simulator_ptr->integratorPoolPtr()->at(0).updPs(dt);
    double currentTime = simulator_ptr->timer().updateSimTime(dt);
    simulator_ptr->eventManager().updateCtrlSimTime(eventPtr->eventId(), currentTime);
    simulator_ptr->eventManager().addEvent(std::move(eventPtr));
    return;
  }

  // ---- Geometry & part IDs ----
  std::vector<geometry::CollidableGeometry*> geomPtr(2 * n, nullptr);
  preprocessContactInfo(*enginePtr, penetration_pairs, preservedPairsIdx, geomPtr.data());
  std::vector<sire::Size> prtIdVec(2 * n, 0);
  for (sire::Size i = 0; i < 2 * n; ++i) prtIdVec[i] = geomPtr[i]->partId();

  // ---- Initial conditions & stiffness ----
  std::vector<double> stiffness(n), damping(n), fri_coef(n);
  std::vector<double> x0(2 * n), realDepthX0(2 * n), v0(3 * n);
  double stiffScale = cptInitialCondition(*enginePtr, *imp_->material_manager_,
      penetration_pairs, T_C_vec, preservedPairsIdx, geomPtr,
      stiffness.data(), damping.data(), fri_coef.data(), x0.data(),
      realDepthX0.data(), v0.data());
  DLOG(DEBUG) << "v5 stiffScale=" << stiffScale << " x0=" << std::vector<double>(x0.begin(), x0.end())
              << " v0=" << std::vector<double>(v0.begin(), v0.end());

  // ---- Inverse inertia ----
  std::vector<double> allAccelExt(6 * n);
  cptAllAccelExtVector(*modelPtr, penetration_pairs, T_C_vec,
                       preservedPairsIdx, prtIdVec.data(), allAccelExt.data());
  std::vector<double> allInvCpi(36 * n * n, 0);
  cptInverseCpiMatrix(*modelPtr, penetration_pairs, T_C_vec,
                      preservedPairsIdx, prtIdVec.data(),
                      allAccelExt.data(), allInvCpi.data());

  // Assemble 3n×3n invM
  std::vector<double> invM2(9 * n * n, 0);
  for (sire::Size i = 0; i < n; ++i)
    for (sire::Size j = 0; j < n; ++j)
      for (sire::Size k = 0; k < 3; ++k)
        for (sire::Size l = 0; l < 3; ++l)
          invM2[9 * n * i + 3 * n * k + 3 * j + l] =
              allInvCpi[6 * n * (6 * i + k) + 6 * j + l + 3] +
              allInvCpi[6 * n * (6 * i + k + 3) + 6 * j + l] -
              allInvCpi[6 * n * (6 * i + k) + 6 * j + l] -
              allInvCpi[6 * n * (6 * i + k + 3) + 6 * j + l + 3];

  std::vector<double> accelExt2(3 * n);
  for (sire::Size i = 0; i < n; ++i)
    for (sire::Size k = 0; k < 3; ++k)
      accelExt2[3 * i + k] = allAccelExt[6 * i + k] - allAccelExt[6 * i + k + 3];

  // ---- DAE coefficients ----
  // Extract 2n×2n shrinked invCpi (normal/z components only) from allInvCpi
  std::vector<double> invCpi_dae(4 * n * n, 0);
  for (sire::Size i = 0; i < n; ++i)
    for (sire::Size j = 0; j < n; ++j)
      for (sire::Size i2 = 0; i2 < 2; ++i2)
        for (sire::Size j2 = 0; j2 < 2; ++j2)
          invCpi_dae[2 * n * (2 * i + i2) + 2 * j + j2] =
              allInvCpi[6 * n * (6 * i + 3 * i2 + 2) + 6 * j + 3 * j2 + 2];

  std::vector<double> A_mat(4 * n * n, 0), b_vec(2 * n, 0);
  cptDAECoeff(*enginePtr, static_cast<sire::Size>(n), stiffness.data(),
              damping.data(), stiffScale, accelExt2.data(), invCpi_dae.data(),
              A_mat.data(), b_vec.data());

  // double minTime = findMinRootSchur(static_cast<sire::Size>(n), suggest_dt,
  //     A_mat.data(), b_vec.data(), x0.data(), 1e-10, 50);
  double minTime = -1;
  if (minTime < 0.0) minTime = suggest_dt;

  // ---- DAE target velocity ----
  sire::Size n2 = 2 * n;
  std::vector<double> x1t(n2 + 2), Ab_mat((n2 + 1) * (n2 + 1), 0), x01(n2 + 1);
  sire::core::screw::matrixVectorComposeBack(n2, A_mat.data(), b_vec.data(), Ab_mat.data());
  std::copy(x0.data(), x0.data() + n2, x01.data());
  x01[n2] = 1;
  cptFormulaXComposeAb(static_cast<int>(n2 + 1), Ab_mat.data(), minTime, x01.data(), x1t.data());

  std::vector<double> vTargetVel(n);
  for (sire::Size i = 0; i < n; ++i) vTargetVel[i] = x1t[n + i];

  // ---- Contact-force QP solver ----
  std::vector<double> contactFce(3 * n);
  double error = solveContactForceQP(n, fri_coef, invM2, v0, vTargetVel,
                                     accelExt2, minTime, contactFce, 200,
                                     1e-8);
  imp_->recordSolverFrame(simulator_ptr->timer().simTime(), n, fri_coef, invM2,
                          v0, vTargetVel, accelExt2, minTime, contactFce, error,
                          200, 1e-8, typeid(*this).name());
  DLOG(DEBUG) << "v5 error=" << error << " minTime=" << minTime
              << " vTargetVel=" << std::vector<double>(vTargetVel.begin(), vTargetVel.end());

  // ---- Apply forces to model (v3 full pipeline tail) ----
  enginePtr->resetPartContactForce();
  auto& force_pool = modelPtr->forcePool();
  const sire::Size contact_force_offset = enginePtr->contactForceIdx();
  std::vector<sire::simulator::ContactPairResult> pairResults;
  pairResults.reserve(n);

  for (sire::Size i = 0; i < n; ++i) {
    sire::simulator::ContactPairResult r;
    r.geomIdA = geomPtr[2 * i]->geometryId();
    r.geomIdB = geomPtr[2 * i + 1]->geometryId();
    double f_Bc_C[3]{contactFce[3 * i], contactFce[3 * i + 1], contactFce[3 * i + 2]};
    double fs[6];
    core::screw::s_fpm2fs(f_Bc_C, T_C_vec[preservedPairsIdx[i]].data(), fs);
    r.force_W[0] = fs[0]; r.force_W[1] = fs[1]; r.force_W[2] = fs[2];
    r.point_W[0] = penetration_pairs[preservedPairsIdx[i]].p_WC[0];
    r.point_W[1] = penetration_pairs[preservedPairsIdx[i]].p_WC[1];
    r.point_W[2] = penetration_pairs[preservedPairsIdx[i]].p_WC[2];
    pairResults.push_back(r);

    DLOG(DEBUG) << "v5 c" << i << ": f_Bc_C=[" << f_Bc_C[0] << ", " << f_Bc_C[1] << ", " << f_Bc_C[2] << "]";

    aris::dynamic::GeneralForce& force_A =
        dynamic_cast<aris::dynamic::GeneralForce&>(
            force_pool.at(geomPtr[2 * i]->partId() + contact_force_offset));
    aris::dynamic::GeneralForce& force_B =
        dynamic_cast<aris::dynamic::GeneralForce&>(
            force_pool.at(geomPtr[2 * i + 1]->partId() + contact_force_offset));
    double fs_A[6]{0}; aris::dynamic::s_vc(6, force_A.fce(), fs_A);
    aris::dynamic::s_vs(6, fs, fs_A); force_A.setFce(fs_A);
    double fs_B[6]{0}; aris::dynamic::s_vc(6, force_B.fce(), fs_B);
    aris::dynamic::s_va(6, fs, fs_B); force_B.setFce(fs_B);
  }

  simulator_ptr->recorder().recordModelState(*modelPtr);
  simulator_ptr->recorder().recordPenetrationPairs(penetration_pairs);
  simulator_ptr->recorder().recordContactPairResults(pairResults);
  simulator_ptr->recorder().recordDt(minTime);
  simulator_ptr->integratorPoolPtr()->at(0).updPs(minTime);
  double currentTime = simulator_ptr->timer().updateSimTime(minTime);
  simulator_ptr->eventManager().updateCtrlSimTime(eventPtr->eventId(), currentTime);
  simulator_ptr->model()->setTime(currentTime);
  simulator_ptr->eventManager().addEvent(std::move(eventPtr));

  result.dt = minTime;
}

auto PsVsSolverV5::cptContactForces(
    aris::dynamic::Model& model, sire::physics::PhysicsEngine& engine,
    std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    std::vector<std::array<double, 16>>& T_C_vec,
    std::vector<common::PointPairContactInfo>& contact_info,
    double suggest_dt) -> double {
  // Deprecated: logic moved to cptContactSolverResult
  ContactSolverResult result{suggest_dt};
  cptContactSolverResult(nullptr, penetration_pairs, T_C_vec, result);
  return result.dt;
}

auto PsVsSolverV5::solveContactForceQP(
    sire::Size n, std::vector<double>& fri_coef,
    std::vector<double>& invM_3n, std::vector<double>& v0,
    std::vector<double>& v_target, std::vector<double>& b, double h,
    std::vector<double>& contactFce, sire::Size max_iters,
    double max_err) -> double {
  return cptContactForceWithTargetState6(
      n, fri_coef, invM_3n, v0, v_target, b, h, contactFce, max_iters,
      max_err);
}

ARIS_REGISTRATION {
  typedef sire::physics::collision::CollisionFilter& (
      PsVsSolverV5::*CollisionFilterPoolFunc)();
  typedef sire::core::MaterialManager& (PsVsSolverV5::*MaterialManagerFunc)();
  aris::core::class_<PsVsSolverV5>("PsVsSolverV5")
      .inherit<ContactSolver>()
      .prop("material_manager", &PsVsSolverV5::resetMaterialManager,
            MaterialManagerFunc(&PsVsSolverV5::materialManager))
      .prop("default_k", &PsVsSolverV5::setDefaultStiffness,
            &PsVsSolverV5::defaultStiffness)
      .prop("default_cr", &PsVsSolverV5::setDefaultCr, &PsVsSolverV5::defaultCr);
}

// ============================================================================
// v6: 冻结 De Saxce 修正项的嵌套 ADMM, 无 DAE v_target
//   求解 NCP: Kμ ∋ λ ⊥ (G+R)·λ + g + Γ((G+R)·λ+g) ∈ Kμ*
// ============================================================================
auto cptContactForceWithTargetState6(
    sire::Size n, std::vector<double>& fri_coef,
    std::vector<double>& invM_3n, std::vector<double>& v0,
    std::vector<double>& v_target, std::vector<double>& b, double h,
    std::vector<double>& contactFce, sire::Size max_iters,
    double max_err) -> double {
  SIRE_PROFILE_FUNCTION();

  const int nc = static_cast<int>(n);
  const int dim3 = 3 * nc;
  if (nc == 0) {
    std::fill(contactFce.begin(), contactFce.end(), 0.0);
    return 0.0;
  }

  Eigen::Map<MatrixXdRM> W_map(invM_3n.data(), dim3, dim3);
  Eigen::Map<Eigen::VectorXd> v0_vec(v0.data(), dim3);
  Eigen::Map<Eigen::VectorXd> b_vec(b.data(), dim3);

  // H = -h·W (positive semidefinite, matching v2/v5)
  Eigen::MatrixXd H = -h * W_map;
  H = 0.5 * (H + H.transpose());

  // Free velocity (matching v2's vFree = v0 - h·b)
  Eigen::VectorXd g = v0_vec - h * b_vec;

  // ADMM parameters (from paper)
  const double eta = 1e-6;
  double rho = 0.01;  // fixed for pure NCP

  // State (from paper Algorithm 1)
  Eigen::VectorXd f = Eigen::VectorXd::Zero(dim3);
  Eigen::VectorXd y = Eigen::VectorXd::Zero(dim3);
  Eigen::VectorXd z = Eigen::VectorXd::Zero(dim3);
  Eigen::VectorXd s = Eigen::VectorXd::Zero(dim3);  // Γ estimate

  double error = -1.0;
  int total_iters = 0;

  // Pre-build H_aug (will be rebuilt when rho changes)
  Eigen::MatrixXd H_aug = H + (eta + rho) * Eigen::MatrixXd::Identity(dim3, dim3);
  Eigen::LDLT<Eigen::MatrixXd> ldlt(H_aug);
  bool ldlt_ok = (ldlt.info() == Eigen::Success);

  for (int outer = 0; outer < static_cast<int>(max_iters); ++outer) {
    // Paper Alg1 line 2: s ← Γ(zk-1), using original de Saxcé correction
    s = computeDeSaxceOriginal(z, nc, fri_coef);

    for (int inner = 0; inner < 20; ++inner) {  // v6 pure NCP
      ++total_iters;
      Eigen::VectorXd f_old = f;

      // Paper Eq (37): f-update
      Eigen::VectorXd rhs = rho * y + z - (g + s) + eta * f_old;
      if (ldlt_ok) {
        f = ldlt.solve(rhs);  // Eq (37): f = M⁻¹·(-g-s+ηf⁻+ρy+z)
      } else {
        // Regularized fallback
        Eigen::MatrixXd H_reg = H + (eta + rho + 1e-6) *
                                     Eigen::MatrixXd::Identity(dim3, dim3);
        f = H_reg.ldlt().solve(rhs);
      }

      // Paper Eq (38-39): y-update (cone projection)
      Eigen::VectorXd y_old = y;
      y = projConeAll(f - z / rho, nc, fri_coef);

      // Paper Eq (36c): z-update
      z -= rho * (f - y);

      // Paper Eq (40-41): primal/dual residuals
      double prim_resid = (f - y).lpNorm<Eigen::Infinity>();
      double dual_resid =
          (eta * (f - f_old) + rho * (y - y_old)).lpNorm<Eigen::Infinity>();

      if (prim_resid < max_err && dual_resid < max_err) break;

      // Adaptive rho
      if (prim_resid > 10.0 * dual_resid) {
        rho *= 2.0;
      } else if (dual_resid > 10.0 * prim_resid) {
        rho *= 0.5;
        rho = std::max(rho, 1e-6);
      } else {
        continue;
      }
      H_aug = H + (eta + rho) * Eigen::MatrixXd::Identity(dim3, dim3);
      ldlt.compute(H_aug);
      ldlt_ok = (ldlt.info() == Eigen::Success);
    }

    // Paper Alg1 line 13: σ ← z - Γ(z), check outer convergence
    Eigen::VectorXd s_new = computeDeSaxceOriginal(z, nc, fri_coef);
    error = (s_new - s).lpNorm<Eigen::Infinity>();
    s = s_new;

    SIRE_PROFILE_PLOT("ps_vs_v6.outer", static_cast<double>(outer));
    SIRE_PROFILE_PLOT("ps_vs_v6.error", error);
    if (error < max_err) break;
  }

  // Paper Alg1 line 13: output λ ← y
  Eigen::Map<Eigen::VectorXd> fce(contactFce.data(), dim3);
  fce = y;
  for (int i = 0; i < nc; ++i) {
    if (contactFce[3 * i + 2] < 0)  // safety clamp
      contactFce[3 * i + 0] = contactFce[3 * i + 1] = contactFce[3 * i + 2] = 0.0;
  }

  SIRE_PROFILE_PLOT("ps_vs_v6.iters", static_cast<double>(total_iters));
  // DLOG(DEBUG) << "v6 iters=" << total_iters << " error=" << error;
  return error;
}

// ============================================================================
// v7: Davis-Yin 三算子分裂 (DYS), 无增广 Lagrangian
//   求解: min ½ fᵀ·H·f + fᵀ·(g+s) + I_Kμ(f) + I_{H_N·f=a_pos}(f)
//   三步: 等式投影 → 锥投影 (梯度步+反射) → 反射更新
//   参考: Davis & Yin (2017), Salim, Condat et al. (2020 JOTA)
// ============================================================================
auto cptContactForceWithTargetState7(
    sire::Size n, std::vector<double>& fri_coef,
    std::vector<double>& invM_3n, std::vector<double>& v0,
    std::vector<double>& v_target, std::vector<double>& b, double h,
    std::vector<double>& contactFce, sire::Size max_iters,
    double max_err) -> double {
  SIRE_PROFILE_FUNCTION();

  const int nc = static_cast<int>(n);
  const int dim3 = 3 * nc;
  if (nc == 0) {
    std::fill(contactFce.begin(), contactFce.end(), 0.0);
    return 0.0;
  }

  Eigen::Map<MatrixXdRM> W_map(invM_3n.data(), dim3, dim3);
  Eigen::Map<Eigen::VectorXd> v0_vec(v0.data(), dim3);
  Eigen::Map<Eigen::VectorXd> v_tgt_vec(v_target.data(), nc);
  Eigen::Map<Eigen::VectorXd> b_vec(b.data(), dim3);

  // H = -h * W (positive semidefinite)
  Eigen::MatrixXd H_mat = -h * W_map;
  H_mat = 0.5 * (H_mat + H_mat.transpose());
  Eigen::VectorXd g = v0_vec - h * b_vec;

  // a_target, a_pos
  Eigen::VectorXd a_target(nc);
  for (int i = 0; i < nc; ++i) {
    double dv = v_tgt_vec(i) + v0_vec(3*i+2);
    a_target(i) = dv - h * b_vec(3*i+2);
  }
  Eigen::VectorXd a_pos = -a_target;

  // Opening: a_pos <= 0 → solver would need to PULL → physics says open
  for (int i = 0; i < nc; ++i) {
    if (a_pos(i) <= 0.0) {
      a_pos(i) = 0.0;
      // DLOG(DEBUG) << "v7 c" << i << " opening, a_pos=" << a_pos(i) << " force=0";
    }
  }

  // H_N: normal rows (nc × dim3)
  Eigen::MatrixXd H_N(nc, dim3);
  H_N.setZero();
  for (int i = 0; i < nc; ++i) H_N.row(i) = H_mat.row(3*i+2);

  // Precompute Schur complement S = H_N·H_Nᵀ (nc × nc) for equality projection
  Eigen::MatrixXd S = H_N * H_N.transpose();
  Eigen::LDLT<Eigen::MatrixXd> ldlt_S(S);
  bool s_ok = (ldlt_S.info() == Eigen::Success);

  // DYS step size
  double L = H_mat.norm();  // Lipschitz constant of ∇F
  double gamma = 2.0 / std::max(L, 1e-6);  // optimal DYS step

  // State
  Eigen::VectorXd x = Eigen::VectorXd::Zero(dim3);  // reflected variable
  Eigen::VectorXd s = Eigen::VectorXd::Zero(dim3);   // Γ estimate
  Eigen::VectorXd sigma = g;                         // σ = H_mat·z + g

  double error = -1.0;
  int total_iters = 0;

  for (int outer = 0; outer < static_cast<int>(max_iters); ++outer) {
    // Freeze s ← Γ_new(σ)
    s = computeDeSaxceCorrection(sigma, nc, fri_coef);

    for (int inner = 0; inner < 200; ++inner) {  // v7 DYS
      ++total_iters;

      // Step 1: z = prox_{γ,H}(x) — project onto H_N·z = a_pos
      Eigen::VectorXd z = x;
      if (s_ok) {
        Eigen::VectorXd resid = H_N * x - a_pos;
        z = x - H_N.transpose() * ldlt_S.solve(resid);
      }

      // Step 2: v = 2z - x - γ·∇F(z),  y = prox_{γ,G}(v) — cone project
      Eigen::VectorXd v = 2.0 * z - x - gamma * (H_mat * z + g + s);
      Eigen::VectorXd y = projConeAll(v, nc, fri_coef);

      // Step 3: x ← x + (y - z) — reflection update
      Eigen::VectorXd x_old = x;
      x += (y - z);

      // Convergence: ‖y - z‖ → 0 means x = z = y (fixed point)
      double prim_resid = (y - z).lpNorm<Eigen::Infinity>();
      if (prim_resid < max_err) break;
    }

    // z at convergence ≈ f (contact force)
    Eigen::VectorXd z_final = x;
    if (s_ok) {
      Eigen::VectorXd resid = H_N * x - a_pos;
      z_final = x - H_N.transpose() * ldlt_S.solve(resid);
    }

    // Update σ from converged z
    sigma = g + H_mat * z_final;

    // Check Γ self-consistency
    Eigen::VectorXd s_new = computeDeSaxceCorrection(sigma, nc, fri_coef);
    error = (s_new - s).lpNorm<Eigen::Infinity>();
    s = s_new;

    SIRE_PROFILE_PLOT("ps_vs_v7.outer", static_cast<double>(outer));
    SIRE_PROFILE_PLOT("ps_vs_v7.error", error);
    if (error < max_err) break;
  }

  // Output: z at convergence = contact force
  Eigen::VectorXd z_final = x;
  if (s_ok) {
    Eigen::VectorXd resid = H_N * x - a_pos;
    z_final = x - H_N.transpose() * ldlt_S.solve(resid);
  }
  Eigen::Map<Eigen::VectorXd> fce(contactFce.data(), dim3);
  fce = z_final;
  for (int i = 0; i < nc; ++i) {
    if (contactFce[3 * i + 2] < 0)
      contactFce[3 * i + 0] = contactFce[3 * i + 1] = contactFce[3 * i + 2] = 0.0;
  }

  SIRE_PROFILE_PLOT("ps_vs_v7.iters", static_cast<double>(total_iters));
  // DLOG(DEBUG) << "v7 iters=" << total_iters << " error=" << error;
  return error;
}

// ============================================================================
// v8: PDDY — PPA (primal-dual DRS) with exact F prox (LDLT)
//    Same three-operator structure as v7, but replaces ∇F gradient step
//    with exact prox_{γ,F}(z) = (H + I/γ)⁻¹(z/γ - g - s).
//    Converges linearly (vs sublinear DRS) at cost of one (3n)×(3n) LDLT/iter.
//    Ref: Salim, Condat et al. (2020 JOTA), Primal-Dual Davis-Yin.
// ============================================================================
auto cptContactForceWithTargetState8(
    sire::Size n, std::vector<double>& fri_coef,
    std::vector<double>& invM_3n, std::vector<double>& v0,
    std::vector<double>& v_target, std::vector<double>& b, double h,
    std::vector<double>& contactFce, sire::Size max_iters,
    double max_err) -> double {
  SIRE_PROFILE_FUNCTION();

  const int nc = static_cast<int>(n);
  const int dim3 = 3 * nc;
  if (nc == 0) {
    std::fill(contactFce.begin(), contactFce.end(), 0.0);
    return 0.0;
  }

  Eigen::Map<MatrixXdRM> W_map(invM_3n.data(), dim3, dim3);
  Eigen::Map<Eigen::VectorXd> v0_vec(v0.data(), dim3);
  Eigen::Map<Eigen::VectorXd> v_tgt_vec(v_target.data(), nc);
  Eigen::Map<Eigen::VectorXd> b_vec(b.data(), dim3);

  // H = -h * W, g = v0 - h*b (same as v5/v7)
  Eigen::MatrixXd H_mat = -h * W_map;
  H_mat = 0.5 * (H_mat + H_mat.transpose());
  Eigen::VectorXd g = v0_vec - h * b_vec;

  // a_target, a_pos, opening check (same as v7)
  Eigen::VectorXd a_target(nc);
  for (int i = 0; i < nc; ++i) {
    double dv = v_tgt_vec(i) + v0_vec(3*i+2);
    a_target(i) = dv - h * b_vec(3*i+2);
  }
  Eigen::VectorXd a_pos = -a_target;
  for (int i = 0; i < nc; ++i) {
    if (a_pos(i) <= 0.0) { a_pos(i) = 0.0; }
  }

  // H_N and Schur for equality projection
  Eigen::MatrixXd H_N(nc, dim3);
  H_N.setZero();
  for (int i = 0; i < nc; ++i) H_N.row(i) = H_mat.row(3*i+2);
  Eigen::MatrixXd S = H_N * H_N.transpose();
  Eigen::LDLT<Eigen::MatrixXd> ldlt_S(S);
  bool s_ok = (ldlt_S.info() == Eigen::Success);

  // PDDY step size
  double L = H_mat.norm();
  double gamma = 1.0 / std::max(L, 1e-6);

  // LDLT for exact F prox: M_F = H + I/γ
  Eigen::MatrixXd M_F = H_mat + (1.0/gamma) * Eigen::MatrixXd::Identity(dim3, dim3);
  Eigen::LDLT<Eigen::MatrixXd> ldlt_F(M_F);
  bool f_ok = (ldlt_F.info() == Eigen::Success);

  // State
  Eigen::VectorXd x = Eigen::VectorXd::Zero(dim3);
  Eigen::VectorXd s = Eigen::VectorXd::Zero(dim3);
  Eigen::VectorXd sigma = g;

  double error = -1.0;
  int total_iters = 0;

  for (int outer = 0; outer < static_cast<int>(max_iters); ++outer) {
    s = computeDeSaxceCorrection(sigma, nc, fri_coef);

    for (int inner = 0; inner < 200; ++inner) {
      ++total_iters;

      // z = prox_H(x) — equality projection (same as v7)
      Eigen::VectorXd z = x;
      if (s_ok) {
        Eigen::VectorXd resid = H_N * x - a_pos;
        z = x - H_N.transpose() * ldlt_S.solve(resid);
      }

      // f = prox_{γ, F}(z) — exact F prox via LDLT (NEW vs v7)
      Eigen::VectorXd f_F = z;
      if (f_ok) {
        Eigen::VectorXd rhs_F = z / gamma - (g + s);
        f_F = ldlt_F.solve(rhs_F);
      }

      // y = prox_G(2f_F - x - (f_F - z)/γ) — cone projection
      Eigen::VectorXd v = 2.0 * f_F - x - (f_F - z) / gamma;
      Eigen::VectorXd y = projConeAll(v, nc, fri_coef);

      // x ← x + (y - f_F) — reflection on exact F output
      x += (y - f_F);

      double prim_resid = (y - f_F).lpNorm<Eigen::Infinity>();
      if (prim_resid < max_err) break;
    }

    Eigen::VectorXd z_final = x;
    if (s_ok) {
      Eigen::VectorXd resid = H_N * x - a_pos;
      z_final = x - H_N.transpose() * ldlt_S.solve(resid);
    }
    sigma = g + H_mat * z_final;

    Eigen::VectorXd s_new = computeDeSaxceCorrection(sigma, nc, fri_coef);
    error = (s_new - s).lpNorm<Eigen::Infinity>();
    s = s_new;

    SIRE_PROFILE_PLOT("ps_vs_v8.outer", static_cast<double>(outer));
    SIRE_PROFILE_PLOT("ps_vs_v8.error", error);
    if (error < max_err) break;
  }

  Eigen::VectorXd z_final = x;
  if (s_ok) {
    Eigen::VectorXd resid = H_N * x - a_pos;
    z_final = x - H_N.transpose() * ldlt_S.solve(resid);
  }
  Eigen::Map<Eigen::VectorXd> fce(contactFce.data(), dim3);
  fce = z_final;
  for (int i = 0; i < nc; ++i) {
    if (contactFce[3 * i + 2] < 0)
      contactFce[3 * i + 0] = contactFce[3 * i + 1] = contactFce[3 * i + 2] = 0.0;
  }

  SIRE_PROFILE_PLOT("ps_vs_v8.iters", static_cast<double>(total_iters));
  DLOG(DEBUG) << "v8 iters=" << total_iters << " error=" << error;
  return error;
}

}  // namespace sire::physics::contact::ps_vs_solver_v5
