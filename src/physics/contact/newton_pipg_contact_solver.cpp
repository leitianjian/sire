#include "sire/physics/contact/newton_pipg_contact_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <aris/core/reflection.hpp>

#include "sire/core/profiler.hpp"

namespace sire::physics::contact::newton_pipg {
namespace {

using MatrixXdRM = Eigen::Matrix<double, -1, -1, Eigen::RowMajor>;

enum class ConeFace { kInterior, kVertex, kBoundary, kNonsmooth };

struct ConeProjection {
  Eigen::Vector3d value{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d jacobian{Eigen::Matrix3d::Zero()};
  ConeFace face{ConeFace::kNonsmooth};
};

auto projectConeWithJacobian(const Eigen::Vector3d& x, double mu,
                             double face_tol) -> ConeProjection {
  ConeProjection out;
  mu = std::max(mu, 1e-8);
  const Eigen::Vector2d t = x.head<2>();
  const double r = t.norm();
  const double normal = x(2);
  const double upper_gap = mu * normal - r;
  const double polar_gap = -normal / mu - r;

  if (upper_gap > face_tol) {
    out.value = x;
    out.jacobian.setIdentity();
    out.face = ConeFace::kInterior;
    return out;
  }
  if (polar_gap > face_tol) {
    out.face = ConeFace::kVertex;
    return out;
  }
  if (r <= face_tol || std::abs(upper_gap) <= face_tol ||
      std::abs(polar_gap) <= face_tol) {
    // Projection is not classically differentiable on these transition sets.
    // The value is still computed below, but Newton is disabled for this step.
    out.face = ConeFace::kNonsmooth;
  } else {
    out.face = ConeFace::kBoundary;
  }

  const double denom = 1.0 + mu * mu;
  const double beta = (mu * r + normal) / denom;
  if (beta <= 0.0 || r <= std::numeric_limits<double>::epsilon()) return out;

  const Eigen::Vector2d u = t / r;
  out.value.head<2>() = mu * beta * u;
  out.value(2) = beta;

  const Eigen::Matrix2d uu = u * u.transpose();
  const double scale = mu * beta / r;
  out.jacobian.topLeftCorner<2, 2>() =
      scale * (Eigen::Matrix2d::Identity() - uu) +
      (mu * mu / denom) * uu;
  out.jacobian.topRightCorner<2, 1>() = (mu / denom) * u;
  out.jacobian.bottomLeftCorner<1, 2>() = (mu / denom) * u.transpose();
  out.jacobian(2, 2) = 1.0 / denom;
  return out;
}

struct PipgEvaluation {
  Eigen::VectorXd f_next;
  Eigen::VectorXd dual_next;
  Eigen::VectorXd residual;
  Eigen::MatrixXd projection_jacobian;
  std::vector<ConeFace> faces;
  bool differentiable{true};
};

auto evaluatePipg(const Eigen::MatrixXd& P, const Eigen::VectorXd& q,
                  const Eigen::MatrixXd& A, const Eigen::VectorXd& rhs,
                  const std::vector<double>& mu, double alpha, double beta,
                  const Eigen::VectorXd& f, const Eigen::VectorXd& dual,
                  double face_tol) -> PipgEvaluation {
  const int dim = static_cast<int>(f.size());
  const int nc = dim / 3;
  PipgEvaluation e;
  e.f_next.resize(dim);
  e.projection_jacobian = Eigen::MatrixXd::Zero(dim, dim);
  e.faces.resize(nc);

  const Eigen::VectorXd projection_arg =
      f - alpha * (P * f + q + A.transpose() * dual);
  for (int i = 0; i < nc; ++i) {
    const auto p = projectConeWithJacobian(
        projection_arg.segment<3>(3 * i), mu[i], face_tol);
    e.f_next.segment<3>(3 * i) = p.value;
    e.projection_jacobian.block<3, 3>(3 * i, 3 * i) = p.jacobian;
    e.faces[i] = p.face;
    e.differentiable &= p.face != ConeFace::kNonsmooth;
  }
  e.dual_next = dual + beta * (A * (2.0 * e.f_next - f) - rhs);
  e.residual.resize(dim + A.rows());
  e.residual.head(dim) = e.f_next - f;
  e.residual.tail(A.rows()) = e.dual_next - dual;
  return e;
}

auto sameFaces(const std::vector<ConeFace>& a,
               const std::vector<ConeFace>& b) -> bool {
  return a.size() == b.size() && std::equal(a.begin(), a.end(), b.begin());
}

}  // namespace

auto cptContactForceNewtonPipg(
    sire::Size n, std::vector<double>& fri_coef,
    std::vector<double>& invM_3n, std::vector<double>& v0,
    std::vector<double>& v_target, std::vector<double>& b, double h,
    std::vector<double>& contactFce, sire::Size max_iters,
    double max_err) -> double {
  SIRE_PROFILE_FUNCTION();
  const int nc = static_cast<int>(n);
  const int dim = 3 * nc;
  if (nc == 0) {
    std::fill(contactFce.begin(), contactFce.end(), 0.0);
    return 0.0;
  }
  if (fri_coef.size() < n || invM_3n.size() < static_cast<size_t>(dim * dim) ||
      v0.size() < static_cast<size_t>(dim) || b.size() < static_cast<size_t>(dim) ||
      v_target.size() < n || contactFce.size() < static_cast<size_t>(dim))
    return std::numeric_limits<double>::infinity();

  const Eigen::Map<const MatrixXdRM> W(invM_3n.data(), dim, dim);
  const Eigen::Map<const Eigen::VectorXd> v0_map(v0.data(), dim);
  const Eigen::Map<const Eigen::VectorXd> b_map(b.data(), dim);
  Eigen::MatrixXd P = -h * W;
  P = 0.5 * (P + P.transpose());

  // Equality rows belong to the physical Delassus operator.  Keep them
  // unchanged when regularizing the objective below.
  Eigen::MatrixXd A_full(nc, dim);
  for (int i = 0; i < nc; ++i) A_full.row(i) = P.row(3 * i + 2);

  // Tiny regularization supplies the strong convexity assumed by Newton-PIPG
  // and removes null directions that do not affect contact acceleration.
  const double p_scale = std::max(1.0, P.norm());
  P += (1e-10 * p_scale) * Eigen::MatrixXd::Identity(dim, dim);
  const Eigen::VectorXd q = v0_map - h * b_map;

  Eigen::VectorXd rhs_full(nc);
  for (int i = 0; i < nc; ++i) {
    // Post-step velocity is g + H f.  The v5 DAE convention requires its
    // normal component to equal -v_target, hence H_N f = -v_target - g_N.
    rhs_full(i) =
        -v_target[i] - v0[3 * i + 2] + h * b[3 * i + 2];
  }

  // FR-lite: retain only independent equality directions.  This prevents
  // redundant contact-normal rows from making the Newton system singular.
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      A_full, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const double rank_tol = std::max(A_full.rows(), A_full.cols()) *
                          std::numeric_limits<double>::epsilon() *
                          std::max(1.0, svd.singularValues()(0));
  int rank = 0;
  while (rank < svd.singularValues().size() &&
         svd.singularValues()(rank) > rank_tol)
    ++rank;
  if (rank == 0) {
    std::fill(contactFce.begin(), contactFce.end(), 0.0);
    return rhs_full.lpNorm<Eigen::Infinity>();
  }
  const Eigen::MatrixXd U_r = svd.matrixU().leftCols(rank);
  const Eigen::MatrixXd A = U_r.transpose() * A_full;
  const Eigen::VectorXd rhs = U_r.transpose() * rhs_full;
  const Eigen::VectorXd rhs_discarded = rhs_full - U_r * rhs;
  const double equality_consistency =
      rhs_discarded.lpNorm<Eigen::Infinity>();

  const double norm_p = std::max(P.norm(), 1e-12);
  const double norm_a_sq = std::max(A.squaredNorm(), 1e-12);
  // alpha (||P|| + beta ||A||^2) = 0.95, satisfying the paper's bound.
  const double alpha = 0.5 / norm_p;
  const double beta = 0.45 / (alpha * norm_a_sq);
  const double face_tol = std::max(1e-10, 0.1 * max_err);

  Eigen::VectorXd f = Eigen::VectorXd::Zero(dim);
  Eigen::VectorXd dual = Eigen::VectorXd::Zero(rank);
  std::vector<ConeFace> previous_faces;
  int stable_faces = 0;
  double residual_norm = std::numeric_limits<double>::infinity();
  int newton_steps = 0;

  for (int iteration = 0; iteration < static_cast<int>(max_iters); ++iteration) {
    auto eval = evaluatePipg(P, q, A, rhs, fri_coef, alpha, beta, f, dual,
                             face_tol);
    residual_norm = eval.residual.lpNorm<Eigen::Infinity>();
    if (residual_norm <= max_err) break;

    stable_faces = sameFaces(eval.faces, previous_faces) ? stable_faces + 1 : 0;
    previous_faces = eval.faces;
    bool accepted_newton = false;

    // Face identification is deliberately conservative at mode transitions.
    if (stable_faces >= 2 && eval.differentiable) {
      const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dim, dim);
      const Eigen::MatrixXd df_df = eval.projection_jacobian * (I - alpha * P);
      const Eigen::MatrixXd df_dw =
          -alpha * eval.projection_jacobian * A.transpose();
      Eigen::MatrixXd JT(dim + rank, dim + rank);
      JT.topLeftCorner(dim, dim) = df_df;
      JT.topRightCorner(dim, rank) = df_dw;
      JT.bottomLeftCorner(rank, dim) = beta * A * (-I + 2.0 * df_df);
      JT.bottomRightCorner(rank, rank) =
          Eigen::MatrixXd::Identity(rank, rank) + 2.0 * beta * A * df_dw;
      Eigen::MatrixXd newton_matrix =
          Eigen::MatrixXd::Identity(dim + rank, dim + rank) - JT;
      Eigen::FullPivLU<Eigen::MatrixXd> lu(newton_matrix);
      if (lu.isInvertible()) {
        const Eigen::VectorXd step = lu.solve(eval.residual);
        if (step.allFinite() &&
            step.norm() <= 100.0 * std::max(1.0, eval.residual.norm())) {
          // Backtracking is a practical version of Algorithm 2's residual
          // safeguard.  Full steps are accepted near a strictly complementary
          // solution and recover quadratic convergence.
          for (double damping = 1.0; damping >= 1.0 / 64.0; damping *= 0.5) {
            const Eigen::VectorXd trial_f = f + damping * step.head(dim);
            const Eigen::VectorXd trial_dual = dual + damping * step.tail(rank);
            auto trial = evaluatePipg(P, q, A, rhs, fri_coef, alpha, beta,
                                      trial_f, trial_dual, face_tol);
            if (trial.residual.lpNorm<Eigen::Infinity>() < residual_norm) {
              f = trial_f;
              dual = trial_dual;
              accepted_newton = true;
              ++newton_steps;
              break;
            }
          }
        }
      }
    }
    if (!accepted_newton) {
      f = eval.f_next;
      dual = eval.dual_next;
    }
  }

  // Return a cone-feasible force even when the iteration budget is exhausted.
  for (int i = 0; i < nc; ++i) {
    auto p = projectConeWithJacobian(f.segment<3>(3 * i), fri_coef[i], face_tol);
    Eigen::Map<Eigen::Vector3d>(contactFce.data() + 3 * i) = p.value;
  }
  // Report the true fixed-point/equality error, including any inconsistent
  // component removed by FR-lite rank compression.
  residual_norm = std::max(residual_norm, equality_consistency);
  SIRE_PROFILE_PLOT("newton_pipg.residual", residual_norm);
  SIRE_PROFILE_PLOT("newton_pipg.newton_steps", static_cast<double>(newton_steps));
  return residual_norm;
}

auto NewtonPipgContactSolver::solveContactForceQP(
    sire::Size n, std::vector<double>& fri_coef,
    std::vector<double>& invM_3n, std::vector<double>& v0,
    std::vector<double>& v_target, std::vector<double>& b, double h,
    std::vector<double>& contactFce, sire::Size max_iters,
    double max_err) -> double {
  return cptContactForceNewtonPipg(n, fri_coef, invM_3n, v0, v_target, b, h,
                                   contactFce, max_iters, max_err);
}

ARIS_REGISTRATION {
  aris::core::class_<NewtonPipgContactSolver>("NewtonPipgContactSolver")
      .inherit<ps_vs_solver_v5::PsVsSolverV5>();
}

}  // namespace sire::physics::contact::newton_pipg
