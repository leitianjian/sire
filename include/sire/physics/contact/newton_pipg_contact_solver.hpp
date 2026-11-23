#ifndef SIRE_NEWTON_PIPG_CONTACT_SOLVER_HPP_
#define SIRE_NEWTON_PIPG_CONTACT_SOLVER_HPP_

#include "sire/physics/contact/admm_contact_solver.hpp"

namespace sire::physics::contact::newton_pipg {

/// Hybrid PIPG/Newton solver for the equality-constrained Coulomb-cone QP
/// assembled by AdmmContactSolver.
class SIRE_API NewtonPipgContactSolver
    : public admm_contact::AdmmContactSolver {
 public:
  NewtonPipgContactSolver() = default;
  ~NewtonPipgContactSolver() override = default;

 protected:
  auto solveContactForceQP(
      sire::Size n, std::vector<double>& fri_coef,
      std::vector<double>& invM_3n, std::vector<double>& v0,
      std::vector<double>& v_target, std::vector<double>& b, double h,
      std::vector<double>& contactFce, sire::Size max_iters,
      double max_err) -> double override;
};

/// Standalone entry point, useful for regression tests and Python experiments.
auto cptContactForceNewtonPipg(
    sire::Size n, std::vector<double>& fri_coef,
    std::vector<double>& invM_3n, std::vector<double>& v0,
    std::vector<double>& v_target, std::vector<double>& b, double h,
    std::vector<double>& contactFce, sire::Size max_iters = 200,
    double max_err = 1e-8) -> double;

}  // namespace sire::physics::contact::newton_pipg

#endif
