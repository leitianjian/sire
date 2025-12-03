#ifndef SIRE_CONSTRAINT_SOLVER_HPP_
#define SIRE_CONSTRAINT_SOLVER_HPP_

#include <sire_lib_export.h>

#include <aris/dynamic/model_solver.hpp>

#include "sire/core/constants.hpp"

namespace sire::solver {
class SIRE_API JointConstraintSolver : public aris::dynamic::UniversalSolver {
 public:
  auto virtual allocateMemory() -> void override;
  auto virtual kinPos() -> int override;
  auto virtual kinVel() -> int override;
  auto virtual dynAccAndFce() -> int override;

  // auto cptJacobi() noexcept -> void;
  // auto mJf() const noexcept -> Size;  // equal mot num
  // auto nJf() const noexcept -> Size;  // equal ee num * 6
  // auto Jf() const noexcept -> const
  //     double*;  // inverse jacobi   ee_vs = Jf * ee_vs + mot_vs
  // auto cf() const noexcept -> const double*;  // dimension : mJ x 1

  virtual ~JointConstraintSolver();
  explicit JointConstraintSolver(Size max_iter_count = 100,
                                  double max_error = 1e-10);
  ARIS_DECLARE_BIG_FOUR(JointConstraintSolver);

//  private:
//   struct Imp;
//   aris::core::ImpPtr<Imp> imp_;
};
}  // namespace sire::solver

#endif
