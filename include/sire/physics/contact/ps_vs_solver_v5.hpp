#ifndef SIRE_PS_VS_SOLVER_V5_HPP_
#define SIRE_PS_VS_SOLVER_V5_HPP_

#include "sire/physics/contact/admm_contact_solver.hpp"

namespace sire::physics::contact::ps_vs_solver_v5 {

using PsVsSolverV5 = admm_contact::AdmmContactSolver;
using admm_contact::cptContactForceWithTargetState5;
using admm_contact::cptContactForceWithTargetState6;
using admm_contact::cptContactForceWithTargetState7;
using admm_contact::cptContactForceWithTargetState8;

}  // namespace sire::physics::contact::ps_vs_solver_v5

#endif  // SIRE_PS_VS_SOLVER_V5_HPP_
