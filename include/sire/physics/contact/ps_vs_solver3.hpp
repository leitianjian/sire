#ifndef SIRE_PS_VS_SOLVER3_HPP_
#define SIRE_PS_VS_SOLVER3_HPP_

#include "sire/physics/contact/sor_contact_solver.hpp"

namespace sire::physics::contact::ps_vs_solver3 {

using PsVsSolver3 = sor_contact::SorContactSolver;
using sor_contact::LhsVariableType;
using sor_contact::cptAccelExtVector;
using sor_contact::cptInitialCondition;
using sor_contact::cptKdMatrix;
using sor_contact::cptInvCpi;
using sor_contact::cptDAECoeff;
using sor_contact::cptFormulaX;
using sor_contact::cptFormulaIXdt;
using sor_contact::cptFormulaXComposeAb;
using sor_contact::cptFormulaIXdtComposeAbx0;
using sor_contact::findMinRootOriginal;
using sor_contact::findMinRootBisection;
using sor_contact::findMinRootSchur;
using sor_contact::findSinglePointContactEndTime;
using sor_contact::cptAvgContactFce;
using sor_contact::cptCpiMatrix;
using sor_contact::cptInverseCpiMatrix;
using sor_contact::filterPairsAndPreprocessInfo;
using sor_contact::cptContactForceWithTargetState2;
using sor_contact::cptGlbContactWrench;

}  // namespace sire::physics::contact::ps_vs_solver3

#endif  // SIRE_PS_VS_SOLVER3_HPP_
