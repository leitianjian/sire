#ifndef SIRE_PS_VS_SOLVER2_HPP_
#define SIRE_PS_VS_SOLVER2_HPP_

#include "sire/physics/contact/clarabel_contact_solver.hpp"

namespace sire::physics::contact::ps_vs_solver2 {

using PsVsSolver2 = clarabel_contact::ClarabelContactSolver;
using clarabel_contact::LhsVariableType;
using clarabel_contact::cptAccelExtVector;
using clarabel_contact::cptInitialCondition;
using clarabel_contact::cptKdMatrix;
using clarabel_contact::cptInvCpi;
using clarabel_contact::cptDAECoeff;
using clarabel_contact::cptFormulaX;
using clarabel_contact::cptFormulaIXdt;
using clarabel_contact::cptFormulaXComposeAb;
using clarabel_contact::cptFormulaIXdtComposeAbx0;
using clarabel_contact::findMinRootOriginal;
using clarabel_contact::findMinRootBisection;
using clarabel_contact::findMinRootSchur;
using clarabel_contact::cptAvgContactFce;
using clarabel_contact::cptCpiMatrix;
using clarabel_contact::cptInverseCpiMatrix;
using clarabel_contact::filterPairsAndPreprocessInfo;
using clarabel_contact::cptContactForceWithTargetState2;
using clarabel_contact::cptGlbContactWrench;

}  // namespace sire::physics::contact::ps_vs_solver2

#endif  // SIRE_PS_VS_SOLVER2_HPP_
