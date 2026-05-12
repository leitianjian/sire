#ifndef SIRE_PS_VS_SOLVER2_HPP_
#define SIRE_PS_VS_SOLVER2_HPP_

#include <array>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <coal/broadphase/broadphase_callbacks.h>
#include <coal/broadphase/broadphase_collision_manager.h>
#include <coal/broadphase/default_broadphase_callbacks.h>
#include <coal/collision.h>
#include <coal/collision_data.h>
#include <coal/collision_object.h>

#include <aris/core/expression_calculator.hpp>
#include <aris/core/object.hpp>

#include "sire/core/sire_decl_def_macro.hpp"
#include "sire/physics/collision/collided_objects_callback.hpp"
#include "sire/physics/collision/collision_filter.hpp"
#include "sire/physics/common/penetration_as_point_pair.hpp"
#include "sire/physics/common/point_pair_contact_info.hpp"
#include "sire/physics/contact/contact_solver.hpp"
#include "sire/physics/contact/contact_solver_result.hpp"

namespace sire::physics {
namespace contact::ps_vs_solver2 {
// TODO: 手动去掉两个ground相关的碰撞。
enum class LhsVariableType { OneDelta, TwoAccel };
auto cptAccelExtVector(
    aris::dynamic::Model& model,
    const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    const std::vector<std::array<double, 16>>& T_C_vec,
    const std::vector<sire::Size>& preservedPairsIdx,
    const sire::PartId* prtIdVector, double* accelExt) -> void;
auto cptInitialCondition(
    sire::physics::PhysicsEngine& engine, sire::core::MaterialManager& manager,
    const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    const std::vector<std::array<double, 16>>& T_C_vec,
    const std::vector<sire::Size>& preservedPairsIdx,
    const std::vector<geometry::CollidableGeometry*>& geomPtrVector,
    double* stiffness, double* damping, double* x0, double* realDepthX0,
    double* v0) -> double;
auto cptKdMatrix(sire::Size n, const double* stiffness, const double* damping,
                 double* kdMatrix) -> void;
auto cptInvCpi(sire::Size n, sire::Size cpiWidth, double minDamp,
               const LhsVariableType* variableType, double* cpi,
               double* kdMatrix, double* fext, double* invCpi) -> void;
auto cptDAECoeff(sire::physics::PhysicsEngine& engine, sire::Size n,
                 const double* stiffness, const double* damping,
                 double stiffScale, double* accelExt, double* invCpi, double* A,
                 double* b) -> void;
auto cptFormulaX(sire::Size n, const double* A, double t, const double* b,
                 const double* x0, double* x) -> void;
auto cptFormulaIXdt(sire::Size n, const double* A, double t0, double tc,
                    const double* b, const double* x0, double* x) -> void;
auto cptFormulaXComposeAb(sire::Size n, const double* Ab, double t,
                          const double* x01, double* x1t) -> void;
auto cptFormulaIXdtComposeAbx0(sire::Size n, const double* Abx0, double t0,
                               double tc, double* ixdt) -> void;
auto findMinRootBisection(sire::Size nContact, const double* A, const double* b,
                          const double* x0, double tolerance,
                          sire::Size maxIter) -> double;
auto cptAvgContactFce(sire::Size nContact, const double* A, const double* b,
                      const double* x0, double t0, double tc,
                      const double* stiffness, const double* damping,
                      double* avgFce) -> void;
auto cptCpiMatrix(
    aris::dynamic::Model& model,
    const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    const std::vector<std::array<double, 16>>& T_C_vec,
    const std::vector<sire::Size>& preservedPairsIdx,
    const sire::PartId* prtIdVector, sire::Size cpiWidth, const int* groundFlag,
    double* cpi) -> void;
auto cptInverseCpiMatrix(
    aris::dynamic::Model& model,
    const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    const std::vector<std::array<double, 16>>& T_C_vec,
    const std::vector<sire::Size>& preservedPairsIdx,
    const sire::PartId* prtIdVector, double* accelExt, double* invCpi) -> void;
auto filterPairsAndPreprocessInfo(
    sire::physics::PhysicsEngine& engine,
    std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    std::vector<common::PenetrationAsPointPair>& contactEnded,
    std::vector<common::PenetrationAsPointPair>& contactNotEnd,
    std::vector<std::array<double, 16>>& T_C_vec,
    std::vector<sire::Size>& preservedPairsIdx,
    std::vector<sire::Size>& pairsNeedModifiedIdx,
    std::vector<sire::Size>& targetConditionIdx) -> void;
auto cptContactForceWithTargetState2(
    sire::Size n, std::vector<double>& fri_coef, std::vector<double>& invM,
    std::vector<double>& v0, std::vector<double>& v_target,
    std::vector<double>& b, double h, std::vector<double>& contactFce,
    sire::Size max_iters = 20, double max_err = 1e-2) -> double;
auto cptGlbContactWrench(
    aris::dynamic::Model& model, sire::physics::PhysicsEngine& engine,
    const std::vector<double>& contactFce,
    const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    const std::vector<std::array<double, 16>>& T_C_vec,
    const std::vector<sire::Size>& preservedPairsIdx,
    const std::vector<geometry::CollidableGeometry*>& geomPtrVector) -> void;
/* contact-based implementation */
class SIRE_API PsVsSolver2 : public ContactSolver {
 public:
  PsVsSolver2();
  virtual ~PsVsSolver2();
  SIRE_DECLARE_MOVE_CTOR(PsVsSolver2);
  virtual auto doInit(physics::PhysicsEngine* engine_ptr) -> void override {};

  // Material manager
  auto resetMaterialManager(core::MaterialManager* manager) -> void;
  auto materialManager() -> core::MaterialManager&;

  auto setDefaultStiffness(double k) noexcept -> void;
  auto defaultStiffness() noexcept -> double;
  auto setDefaultCr(double cr) noexcept -> void;
  auto defaultCr() noexcept -> double;
  auto setDefaultVelocityThreshold(double tv) noexcept -> void;
  auto defaultVelocityThreshold() noexcept -> double;
  auto debugByRecords() -> nlohmann::json override;
  virtual auto cptContactSolverResult(
      const aris::dynamic::Model* current_state,
      std::vector<common::PenetrationAsPointPair>& penetration_pairs,
      std::vector<std::array<double, 16>>& T_C_vec, ContactSolverResult& result)
      -> void override;

 private:
  struct Imp;
  std::unique_ptr<Imp> imp_;
};
}  // namespace contact::ps_vs_solver2
}  // namespace sire::physics
#endif