#ifndef SIRE_AVG_FORCE_CONTACT_SOLVER_HPP_
#define SIRE_AVG_FORCE_CONTACT_SOLVER_HPP_

#include <array>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <hpp/fcl/broadphase/broadphase_callbacks.h>
#include <hpp/fcl/broadphase/broadphase_collision_manager.h>
#include <hpp/fcl/broadphase/default_broadphase_callbacks.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/collision_object.h>

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
namespace contact {
using namespace hpp;
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
    const std::vector<sire::Size>& preservedPairsIdx, double* stiffness,
    double* damping, double* x0, double* v0) -> double;
auto cptKdMatrix(sire::Size n, const double* stiffness, const double* damping,
                 double* kdMatrix) -> void;
auto cptInvCpi(sire::Size n, sire::Size cpiWidth, double minDamp,
               const LhsVariableType* variableType, double* cpi,
               double* kdMatrix, double* fext, double* invCpi) -> void;
auto cptDAECoeff(sire::physics::PhysicsEngine& engine, sire::Size n,
                 const double* stiffness, const double* damping, double stiffScale,
                 double* accelExt, double* invCpi, double* A, double* b)
    -> void;
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
auto preprocessContactInfo(
    sire::physics::PhysicsEngine& engine,
    const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    const std::vector<sire::Size>& preservedPairsIdx, sire::Size* pairStartIdx,
    sire::PartId* prtIdVector, LhsVariableType* variableType, int* groundFlag)
    -> void;
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
    const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    const std::vector<std::array<double, 16>>& T_C_vec,
    std::vector<sire::Size>& preservedPairsIdx,
    std::vector<sire::PartId>& prtIdVector, std::vector<double>& accelExt,
    std::vector<double>& invCpiResult) -> void;
/* contact-based implementation */
class SIRE_API AverageForceContactSolver : public ContactSolver {
 public:
  AverageForceContactSolver();
  virtual ~AverageForceContactSolver();
  SIRE_DECLARE_MOVE_CTOR(AverageForceContactSolver);
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
  virtual auto cptContactSolverResult(
      const aris::dynamic::Model* current_state,
      const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
      const std::vector<std::array<double, 16>>& T_C_vec,
      ContactSolverResult& result) -> void override;
  // compute contact point inertia matrix (aka. cpi)
  // Problem1: Model copy and copy subsystem or Model calc in place and reset
  // normally Solution1: 尝试梳理仿真 Model
  // 重置当时进行了哪些步骤，在这里应该要用的上
  // 当时是直接从Model里面重新读来解决的，就是要解决物理引擎里面指针的问题
  // Problem2: Model里面的例如 SingleComponentForce
  // 设置过控制的值，这个时候再添加generalForce并 init model
  //     会不会导致之前的控制失效
  // Solution2: ForcePool的修改好像不用
  // model->init()，因为本质上没有修改求解器和其他部分，只要自己初始化了就行
  // 我们这些加入的测试接触点的惯量的力也是在执行完之后就需要销毁的，可以看看
  // helpResetRAII的操作。
  auto cptContactPointInertiaMatrix(
      const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
      const std::vector<std::array<double, 16>>& T_C_vec,
      std::vector<double>& cpi) -> void;
  // 只计算碰撞点法向接触方向上的惯量矩阵，如果有 nContact 个碰撞点，
  // 返回的矩阵大小为 2*nContact x 2*nContact
  auto cptContactPointNormalInertiaMatrix(
      const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
      const std::vector<std::array<double, 16>>& T_C_vec,
      const double* stiffness, const double* damping, std::vector<double>& cpi,
      std::vector<double>& extInvCpi, std::vector<double>& A) -> sire::Size;
  // result: 2 * nContact * 1 vector with all collided prt normal fext
  auto cptContactPrtExtForce(
      const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
      const std::vector<std::array<double, 16>>& T_C_vec, const double* cpi,
      sire::Size cpiWidth, const double* extInvCpi, double* fext, double* b)
      -> void;
  auto preprocessContactInfo(
      const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
      sire::PartId* prtIdVector, LhsVariableType* variableType, int* groundFlag)
      -> sire::Size;
  auto cptCpiMatrix(
      const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
      const std::vector<std::array<double, 16>>& T_C_vec,
      const sire::PartId* prtIdVector, sire::Size cpiWidth,
      const int* groundFlag, double* cpi) -> void;

 private:
  auto cptContactForce(double A, double B, double k, double D, double r,
                       double w, double t) -> double;
  auto cptPenaltyODE(double contact_time, double projected_start_diff_v,
                     double cr, double m, double k, double delta_t) -> double;
  struct Imp;
  std::unique_ptr<Imp> imp_;
};
}  // namespace contact
}  // namespace sire::physics
#endif