#ifndef SIRE_ANALYTICAL_IMPLICIT_FRICTION_SOLVER_HPP_
#define SIRE_ANALYTICAL_IMPLICIT_FRICTION_SOLVER_HPP_

#include <memory>
#include <vector>

#include <aris/core/object.hpp>

#include "sire/core/sire_decl_def_macro.hpp"
#include "sire/physics/contact/contact_solver.hpp"
#include "sire/physics/contact/contact_solver_result.hpp"

namespace sire::physics {
namespace geometry {
class CollidableGeometry;
}
namespace contact::analytical_implicit_friction {

/// @brief 解析 DAE + 隐式 Coulomb 投影接触求解器
///
/// 融合 analytical_tangent_force 的 DAE 框架（矩阵指数, RealSchur 求根）
/// 与 ps_vs_solver2 的隐式切向迭代思想，用 O(n) Coulomb 维投影替代 SOCP:
///
///   for each contact i:
///     f_tan_desired = 平衡切向外力 + 惯量耦合所需的力
///     if ||f_tan_desired|| ≤ μ_i · fn_i:
///         ft_i = f_tan_desired          → 静摩擦 (锥内)
///     else:
///         ft_i = μ_i · fn_i · normalize(f_tan_desired)  → 滑动
///
/// 优势: 静摩擦自然出现, 无 SOCP/Clarabel 依赖, 保留 DAE 精确穿透时间.
class SIRE_API AnalyticalImplicitFrictionSolver : public ContactSolver {
 public:
  AnalyticalImplicitFrictionSolver();
  virtual ~AnalyticalImplicitFrictionSolver();
  SIRE_DECLARE_MOVE_CTOR(AnalyticalImplicitFrictionSolver);
  virtual auto doInit(physics::PhysicsEngine* engine_ptr) -> void override {};

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
}  // namespace contact::analytical_implicit_friction
}  // namespace sire::physics
#endif
