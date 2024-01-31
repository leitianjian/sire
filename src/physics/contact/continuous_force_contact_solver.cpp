#include "sire/physics/contact/continuous_force_contact_solver.hpp"

#include <fstream>
#include <mutex>
#include <string>
#include <thread>

#include <hpp/fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <hpp/fcl/distance.h>
#include <hpp/fcl/math/transform.h>
#include <hpp/fcl/mesh_loader/assimp.h>
#include <hpp/fcl/mesh_loader/loader.h>
#include <hpp/fcl/shape/geometric_shapes.h>

#include <aris/core/reflection.hpp>
#include <aris/server/control_server.hpp>

#include "sire/core/constants.hpp"
#include "sire/core/material_manager.hpp"
#include "sire/core/prop_map.hpp"
#include "sire/physics/collision/collision_detection.hpp"
#include "sire/physics/collision/collision_exists_callback.hpp"
#include "sire/physics/geometry/collidable_geometry.hpp"
#include "sire/physics/physics_engine.hpp"
#include "sire/transfer/part_pq_transfer.hpp"

namespace sire::physics::contact {
using PartPool =
    aris::core::PointerArray<aris::dynamic::Part, aris::dynamic::Element>;
struct ContinuousForceContactSolver::Imp {
  unique_ptr<core::MaterialManager> material_manager_;
  double default_cr_;
  double default_k_;

  auto combineContactMass(double m1, double m2) -> double {
    auto safe_div = [](double number, double denominator) -> double {
      return denominator == 0.0 ? 0.0 : number / denominator;
    };
    return safe_div(m1 * m2, m1 + m2);
  }
  auto combineContactStiffness(double k1, double k2) -> double {
    auto safe_div = [](double number, double denominator) -> double {
      return denominator == 0.0 ? 0.0 : number / denominator;
    };
    return safe_div(k1 * k2, k1 + k2);
  }
  Imp()
      : material_manager_(std::make_unique<core::MaterialManager>()),
        default_cr_(0.8),
        default_k_(2.8e14) {}
};
ContinuousForceContactSolver::ContinuousForceContactSolver()
    : imp_(std::make_unique<Imp>()) {}
ContinuousForceContactSolver::~ContinuousForceContactSolver(){};
SIRE_DEFINE_MOVE_CTOR_CPP(ContinuousForceContactSolver);
auto ContinuousForceContactSolver::resetMaterialManager(
    core::MaterialManager* manager) -> void {
  imp_->material_manager_.reset(manager);
}
auto ContinuousForceContactSolver::materialManager() -> core::MaterialManager& {
  return *imp_->material_manager_;
}
auto ContinuousForceContactSolver::setDefaultStiffness(double k) noexcept
    -> void {
  imp_->default_k_ = k;
}
auto ContinuousForceContactSolver::defaultStiffness() noexcept -> double {
  return imp_->default_k_;
}
auto ContinuousForceContactSolver::setDefaultCr(double cr) noexcept -> void {
  imp_->default_cr_ = cr;
}
auto ContinuousForceContactSolver::defaultCr() noexcept -> double {
  return imp_->default_cr_;
}
auto ContinuousForceContactSolver::cptContactSolverResult(
    const aris::dynamic::Model* current_state,
    const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    const std::vector<std::array<double, 16>>& T_C_vec,
    ContactSolverResult& result) -> void {
  for (int i = 0; i < penetration_pairs.size(); ++i) {
    const common::PenetrationAsPointPair& pair = penetration_pairs[i];
    if (pair.depth <= 0) {
      // double vn = physicsEnginePtr()->cptProximityVelocity(pair);
      // if (pair.id_B == 1) {
      //   std::cout << " vn=" << vn << " fn1=" << result.fn[i] << " ";
      // }
      result.fn[i] = 0;
      continue;
    }
    auto* geometry_A = physicsEnginePtr()->queryGeometryPoolById(pair.id_A);
    auto* geometry_B = physicsEnginePtr()->queryGeometryPoolById(pair.id_B);
    core::PropMap& contact_prop_A = geometry_A->contactProp();
    core::PropMap& contact_prop_B = geometry_B->contactProp();
    double vn = physicsEnginePtr()->cptProximityVelocity(pair);
    const core::PropMap& pair_prop =
        imp_->material_manager_->getPropMapOrDefault(
            {geometry_A->material(), geometry_B->material()});
    double cr = pair_prop.getPropValueOrDefault("cr", imp_->default_cr_);
    double absLnCr = std::abs(cr == 0 ? 1000 : std::log(cr));
    auto k = imp_->combineContactStiffness(
        contact_prop_A.getPropValueOrDefault("k", imp_->default_k_),
        contact_prop_B.getPropValueOrDefault("k", imp_->default_k_));
    auto m = imp_->combineContactMass(
        this->partPoolPtr()->at(geometry_A->partId()).prtIv()[0],
        this->partPoolPtr()->at(geometry_A->partId()).prtIv()[0]);
    double fn =
        k * pair.depth +
        2 * absLnCr * vn *
            std::sqrt((k * m) / (sire::PI * sire::PI + absLnCr * absLnCr));
    // result.fn[i] = fn < 0 ? 0 : fn;
    result.fn[i] = fn;
    // if (pair.id_B == 1) {
    //   std::cout << " vn=" << vn << " fn1=" << result.fn[i] << " ";
    // }
    // std::cout << "kx=" << k * pair.depth << " vn=" << vn
    //           << " fn1=" << result.fn[i] << " ";
  }
}
auto ContinuousForceContactSolver::cptContactForce(double A, double B, double k,
                                                   double D, double r, double w,
                                                   double t) -> double {
  // 积分
  double first = (A * k + D * A * r + D * B * w) * r * r *
                 ((std::cos(w * t) * r) + w * std::sin(w * t)) *
                 std::exp(r * t) / (r * r + w * w);
  double second = (B * k + D * B * r - D * A * w) * r * r *
                  ((std::sin(w * t) * r) - w * std::cos(w * t)) *
                  std::exp(r * t) / (r * r - w * w);
  double force = first + second;
  // double first = (A * k * r * r + D * A * r * r * r + D * B * w * r * r) *
  //                ((std::cos(w * t) * r) + w * std::sin(w * t)) *
  //                std::exp(r * t) / (r * r + w * w);
  // double second = (B * k * r * r + D * B * r * r * r - D * A * w * r * r) *
  //                 ((std::sin(w * t) * r) - w * std::cos(w * t)) *
  //                 std::exp(r * t) / (r * r - w * w);
  // double force = first + second;
  // double force = (A * k + D * A * r + D * B * w)*((std::cos(w * t) / r) + w *
  // std::sin(w * t) / (r * r)) *
  //                    std::exp(r * t) / (1 + w * w / (r * r)) + (B * k + D * B
  //                    * r - D * A * w)*( (std::sin(w * t) / r) - w *
  //                    std::cos(w * t) / (r * r)) * std::exp(r * t) / (1 - w *
  //                    w / (r * r));
  return force;
}
auto ContinuousForceContactSolver::cptPenaltyODE(double contact_time,
                                                 double proj_start_diff_v,
                                                 double cr, double m, double k,
                                                 double delta_t) -> double {
  // double dt = delta_t;
  // // penalty method with ODE
  // static double d =
  //     2 * std::abs(std::log(cr)) *
  //     std::sqrt(k * m / (aris::PI * aris::PI + std::log(cr) * std::log(cr)));
  // static double r = -d / (2 * m), w = std::sqrt((4 * k * m - d * d)) / (2 *
  // m);
  // // x_0 x_0'
  // if (contact_time < 2 * delta_t) {
  //   // dt = contact_time;
  //   // imp_->contact_x_init = sphere_pq[2];
  //   // imp_->contact_v_init = sphere_vs[2];
  //   // imp_->position_contact.push_back(0);
  //   // imp_->velocity_contact.push_back(sphere_vs[2]);
  //   // imp_->acceleration_contact.push_back(-9.8);
  //   // imp_->force_contact.push_back(0);
  // }
  // double F_ext = m * imp_->g;
  // double A = -F_ext / k, B = (proj_start_diff_v + r * F_ext / k) / w;
  // // velocity
  // double delta_v =
  //     (A * r + B * w) * std::exp(r * contact_time) *
  //         std::cos(w * contact_time) +
  //     (B * r - A * w) * std::exp(r * contact_time) * std::sin(w *
  //     contact_time);
  // double next_t = contact_time /* + dt*/;
  //
  // // double delta_x =
  // //     A * std::exp(r * next_t) * std::cos(w * next_t) +
  // //                  B * std::exp(r * next_t) * std::sin(w * next_t) + F_ext
  // /
  // //                  k;
  // double delta_x1 =
  //     A * std::exp(r * contact_time) * std::cos(w * contact_time) +
  //     B * std::exp(r * contact_time) * std::sin(w * contact_time) - F_ext /
  //     k;
  // average contact force
  // double force = m * (delta_v - sphere_vs[2]) / dt;
  //
  // sphere_pq[2] = imp_->contact_x_init + delta_x1;
  // sphere_vs[2] = delta_v;
  //
  // double F = cptContactForce(A, B, k, d, r, w, contact_time) -
  //            cptContactForce(A, B, k, d, r, w,
  //                            contact_time - delta_t) /*+ F_ext * delat_t*/;
  // return F;
  return 0;

  // imp_->position_contact.push_back(delta_x1);
  // imp_->velocity_contact.push_back(sphere_vs[2]);
  // imp_->acceleration_contact.push_back(-force / m);
  // imp_->force_contact.push_back(force - F_ext);

  // if (sphere_pq[2] > imp_->contact_x_init) {
  //  //用速度回退到初始碰撞面
  //   double temp_x = sphere_pq[2];
  //   double temp_v = sphere_vs[2];
  //   double temp_a = contact_force / m;
  //   sphere_vs[2] = std::sqrt(temp_v * temp_v + 2 * temp_a *
  //   std::abs(imp_->contact_x_init - temp_x));  // a != g sphere_pq[2] =
  //   imp_->contact_x_init; double dt_modify =
  //       std::abs((sphere_vs[2] - temp_v) / temp_a);  //退回的时间差 // a != g
  //   std::cout << "----out dt_modify" << dt_modify << std::endl;
  //   sphere_vs[2] += -imp_->g * dt_modify;//只有重力
  //   sphere_pq[2] += sphere_vs[2] * dt_modify;
  // }
}

ARIS_REGISTRATION {
  typedef sire::physics::collision::CollisionFilter& (
      ContinuousForceContactSolver::*CollisionFilterPoolFunc)();
  typedef sire::core::MaterialManager& (
      ContinuousForceContactSolver::*MaterialManagerFunc)();
  aris::core::class_<ContinuousForceContactSolver>(
      "ContinuousForceContactSolver")
      .inherit<ContactSolver>()
      .prop("material_manager",
            &ContinuousForceContactSolver::resetMaterialManager,
            MaterialManagerFunc(&ContinuousForceContactSolver::materialManager))
      .prop("default_k", &ContinuousForceContactSolver::setDefaultStiffness,
            &ContinuousForceContactSolver::defaultStiffness);
}
}  // namespace sire::physics::contact