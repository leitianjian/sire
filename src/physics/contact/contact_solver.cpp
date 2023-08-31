#include "sire/physics/contact/contact_solver.hpp"

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
#include "sire/core/prop_map.hpp"
#include "sire/physics/collision/collision_detection.hpp"
#include "sire/physics/collision/collision_exists_callback.hpp"
#include "sire/physics/geometry/collidable_geometry.hpp"
#include "sire/physics/physics_engine.hpp"
#include "sire/transfer/part_pq_transfer.hpp"

namespace sire::physics::contact {
using PartPool =
    aris::core::PointerArray<aris::dynamic::Part, aris::dynamic::Element>;
struct ContactSolver::Imp {
  fcl::BroadPhaseCollisionManager* dynamic_tree_{nullptr};
  double g;
  double r;
  std::array<double, 7> sphere_pq;
  std::array<double, 7> sphere_vs;
  size_t num_contacts;
  double m, k, cr;
  double delta_t;
  double contact_time;

  physics::PhysicsEngine* engine_ptr_{nullptr};
  aris::server::ControlServer* server_{nullptr};
  PartPool* part_pool_ptr_{nullptr};
  sire::Size part_size_{0};

  explicit Imp()
      : g(9.8),  // m/s
        r(0.1),  // m
        m(1),    // kg
        k(1.4e8),
        cr(1),
        // delta_t(2), // delat_t = 2s
        // delta_t(0.00001),  // delat_t = 0.01ms
        // delta_t(0.000001),  // delat_t = 0.001ms
        delta_t(0.15),
        contact_time(0),
        num_contacts(0),
        sphere_pq({0, 0, 1, 0, 0, 0, 1}),
        sphere_vs({0, 0, 0, 0, 0, 0, 0}) {}
};
ContactSolver::ContactSolver() : imp_(new Imp) {}
ContactSolver::~ContactSolver(){};
ARIS_DEFINE_BIG_FOUR_CPP(ContactSolver);

auto ContactSolver::init(physics::PhysicsEngine* engine_ptr) -> void {
  imp_->engine_ptr_ = engine_ptr;
  imp_->server_ = &aris::server::ControlServer::instance();
  imp_->part_pool_ptr_ =
      &dynamic_cast<aris::dynamic::Model*>(&imp_->server_->model())->partPool();
  imp_->part_size_ = imp_->part_pool_ptr_->size();
}
auto ContactSolver::combineContactParameter(double k1, double k2, double d1,
                                            double d2)
    -> std::pair<double, double> {
  auto safe_div = [](double number, double denominator) -> double {
    return denominator == 0.0 ? 0.0 : number / denominator;
  };
  return std::pair<double, double>(safe_div(k1 * k2, k1 + k2),
                                   safe_div(d1 * d2, d1 + d2));
}
auto ContactSolver::cptContactSolverResult(
    const aris::dynamic::Model* current_state,
    const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    const std::vector<std::array<double, 16>>& T_C_vec,
    ContactSolverResult& result) -> void {
  for (int i = 0; i < penetration_pairs.size(); ++i) {
    const common::PenetrationAsPointPair& pair = penetration_pairs[i];
    core::PropMap& contact_prop_A =
        imp_->engine_ptr_->queryGeometryPoolById(pair.id_A)->contactProp();
    core::PropMap& contact_prop_B =
        imp_->engine_ptr_->queryGeometryPoolById(pair.id_B)->contactProp();
    auto [k, d] = combineContactParameter(
        contact_prop_A.getPropValueOrDefault("k", 2.8e7),
        contact_prop_B.getPropValueOrDefault("k", 2.8e7),
        contact_prop_A.getPropValueOrDefault("d", 2000.0),
        contact_prop_B.getPropValueOrDefault("d", 2000.0));
    double vn =
        imp_->engine_ptr_->cptProximityVelocity(pair);
    result.fn[i] = k * pair.depth - d * vn;
  }
}
auto ContactSolver::cptContactForce(double A, double B, double k, double D,
                                    double r, double w, double t) -> double {
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
auto ContactSolver::cptPenaltyODE(double contact_time, double proj_start_diff_v,
                                  double cr, double m, double k, double delta_t)
    -> double {
  double dt = delta_t;
  // penalty method with ODE
  static double d =
      2 * std::abs(std::log(cr)) *
      std::sqrt(k * m / (aris::PI * aris::PI + std::log(cr) * std::log(cr)));
  static double r = -d / (2 * m), w = std::sqrt((4 * k * m - d * d)) / (2 * m);
  // x_0 x_0'
  if (contact_time < 2 * delta_t) {
    // dt = contact_time;
    // imp_->contact_x_init = sphere_pq[2];
    // imp_->contact_v_init = sphere_vs[2];
    // imp_->position_contact.push_back(0);
    // imp_->velocity_contact.push_back(sphere_vs[2]);
    // imp_->acceleration_contact.push_back(-9.8);
    // imp_->force_contact.push_back(0);
  }
  double F_ext = m * imp_->g;
  double A = -F_ext / k, B = (proj_start_diff_v + r * F_ext / k) / w;
  // velocity
  double delta_v =
      (A * r + B * w) * std::exp(r * contact_time) *
          std::cos(w * contact_time) +
      (B * r - A * w) * std::exp(r * contact_time) * std::sin(w * contact_time);
  double next_t = contact_time /* + dt*/;

  // double delta_x =
  //     A * std::exp(r * next_t) * std::cos(w * next_t) +
  //                  B * std::exp(r * next_t) * std::sin(w * next_t) + F_ext /
  //                  k;
  double delta_x1 =
      A * std::exp(r * contact_time) * std::cos(w * contact_time) +
      B * std::exp(r * contact_time) * std::sin(w * contact_time) - F_ext / k;
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
      ContactSolver::*CollisionFilterPoolFunc)();
  aris::core::class_<ContactSolver>("ContactSolver");
}
}  // namespace sire::physics::contact