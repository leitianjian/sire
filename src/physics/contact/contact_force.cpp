//
// Created by ZHOUYC on 2023/3/4.
//

#include "sire/physics/contact/contact_force.hpp"

namespace sire::physics::contact {
struct ContactForce::Imp {
  ContactForce* contact_force_;
  std::thread collision_thread_;
  std::mutex collision_mutex_;
  double g;
  double r;
  std::array<double, 7> sphere_pq;
  std::array<double, 7> sphere_vs;
  std::array<double, 7> contact_x_init;
  std::array<double, 7> contact_v_init;
  double m, k, cr;
  double delta_t;
  double contact_time;

  explicit Imp(ContactForce* contact_force) : contact_force_(contact_force) {
    g = 9.8;  // m/s
    r = 0.1;  // m
    m = 1;    // kg
    k = 140 * 1e10;
    cr = 0.8;
    delta_t = 0.001; //delat_t = 1ms
    contact_time = 0;
    sphere_pq = {0, 0, 1, 0, 0, 0, 1};
    sphere_vs = {0, 0, 0, 0, 0, 0, 0};
    contact_x_init = {0, 0, 1, 0, 0, 0, 1};
    contact_v_init = {0, 0, 0, 0, 0, 0, 0};
  }
  Imp(const Imp&) = delete;
};
auto ContactForce::CalPenaltyODE(const double& contact_time,
                                std::array<double, 7>& sphere_pq,
                                std::array<double, 7>& sphere_vs,
                                std::array<double, 7>& sphere_pq_next,
                                std::array<double, 7>& sphere_vs_next,
                                const double& cr, const double& m,
                                const double& k, const double& delta_t) -> void {
  // penalty method with ODE
  static double d =
      2 * std::abs(std::log(cr)) *
      std::sqrt(k * m / (aris::PI * aris::PI + std::log(cr) * std::log(cr)));
  static double r = -d / (2 * m), w = std::sqrt((4 * k * m - d * d)) / (2 * m);
  // 记录第一次碰撞的位姿和速度 x_0 x_0'
  if (contact_time < 2 * delta_t) {
    imp_->contact_x_init = sphere_pq;
    imp_->contact_v_init = sphere_vs;
  }
  //外力项
  double F_ext = m * imp_->g;
  double A = -F_ext / k, B = (imp_->contact_v_init[2] + r * F_ext / k) / w; //TODO: contact_v_init[2]用的是y轴，应为碰撞方向
  // 方程的解 相对速度和相对位姿
  double delta_v =
      (A * r + B * w) * std::exp(r * contact_time) *
          std::cos(w * contact_time) +
      (B * r - A * w) * std::exp(r * contact_time) * std::sin(w * contact_time);
  double delta_x =
      A * std::exp(r * contact_time) * std::cos(w * contact_time) +
      B * std::exp(r * contact_time) * std::sin(w * contact_time) - F_ext / k;
  // contact force
  double force = m * (delta_v - sphere_vs[2]) / delta_t;
  sphere_pq_next[2] = imp_->contact_x_init[2] + delta_x; //TODO: contact_v_init[2]用的是y轴，应为碰撞方向
  sphere_vs_next[2] = delta_v;
}

ContactForce::~ContactForce() = default;

}  // namespace sire::physics::contact
