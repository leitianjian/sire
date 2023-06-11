//
// Created by ZHOUYC on 2023/3/4.
//

#ifndef SIRE_BALL_REBOUND_HPP
#define SIRE_BALL_REBOUND_HPP

#include <stdio.h>
#include <array>
#include <cmath>
#include <vector>
#include <io.h>

#include <sire_lib_export.h>
#include <aris.hpp>

namespace sire::physics::contact {
class SIRE_API ContactForce {
 public:
  auto CalPenaltyODE(const double& contact_time,
                     std::array<double, 7>& sphere_pq,
                     std::array<double, 7>& sphere_vs, 
                     std::array<double, 7>& sphere_pq_next,
                     std::array<double, 7>& sphere_vs_next, 
                     const double& cr = 1,
                     const double& m = 1, const double& k = 1e10,
                     const double& delta_t = 1e-2) -> void;

 private:
  struct Imp;
  std::unique_ptr<Imp> imp_;
  ContactForce();
  ~ContactForce();
  ContactForce(const std::string& ee_model_path);
  ContactForce(const ContactForce&) = delete;
  ContactForce& operator=(const ContactForce&) = delete;
};
}  // namespace sire::physics::contact

#endif
