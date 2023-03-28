//
// Created by ZHOUYC on 2023/3/4.
//

#ifndef SIRE_BALL_REBOUND_HPP
#define SIRE_BALL_REBOUND_HPP

#include <array>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <aris.hpp>
#include <sire_lib_export.h>

namespace sire::physics::contact {
// using namespace hpp;
class SIRE_API BallRebound {
 public:
  static auto instance(const std::string& robot_stl_path = "./")
      -> BallRebound&;
  auto CalContact(std::array<double, 7>&, size_t&) -> void;
  auto GetFileName(const std::string& path, std::vector<std::string>& files)
      -> void;
  auto ContactSolver(std::array<double,7>,std::array<double,7>) -> std::array<double, 7>;
  auto CalImpulse(std::array<double, 7>& sphere_pq,
                  std::array<double, 7>& sphere_vs, 
                  const double& delta_t = 1e-2) -> void;
  auto CalPenalty(std::array<double, 7>& sphere_pq,
                  std::array<double, 7>& sphere_vs,
                  const double& min_distance,
                  const double& m = 1, 
                  const double& k = 1e3, 
                  const double& delta_t = 1e-2) -> void;
  auto CalPenaltyODE(const double& contact_time, 
                     std::array<double, 7>& sphere_pq,
                     std::array<double, 7>& sphere_vs, 
                     const double& cr = 1,
                     const double& m = 1,
                     const double& k = 1e3, 
                     const double& delta_t = 1e-2) -> void;
  auto CalPenaltyMaxwell(std::array<double, 7>& sphere_pq,
                         std::array<double, 7>& sphere_vs,
                         const double& min_distance, 
                         const double& cr = 1,
                         const double& m = 1, 
                         const double& k = 1e3, 
                         const double& delta_t = 1e-2) -> void;

 private:
  struct Imp;
  std::unique_ptr<Imp> imp_;
  BallRebound();
  ~BallRebound();
  BallRebound(const std::string& ee_model_path);
  BallRebound(const BallRebound&) = delete;
  BallRebound& operator=(const BallRebound&) = delete;
};
}  // namespace sire::physics::contact

#endif	
