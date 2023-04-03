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
#include <stdio.h>
#include <cmath>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <io.h>

#include <hpp/fcl/BV/AABB.h>
#include <hpp/fcl/BV/OBBRSS.h>
#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/fcl/broadphase/broadphase_callbacks.h>
#include <hpp/fcl/broadphase/broadphase_collision_manager.h>
#include <hpp/fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <hpp/fcl/broadphase/default_broadphase_callbacks.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/data_types.h>
#include <hpp/fcl/distance.h>
#include <hpp/fcl/math/transform.h>
#include <hpp/fcl/mesh_loader/assimp.h>
#include <hpp/fcl/mesh_loader/loader.h>
#include <hpp/fcl/narrowphase/narrowphase.h>
#include <hpp/fcl/shape/geometric_shapes.h>

#include <aris.hpp>
#include <sire_lib_export.h>

namespace sire::physics::contact {
 using namespace hpp;
enum contact_state {in,out};
class SIRE_API BallRebound {
 public:
  static auto instance(const std::string& robot_stl_path = "./") -> BallRebound&;
  auto CalContact(std::array<double, 7>&, size_t&) -> void;
  auto GetFileName(const std::string& path, std::vector<std::string>& files)
      -> void;
  auto Fall(std::array<double, 7>& sphere_pq, std::array<double, 7>& sphere_vs,
            const double& delta_t) -> void;
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
  auto CalContactForce(const double& A, const double& B, const double& k,
                       const double& D, const double& r, const double& w,
                       const double& contact_time) -> double;
  auto ModifyContactPosition(contact_state state,
                             std::array<double, 7> & sphere_pq,
                             std::array<double, 7>& sphere_vs, 
                             const std::array<double, 7>& sphere_pq_old,
                             const std::array<double, 7>& sphere_vs_old,
                             fcl::CollisionObject* sphere,
                             fcl::CollisionObject* box,
                             const double& delta_t=0.01,
                             double max_iteration=100 ) -> bool;
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
