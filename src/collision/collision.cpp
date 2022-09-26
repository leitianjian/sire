//
// Created by ZHOUYC on 2022/8/5.
//
#include "aris_sim/collision/collision.hpp"
#include <hpp/fcl/collision.h>
#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/distance.h>
#include <hpp/fcl/math/transform.h>
#include <hpp/fcl/mesh_loader/assimp.h>
#include <hpp/fcl/mesh_loader/loader.h>
#include <hpp/fcl/shape/geometric_shapes.h>
#include <mutex>
#include <thread>

using namespace hpp;

namespace aris_sim {
static std::thread collision_thread_;
static std::mutex collision_mutex_;
static float x{0.0}, y{0.0}, z{0.0};
std::array<double, 7 * 7> link_pq;
static int num_contacts{0};

auto InitCollision(const std::string& resource_path) -> void {
  if (!collision_thread_.joinable()) {
    collision_thread_ = std::thread([resource_path]() {
      // Configure robot_ee geometry.
      fcl::internal::Loader loader;
      loader.load(resource_path);
      typedef fcl::BVHModel<fcl::OBBRSS> Model;
      std::shared_ptr<Model> geom = std::make_shared<Model>();
      fcl::Vec3f scale{1, 1, 1};
      fcl::internal::meshFromAssimpScene(scale, loader.scene, geom);
      geom->computeLocalAABB();
      fcl::Transform3f X_WBV = fcl::Transform3f::Identity();

      // Configure sphere geometry.
      const fcl::FCL_REAL r = 0.030;
      auto sphere_geometry = std::make_shared<fcl::Sphere>(r);
      // Poses of the geometry.
      fcl::Transform3f X_WS = fcl::Transform3f::Identity();
      // fcl碰撞模型 建立的时候 末端与小球只差0.167米
      X_WS.translation() << 0.560, 0, 0.642;
      fcl::CollisionObject sphere(sphere_geometry, X_WS);

      // thread
      while (num_contacts == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Quaternion3f 一个实部加三个虚部
        X_WBV.setTransform(
            fcl::Quaternion3f(link_pq[6 * 7 + 6], link_pq[6 * 7 + 3],
                              link_pq[6 * 7 + 4], link_pq[6 * 7 + 5]),
            fcl::Vec3f(link_pq[6 * 7], link_pq[6 * 7 + 1], link_pq[6 * 7 + 2]));

        fcl::CollisionObject robot_ee(geom, X_WBV);
        // Compute collision - single contact and enable contact.
        fcl::CollisionRequest col_req(fcl::CollisionRequestFlag::CONTACT, 1);
        // Request.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
        fcl::CollisionResult col_res;
        // Calculate collision
        num_contacts = fcl::collide(&sphere, &robot_ee, col_req, col_res);
        std::vector<fcl::Contact> contacts;
        col_res.getContacts(contacts);

        // set the distance request structure,
        // here we just use the default setting
        fcl::DistanceRequest dis_req;
        // result will be returned via the collision result structure
        fcl::DistanceResult dis_res;
        // perform distance test
        fcl::distance(&sphere, &robot_ee, dis_req, dis_res);
        if (num_contacts) {
          std::cout << "num " << contacts.size() << " contacts found"
                    << std::endl;
          for (const fcl::Contact& contact : contacts) {
            std::cout << "collision position at : " << contact.pos[0] << " "
                      << contact.pos[1] << " " << contact.pos[2] << std::endl;
            std::cout << "min_distance: " << dis_res.min_distance << std::endl;
          }
        }
      }
    });
  }
}

auto Collision(std::array<double, 7 * 7> linpq_, size_t& num_contacts_)
    -> void {
  std::lock_guard<std::mutex> guard(collision_mutex_);
  link_pq = linpq_;
  num_contacts_ = num_contacts;
};

}  // namespace aris_sim
