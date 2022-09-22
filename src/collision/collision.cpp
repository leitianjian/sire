//
// Created by ZHOUYC on 2022/8/5.
//
#include "sire/collision/collision.hpp"
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

namespace sire {
static std::thread collision_thread_;
static std::mutex collision_mutex_;
static float x{0.0}, y{0.0}, z{0.0};
static int num_contacts{0};

auto InitCollision(const std::string& resource_path) -> void {
  if (!collision_thread_.joinable()) {
    collision_thread_ = std::thread([&resource_path]() {
      // Configure robot_ee geometry.
      fcl::internal::Loader loader;
      loader.load(resource_path);
      typedef fcl::BVHModel<fcl::OBBRSS> Model;
      std::shared_ptr<Model> ee_model = std::make_shared<Model>();
      fcl::Vec3f scale{1, 1, 1};
      fcl::internal::meshFromAssimpScene(scale, loader.scene, ee_model);
      ee_model->computeLocalAABB();
      fcl::CollisionObject robot_ee(ee_model);

      // Configure sphere geometry.
      const fcl::FCL_REAL r = 0.03;
      auto sphere_geometry = std::make_shared<fcl::Sphere>(r);
      // Poses of the geometry.
      fcl::Transform3f X_WS(fcl::Vec3f(0.560, 0, 0.642));
      fcl::CollisionObject sphere(sphere_geometry, X_WS);

      // thread
      while (num_contacts == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        robot_ee.setTranslation(fcl::Vec3f(x, y, z));

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

auto Collision(float x_, float y_, float z_, size_t& num_contacts_) -> void {
  std::lock_guard<std::mutex> guard(collision_mutex_);
  x = x_;
  y = y_;
  z = z_;
  num_contacts_ = num_contacts;
};

}  // namespace sire
