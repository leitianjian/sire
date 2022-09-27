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
struct Collision::Imp {
  Collision* collision_;
  std::thread collision_thread_;
  std::mutex collision_mutex_;
  std::array<double, 7 * 7> link_pq;
  int num_contacts;

  Imp(Collision* collision) : collision_(collision) {
    num_contacts = 0;
    link_pq = {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
               0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
               1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1};
  }
  Imp(const Imp&) = delete;
};

Collision::Collision(const std::string& ee_model_path) : imp_(new Imp(this)) {
  // Configure robot_ee geometry.
  fcl::internal::Loader loader;
  loader.load(ee_model_path);
  typedef fcl::BVHModel<fcl::OBBRSS> Model;
  std::shared_ptr<Model> ee_model = std::make_shared<Model>();
  fcl::Vec3f scale{1, 1, 1};
  fcl::internal::meshFromAssimpScene(scale, loader.scene, ee_model);

  fcl::CollisionObject robot_ee(ee_model);
  
  // Configure sphere geometry.
  const fcl::FCL_REAL r = 0.030;
  auto sphere_geometry = std::make_shared<fcl::Sphere>(r);
  // Poses of the geometry.
  fcl::Transform3f X_WS(fcl::Vec3f(0.560, 0, 0.642));
  fcl::CollisionObject sphere(sphere_geometry, X_WS);
  
  imp_->collision_thread_ = std::thread(
      [](int& num_contacts, std::array<double, 7 * 7>& link_pq,
         fcl::CollisionObject robot_ee,fcl::CollisionObject sphere) {
    // thread
    while (num_contacts == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      fcl::Transform3f X_WBV;
      // Quaternion3f 一个实部加三个虚部
      X_WBV.setTransform(
          fcl::Quaternion3f(link_pq[6 * 7 + 6], link_pq[6 * 7 + 3],
                            link_pq[6 * 7 + 4], link_pq[6 * 7 + 5]),
          fcl::Vec3f(link_pq[6 * 7], link_pq[6 * 7 + 1],
                     link_pq[6 * 7 + 2]));

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
      std::cout << "ee.quaternion: " << X_WBV.getQuatRotation() << std::endl
                << "ee.translation: " << X_WBV.getTranslation().transpose() << std::endl
                << "min_distance: " << dis_res.min_distance << std::endl;
    }
  },
      std::ref(imp_->num_contacts), std::ref(imp_->link_pq),
      robot_ee, sphere
  );
}
Collision::~Collision() {
 
}

auto Collision::instance(const std::string& ee_model_path) -> Collision& {
  static Collision instance(ee_model_path);
  return instance;
}

auto Collision::CalCollision(std::array<double, 7 * 7> linpq,
                             size_t& num_contacts) -> void {
  std::lock_guard<std::mutex> guard(imp_->collision_mutex_);
  imp_->link_pq = linpq;
  num_contacts = imp_->num_contacts;
}
}  // namespace sire
