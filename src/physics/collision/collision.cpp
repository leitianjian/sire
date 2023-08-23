//
// Created by ZHOUYC on 2022/8/5.
//
#include "sire/physics/collision/collision.hpp"

#include <fstream>
#include <mutex>
#include <string>
#include <thread>

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
#include <io.h>

using namespace hpp;

namespace sire::physics::collision {
struct Collision::Imp {
  Collision* collision_;
  std::thread collision_thread_;
  std::mutex collision_mutex_;
  fcl::BroadPhaseCollisionManager* dynamic_tree_;
  std::array<double, 7 * 7> link_pq;
  size_t num_contacts;

  Imp(Collision* collision)
      : collision_(collision),
        dynamic_tree_(new fcl::DynamicAABBTreeCollisionManager()) {
    num_contacts = 0;
    link_pq = {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
               0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
               1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1};
  }
  Imp(const Imp&) = delete;
};

Collision::Collision(const std::string& robot_stl_path) : imp_(new Imp(this)) {
  std::vector<std::string> robot_stl_names;
  // 获取该路径下的所有文件
  GetFileName(robot_stl_path, robot_stl_names);

  std::vector<std::unique_ptr<fcl::CollisionObject>> co_robot;
  for (int i = 0; i < robot_stl_names.size(); ++i) {
    std::cout << robot_stl_names[i] << std::endl;
    // Configure robot geometry.
    fcl::internal::Loader loader;
    loader.load(robot_stl_names[i]);
    typedef fcl::BVHModel<fcl::OBBRSS> Model;
    std::shared_ptr<Model> bvhmodel = std::make_shared<Model>();
    fcl::Vec3f scale{1, 1, 1};
    fcl::internal::meshFromAssimpScene(scale, loader.scene, bvhmodel);
    std::unique_ptr<fcl::CollisionObject> co_link =
        std::make_unique<fcl::CollisionObject>(bvhmodel);
    imp_->dynamic_tree_->registerObject(co_link.get());
    co_robot.push_back(std::move(co_link));
  }
  // imp_->dynamic_tree_->update();
  imp_->dynamic_tree_->setup();

  // Configure sphere geometry.
  const fcl::FCL_REAL r = 0.030;
  auto sphere_geometry = std::make_shared<fcl::Sphere>(r);
  // Poses of the geometry.
  fcl::Transform3f X_WS(fcl::Vec3f(0.560, 0, 0.642));
  fcl::CollisionObject sphere(sphere_geometry, X_WS);

  imp_->collision_thread_ = std::thread(
      [](size_t& num_contacts, std::array<double, 7 * 7>& link_pq,
         std::vector<std::unique_ptr<fcl::CollisionObject>> co_robot,
         fcl::CollisionObject sphere,
         fcl::BroadPhaseCollisionManager* dynamic_tree) {
        // thread
        while (num_contacts == 0) {
          // std::cout << "check collision" << std::endl;
          std::this_thread::sleep_for(std::chrono::milliseconds(10));

          for (int i = 1; i < co_robot.size(); ++i) {
            co_robot[i]->setTransform(
                fcl::Quaternion3f(link_pq[i * 7 + 6], link_pq[i * 7 + 3],
                                  link_pq[i * 7 + 4], link_pq[i * 7 + 5])
                    .matrix(),
                fcl::Vec3f(link_pq[i * 7], link_pq[i * 7 + 1],
                           link_pq[i * 7 + 2]));
            co_robot[i]->computeAABB();
          }
          dynamic_tree->update();

          fcl::CollisionCallBackDefault collision_data;
          fcl::CollisionCallBackDefault manipulator_collision_data;
          fcl::DistanceCallBackDefault distance_data;
          // Compute collision - single contact and enable contact.
          fcl::CollisionRequest co_req(fcl::CollisionRequestFlag::CONTACT, 1);
          co_req.security_margin = fcl::FCL_REAL(1e-2);
          fcl::CollisionRequest co_req2(fcl::CollisionRequestFlag::CONTACT, 10);
          co_req2.security_margin = fcl::FCL_REAL(1e-2);
          collision_data.data.request = co_req;
          manipulator_collision_data.data.request = co_req2;
          dynamic_tree->collide(&manipulator_collision_data);
          dynamic_tree->collide(&sphere, &collision_data);
          // dynamic_tree->distance(&sphere, &distance_data);
          num_contacts = collision_data.data.result.numContacts();
          size_t man_num_contacts =
              manipulator_collision_data.data.result.numContacts();
          std::vector<fcl::Contact> contacts;
          collision_data.data.result.getContacts(contacts);
          std::vector<fcl::Contact> contacts2;
          manipulator_collision_data.data.result.getContacts(contacts2);
          // double min_distance = distance_data.data.result.min_distance;

          // std::cout << "min_distance: " << min_distance << std::endl;
          // std::cout << "num_contacts: " << num_contacts << std::endl;
          std::cout << "man_num_contacts: " << man_num_contacts << std::endl;

          if (num_contacts) {
            std::cout << "num " << contacts.size() << " contacts found"
                      << std::endl;
            for (const fcl::Contact& contact : contacts) {
              std::cout << "collision position at : " << contact.pos[0] << " "
                        << contact.pos[1] << " " << contact.pos[2] << std::endl;
              // std::cout << "min_distance: " << min_distance << std::endl;
            }
          }
          if (man_num_contacts) {
            std::cout << "self collision num " << contacts2.size()
                      << " contacts found" << std::endl;
            for (const fcl::Contact& contact : contacts2) {
              std::cout << "robot arm self collision position at : "
                        << contact.pos[0] << " " << contact.pos[1] << " "
                        << contact.pos[2] << std::endl;
              // std::cout << "min_distance: " << min_distance << std::endl;
            }
          }
        }  // thread_while
      },
      std::ref(imp_->num_contacts), std::ref(imp_->link_pq),
      std::move(co_robot), std::move(sphere), imp_->dynamic_tree_);
}
Collision::~Collision() {}

auto Collision::instance(const std::string& robot_stl_path) -> Collision& {
  static Collision instance(robot_stl_path);
  return instance;
}

auto Collision::GetFileName(const std::string& path,
                            std::vector<std::string>& files) -> void {
  // 文件句柄
  intptr_t hFile;
  // 文件信息
  struct _finddata_t fileinfo;
  std::string p;
  if ((hFile = _findfirst(p.assign(path).append("/*.STL").c_str(),
                          &fileinfo)) == -1) {
    std::cout << "Not Find STL File!" << std::endl;
    exit(-1);
  } else {
    do {
      if ((fileinfo.attrib & _A_SUBDIR)) {
        // 读取子目录文件
        /*if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") !=
          0) GetFileName(p.assign(path).append("/").append(fileinfo.name),
          files); */
      } else {
        files.push_back(p.assign(path).append("/").append(fileinfo.name));
      }
    } while (_findnext(hFile, &fileinfo) == 0);
    _findclose(hFile);
  }
}

auto Collision::CalCollision(std::array<double, 7 * 7> linpq,
                             size_t& num_contacts) -> void {
  std::lock_guard<std::mutex> guard(imp_->collision_mutex_);
  imp_->link_pq = linpq;
  num_contacts = imp_->num_contacts;
}
}  // namespace sire::physics::collision
