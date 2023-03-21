//
// Created by ZHOUYC on 2023/3/4.
//

#include "sire/physics/contact/test/ball_rebound.hpp"

#include <stdio.h>

#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <cmath>

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

namespace sire::physics::contact {
using namespace hpp;
struct BallRebound::Imp {
  BallRebound* ball_rebound_;
  std::thread collision_thread_;
  std::mutex collision_mutex_;
  fcl::BroadPhaseCollisionManager* dynamic_tree_;
  std::array<double, 7> sphere_pq;
  std::array<double, 7> sphere_vs;
  size_t num_contacts;
  
  explicit Imp(BallRebound* ball_rebound) : ball_rebound_(ball_rebound) {
    num_contacts = 0;
    sphere_pq = {0, 0, 2, 0, 0, 0, 1};
    sphere_vs = {0, 0, 0, 0, 0, 0, 0};
  }
  Imp(const Imp&) = delete;
};

BallRebound::BallRebound(const std::string& robot_stl_path) : imp_(new Imp(this)) {
  // Configure box geometry.
  const fcl::FCL_REAL x = 20, y = 10, z = /*0.5*/ 20;
  auto box_geometry = std::make_shared<fcl::Box>(x, y, z);
  fcl::Transform3f X_WB( fcl::Vec3f(0, 0, -9.75));
  fcl::CollisionObject box(box_geometry,X_WB);
  // Configure sphere geometry.
  const fcl::FCL_REAL r = 0.300;//m
  auto sphere_geometry = std::make_shared<fcl::Sphere>(r);
  fcl::Transform3f X_WS(fcl::Vec3f(imp_->sphere_pq[0], imp_->sphere_pq[1], imp_->sphere_pq[2]));
  fcl::CollisionObject sphere(sphere_geometry, X_WS);

  imp_->collision_thread_ = std::thread(
  [](size_t& num_contacts, 
    std::array<double, 7>& sphere_pq,
    std::array<double, 7>& sphere_vs,
    fcl::CollisionObject sphere,
    fcl::CollisionObject box,
    fcl::BroadPhaseCollisionManager* dynamic_tree) {
        //std::lock_guard<std::mutex> guard(imp_->collision_mutex_);
    const double delta_t = 0.01;  // s 
    // thread
    while (1) {
      auto start = std::chrono::steady_clock::now();
      
      // collision input
      sphere.setTranslation(fcl::Vec3f(sphere_pq[0],sphere_pq[1],sphere_pq[2]));
      sphere.computeAABB();
      fcl::CollisionCallBackDefault collision_data;
      fcl::DistanceCallBackDefault distance_data;
      fcl::CollisionResult co_res;
      fcl::CollisionRequest co_req(fcl::CollisionRequestFlag::CONTACT, 10);
      fcl::DistanceResult dis_res;
      fcl::DistanceRequest dis_req;
      co_req.security_margin = fcl::FCL_REAL(1e-10);
      collision_data.data.request = co_req;
      fcl::collide(&sphere,&box,co_req,co_res);
      fcl::distance(&sphere, &box, dis_req, dis_res);
      //output
      num_contacts = co_res.numContacts();
      double min_distance = dis_res.min_distance;
      std::vector<fcl::Contact> contacts;
      co_res.getContacts(contacts);
      if (num_contacts) {
        std::cout << "num " << contacts.size() << " contacts found"
            << std::endl;
        for (const fcl::Contact& contact : contacts) {
          std::cout << "collision position at : " << contact.pos[0] << " "
                      << contact.pos[1] << " " << contact.pos[2] << std::endl;
          std::cout << "min_distance: " << min_distance << std::endl;
        }
      }

      // falling
      if (num_contacts == 0) {
        sphere_vs[2] += -9.8 * delta_t;
        sphere_pq[2] += sphere_vs[2] * delta_t;
      } else {
        // impluse
        //sphere_vs[2] += (/*-9.8 + */ (-2 * sphere_vs[2]) / delta_t) * delta_t;
        //// sphere_vs[2] = -sphere_vs[2];
        //sphere_pq[2] += sphere_vs[2] * delta_t;

        // penalty method
        // static double k = 0.99, m = 0.001;
        // //static double k = 4e4, m = 1;
        ////sphere_vs[2] += -9.8 * delta_t +
        ////               k * (-sphere_vs[2]) * delta_t * delta_t / (m * 2);//state
        // sphere_vs[2] += -9.8 * delta_t + k * (-min_distance) * delta_t / m;// min_dis
        // sphere_pq[2] += sphere_vs[2] * delta_t;
        // std::cout << " force "<< k * (-min_distance) * delta_t / m <<
        // std::endl;

        // penalty method with ODE
        //static double m = 1, k = 1e3, d = 1e1;
        //static double r = -d / (2 * m),
        //              w = std::sqrt(4 * k * m - d * d) / (2 * m);
        ////x_0 x_0'
        //static double x_init = sphere_pq[2],v_init=sphere_vs[2];
        //double F_ext = m*9.8;
        //double A=x_init-F_ext/k, B=(v_init-r*x_init+r*F_ext/k) / w;
        ////velocity
        //double temp_velocity =
        //    (A * r + B * w) * std::exp(r * delta_t) * std::cos(w * delta_t) +
        //    (B * r - A * w) * std::exp(r * delta_t) * std::sin(w * delta_t);
        ////contact force 
        //double contact_force = m * (sphere_pq[2] - temp_velocity ) / delta_t;
        //sphere_vs[2] +=( -9.8 + (contact_force/m)) * delta_t;
        //sphere_pq[2] += sphere_vs[2] * delta_t;
        //std::cout << " force " << contact_force << " spring" << k*(sphere_pq[2]-x_init) << std::endl;
        //std::cout << " temp_velocity " << temp_velocity << std::endl;
        //std::cout << " sphere_vs " << sphere_vs[2] << std::endl;

        // penalty method Maxwell
        static double k = 1e3, m = 1, cr = 1;
        static double d =
            2 * std::abs(std::log(cr)) *
            std::sqrt(k * m /
                      (aris::PI * aris::PI + std::log(cr) * std::log(cr)));
        double force = k * (-min_distance) + d * (-sphere_vs[2]);
         sphere_vs[2] += (-9.8 + force/m )* delta_t ;
         sphere_pq[2] += sphere_vs[2] * delta_t; 
         std::cout << " force Maxwell " << force << std::endl;
         std::cout << " sphere_vs " << sphere_vs[2] << std::endl;
         std::cout << " min_distance " << min_distance << std::endl;

      }
      std::cout << "position:"
                << "x " << sphere_pq[0] << " y " << sphere_pq[1] << " z "
                << sphere_pq[2] << std::endl;
      std::cout << "velocity:" << sphere_vs[2] << std::endl;
      std::this_thread::sleep_until(
          start + std::chrono::duration<double, std::milli>(delta_t * 1000));


    }  // thread_while
  },
  std::ref(imp_->num_contacts), std::ref(imp_->sphere_pq),
  std::ref(imp_->sphere_vs), std::move(sphere), std::move(box), imp_->dynamic_tree_);
}

BallRebound::~BallRebound() = default;

//falling
auto ContactSolver(std::array<double, 7> sphere_pq,
                   std::array<double, 7> sphere_vs) -> std::array<double, 7> {
  
  std::array<double, 7> res{0};
  return res;
}

auto BallRebound::instance(const std::string& robot_stl_path) -> BallRebound& {
  static BallRebound instance(robot_stl_path);
  return instance;
}

auto BallRebound::GetFileName(const std::string& path,
                            std::vector<std::string>& files) -> void {
  // 文件句柄
  intptr_t hFile;
  //文件信息
  struct _finddata_t fileinfo {};
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

auto BallRebound::CalContact(std::array<double, 7>& sphere_pq,
                             size_t& num_contacts) -> void {
  std::lock_guard<std::mutex> guard(imp_->collision_mutex_);
  sphere_pq = imp_->sphere_pq;
  num_contacts = imp_->num_contacts;
}
}  // namespace sire::physics::contact
