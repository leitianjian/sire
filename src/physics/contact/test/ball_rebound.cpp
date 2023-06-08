//
// Created by ZHOUYC on 2023/3/4.
//

#include "sire/physics/contact/test/ball_rebound.hpp"

// The ground is long and skinny with size
// (w, d, h) and its geometric frame is aligned with the world frame
// .
// Side view
//                                    small sphere
//                                         ↓
//                                         ◯  ↓g 
//                     z
//                     ┆ 
// ┏━━━━━━━━━━━━━━━━━━━┿━━━━━━━━━━━━━━━━━━━┓      ┬
// ╂┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┼┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄╂  x   h
// ┗━━━━━━━━━━━━━━━━━━━┿━━━━━━━━━━━━━━━━━━━┛      ┴
//                     ┆
//
// ├────────────────── w ──────────────────┤
//
// Top view
//                     y        
//                     ┆            
// ┏━━━━━━━━━━━━━━━━━━━┿━━━━━━━━━━━━━━━━━━━┓      ┬
// ╂┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄◯┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄╂  x   d
// ┗━━━━━━━━━━━━━━━━━━━┿━━━━━━━━━━━━━━━━━━━┛      ┴
//                     ┆


namespace sire::physics::contact {
struct BallRebound::Imp {
  BallRebound* ball_rebound_;
  std::thread collision_thread_;
  std::mutex collision_mutex_;
  fcl::BroadPhaseCollisionManager* dynamic_tree_;
  double g;
  double r;
  std::array<double, 7> sphere_pq;
  std::array<double, 7> sphere_vs;
  size_t num_contacts;
  double contact_x_init;
  double contact_v_init;
  double m, k, cr;
  double delta_t;
  double contact_time;
  std::vector<double> position_ball;
  std::vector<double> velocity_ball;

  std::vector<double> position_contact;
  std::vector<double> velocity_contact;
  std::vector<double> acceleration_contact;
  std::vector<double> force_contact;

  explicit Imp(BallRebound* ball_rebound) : ball_rebound_(ball_rebound) {
    g = 9.8;// m/s
    r = 0.1;// m
    m = 1;// kg
    k = 140*1e6;
    cr = 1;
    //delta_t = 2;  // delat_t = 2s
    //delta_t = 0.00001; //delat_t = 0.01ms
    //delta_t = 0.000001;//delat_t = 0.001ms
    delta_t = 150*1e-3;  
    contact_time = 0;
    num_contacts = 0;
    sphere_pq = {0, 0, 1, 0, 0, 0, 1};
    sphere_vs = {0, 0, 0, 0, 0, 0, 0};
    contact_x_init = 0;
    contact_v_init = 0;


  }
  Imp(const Imp&) = delete;
};
auto BallRebound::CalImpulse(std::array<double, 7>& sphere_pq,
                             std::array<double, 7>& sphere_vs,
                             const double& delta_t) -> void {
  // impluse
   sphere_vs[2] += (/*-9.8 + */ (-2 * sphere_vs[2]) / delta_t) * delta_t;
  // sphere_vs[2] = -sphere_vs[2];
   sphere_pq[2] += sphere_vs[2] * delta_t;
}
auto BallRebound::CalPenalty(std::array<double, 7>& sphere_pq,
                             std::array<double, 7>& sphere_vs,
                             const double& min_distance, 
                             const double& m,
                             const double& k,
                             const double& delta_t) -> void {
   std::cout << " sphere_vs[2]0 " << sphere_vs[2] << std::endl;
   sphere_vs[2] += (/*-9.8 + */k * (-min_distance) /m) * delta_t;// F=kx
   sphere_pq[2] += sphere_vs[2] * delta_t; 
   std::cout << " sphere_vs[2]1 " << sphere_vs[2] << std::endl;
   std::cout << " force "<< k * (-min_distance)  << std::endl;
}
auto BallRebound::CalPenaltyODE(const double& contact_time,
                                std::array<double, 7>& sphere_pq,
                                std::array<double, 7>& sphere_vs,
                                const double& cr, const double& m,
                                const double& k, const double& delta_t)-> void {
   double dt = delta_t;
   //penalty method with ODE
   static double d =
      2 * std::abs(std::log(cr)) *std::sqrt(k * m /(aris::PI * aris::PI + std::log(cr) *std::log(cr)));
   static double r = - d / (2 * m),
                 w = std::sqrt((4 * k * m - d * d))/ (2 * m);
  // x_0 x_0'
   if (contact_time < 2*delta_t){
     //std::cout << "start "
     //          << "w " << w << " t_all" <<aris::PI/w <<std::endl;
     //dt = contact_time;
     imp_->contact_x_init = sphere_pq[2];
     imp_->contact_v_init = sphere_vs[2];
     //std::cout << " contact_v_init " << sphere_vs[2] << std::endl;
     //std::cout << " contact_x_init " << sphere_pq[2] << std::endl;
     imp_->position_contact.push_back(0);
     imp_->velocity_contact.push_back(sphere_vs[2]);
     imp_->acceleration_contact.push_back(-9.8);
     imp_->force_contact.push_back(0);
   }
   double F_ext = m * imp_->g;
   double A =  - F_ext / k,
          B = (imp_->contact_v_init  + r * F_ext / k) / w;
  // velocity
   double delta_v =
       (A * r + B * w) * std::exp(r * contact_time) * std::cos(w * contact_time) +
       (B * r - A * w) * std::exp(r * contact_time) * std::sin(w * contact_time);
   double next_t = contact_time/* + dt*/;

   //double delta_x =
   //    A * std::exp(r * next_t) * std::cos(w * next_t) +
   //                 B * std::exp(r * next_t) * std::sin(w * next_t) + F_ext / k;
   double delta_x1 =  A * std::exp(r * contact_time) * std::cos(w * contact_time) +
                      B * std::exp(r * contact_time) * std::sin(w * contact_time) - F_ext/k ;
  // contact force
   double force = m * (delta_v - sphere_vs[2]) / dt;
   //std::cout << "contact_time: " << contact_time << std::endl;
   //std::cout << " sphere_vs[2]0 " << sphere_vs[2] << std::endl;
   //std::cout << " sphere_pq[2]0 " << sphere_pq[2] << std::endl;
   sphere_pq[2] = imp_->contact_x_init + delta_x1; 
   sphere_vs[2] = delta_v;
   //sphere_pq[2] += sphere_vs[2] * dt;
   //std::cout << " sphere_vs[2]1 " << sphere_vs[2] << std::endl;
   //std::cout << " sphere_pq[2]1 " << sphere_pq[2] << std::endl;
   //std::cout << " d " << d << std::endl;
   //std::cout << " x_init: " << imp_->contact_x_init
   //          << " v_init: " << imp_->contact_v_init
   //          << std::endl;
   //std::cout << " force " << contact_force << " " << F * delta_t << " spring"
   //          << k * (imp_->contact_x_init - sphere_pq[2]) << " "
   //          << d * sphere_vs[2] << std::endl;
   double F = CalContactForce(A, B, k, d, r, w, contact_time) -
              CalContactForce(A, B, k, d, r, w, contact_time - delta_t) /*+ F_ext * delat_t*/;
   // std::cout << " delta_x " << delta_x1 << std::endl;
   // std::cout << " delta_v " << delta_v << std::endl;
   //std::cout << " a " << force / m << std::endl;
   //std::cout << " contact_force " << force - F_ext << std::endl;
   // std::cout << " contact_force_cal "  << F * delta_t << std::endl;
   
   imp_->position_contact.push_back(delta_x1);
   imp_->velocity_contact.push_back(sphere_vs[2]);
   imp_->acceleration_contact.push_back(-force/m);
   imp_->force_contact.push_back(force-F_ext);

   //if (sphere_pq[2] > imp_->contact_x_init) {
   // //用速度回退到初始碰撞面
   //  double temp_x = sphere_pq[2];
   //  double temp_v = sphere_vs[2];
   //  double temp_a = contact_force / m;
   //  sphere_vs[2] = std::sqrt(temp_v * temp_v + 2 * temp_a * std::abs(imp_->contact_x_init - temp_x));  // a != g
   //  sphere_pq[2] = imp_->contact_x_init;
   //  double dt_modify =
   //      std::abs((sphere_vs[2] - temp_v) / temp_a);  //退回的时间差 // a != g
   //  std::cout << "----out dt_modify" << dt_modify << std::endl;
   //  sphere_vs[2] += -imp_->g * dt_modify;//只有重力
   //  sphere_pq[2] += sphere_vs[2] * dt_modify;
   //}
}

  auto BallRebound::CalContactForce(const double& A, const double& B, const double& k,
                     const double& D, const double& r, const double& w,
                     const double& t) -> double {
    //积分
  double first = (A * k * r * r + D * A * r * r * r + D * B * w * r * r) *
                 ((std::cos(w * t) * r) + w * std::sin(w * t)) *
                 std::exp(r * t) / (r * r + w * w);
  double second = (B * k * r * r + D * B * r * r * r - D * A * w * r * r) *
                 ((std::sin(w * t) * r) - w * std::cos(w * t)) *
                    std::exp(r * t) / (r * r - w * w);
  double force = first + second;
  //double force = (A * k + D * A * r + D * B * w)*((std::cos(w * t) / r) + w * std::sin(w * t) / (r * r)) *
  //                   std::exp(r * t) / (1 + w * w / (r * r)) + (B * k + D * B * r - D * A * w)*(
  //                   (std::sin(w * t) / r) - w * std::cos(w * t) / (r * r)) *
  //                   std::exp(r * t) / (1 - w * w / (r * r));
    return force;
  }

auto BallRebound::CalPenaltyMaxwell(std::array<double, 7>& sphere_pq,
                                    std::array<double, 7>& sphere_vs,
                                    const double& min_distance,
                                    const double& cr, const double& m,
                                    const double& k, const double& delta_t) -> void {
  // penalty method Maxwell
   static double d = 2 * std::abs(std::log(cr)) *std::sqrt(k * m /(aris::PI * aris::PI + std::log(cr) * std::log(cr)));
   double force = k * (-min_distance) + d * (-sphere_vs[2]);
   sphere_vs[2] += (-imp_->g + force / m) * delta_t;
   sphere_pq[2] += sphere_vs[2] * delta_t;
   std::cout << " force Maxwell " << force << std::endl;
   std::cout << " sphere_vs " << sphere_vs[2] << std::endl;
   std::cout << " min_distance " << min_distance << std::endl;
}

auto BallRebound::ModifyContactPosition(contact_state state,
  std::array<double, 7>& sphere_pq, std::array<double, 7>& sphere_vs,
  const std::array<double, 7>& sphere_pq_old,
  const std::array<double, 7>& sphere_vs_old, fcl::CollisionObject* sphere,
  fcl::CollisionObject* box, const double& delta_t ,
  double max_iteration ) -> bool {
  bool res{0};
  sphere_pq = sphere_pq_old;
  sphere_vs = sphere_vs_old;
  //std::cout << " sphere_vs[2] " << sphere_vs[2] << std::endl;
  //std::cout << " sphere_pq[2] " << sphere_pq[2] << std::endl;
  double iteration = max_iteration;
  while (max_iteration > 0) {
    --max_iteration;
    //std::cout << "max_iteration" << delta_t * (1 - max_iteration / iteration) << std::endl;
    if (state == in) {
      Fall(sphere_pq, sphere_vs, delta_t / iteration);
    } else {
      CalPenaltyODE(imp_->contact_time + delta_t * (-max_iteration/iteration), sphere_pq, sphere_vs, imp_->cr, imp_->m, imp_->k, imp_->delta_t/iteration);
    }
    //std::cout << " sphere_vs[2] " << sphere_vs[2] << std::endl;
    //std::cout << " sphere_pq[2] " << sphere_pq[2] << std::endl;
    sphere->setTranslation(
        fcl::Vec3f(sphere_pq[0], sphere_pq[1], sphere_pq[2]));
    sphere->computeAABB();
    fcl::CollisionResult co_res;
    fcl::CollisionRequest co_req(fcl::CollisionRequestFlag::CONTACT, 10);
    fcl::collide(sphere, box, co_req, co_res);
    if (state == in) {
      if (co_res.isCollision()) {
        return true;
      } 
    } else {
      if (!co_res.isCollision()) {
        return true;
      } 
    }
  }
  // std::cout << "max_iteration" << max_iteration << std::endl;
  return res;
} 

BallRebound::BallRebound(const std::string& robot_stl_path)
    : imp_(new Imp(this)) {
  // Configure box geometry.
  const fcl::FCL_REAL w = 20, d = 10, h = 0.5;
  auto box_geometry = std::make_shared<fcl::Box>(w, d, h);
  fcl::Transform3f X_WB(fcl::Vec3f(0, 0, -0.25));
  fcl::CollisionObject box(box_geometry, X_WB);
  // Configure sphere geometry.
  auto sphere_geometry = std::make_shared<fcl::Sphere>(imp_->r);
  fcl::Transform3f X_WS(
      fcl::Vec3f(imp_->sphere_pq[0], imp_->sphere_pq[1], imp_->sphere_pq[2]));
  fcl::CollisionObject sphere(sphere_geometry, X_WS);

  imp_->collision_thread_ = std::thread(
      [&](size_t& num_contacts, std::array<double, 7>& sphere_pq,
         std::array<double, 7>& sphere_vs, fcl::CollisionObject sphere,
         fcl::CollisionObject box,
         fcl::BroadPhaseCollisionManager* dynamic_tree) {
        // std::lock_guard<std::mutex> guard(imp_->collision_mutex_);
        //const double delta_t = 0.01;  // s
        //const double k = 1e3, m = 1, cr = 1;
        //double contact_time = 0;
        static double d = 2 * std::abs(std::log(imp_->cr)) * std::sqrt(imp_->k * imp_->m /
                      (aris::PI * aris::PI + std::log(imp_->cr) * std::log(imp_->cr)));
        static double r = - d / (2 * imp_->m),
                      w = std::sqrt((4 * imp_->k * imp_->m - d * d)) / (2 * imp_->m);
        static double contact_time_max = aris::PI / w;
        std::array<double, 7> sphere_vs_old = sphere_vs;
        std::array<double, 7> sphere_pq_old = sphere_pq;
        bool contact_end{false};

        // thread
        while (1) {
          auto start = std::chrono::steady_clock::now();
          // collision input
          sphere.setTranslation(
              fcl::Vec3f(sphere_pq[0], sphere_pq[1], sphere_pq[2]));
          sphere.computeAABB();
          fcl::CollisionCallBackDefault collision_data;
          fcl::DistanceCallBackDefault distance_data;
          fcl::CollisionResult co_res;
          fcl::CollisionRequest co_req(fcl::CollisionRequestFlag::CONTACT, 10);
          fcl::DistanceResult dis_res;
          fcl::DistanceRequest dis_req;
          co_req.security_margin = fcl::FCL_REAL(1e-10);
          collision_data.data.request = co_req;
          fcl::collide(&sphere, &box, co_req, co_res);
          fcl::distance(&sphere, &box, dis_req, dis_res);
          // output
          num_contacts = co_res.numContacts();
          double min_distance = dis_res.min_distance;
          std::vector<fcl::Contact> contacts;
          co_res.getContacts(contacts);
          //记录位置和速度
          imp_->position_ball.push_back(sphere_pq[2]);
          imp_->velocity_ball.push_back(sphere_vs[2]);
          if (num_contacts) {
           // std::cout << "num " << contacts.size()
           //           << " contacts found-------------------" << std::endl;
            for (const fcl::Contact& contact : contacts) {
              // std::cout << "collision position at : " << contact.pos[0] << "
              // "
              //          << contact.pos[1] << " " << contact.pos[2] <<
              //          std::endl;
              // std::cout << "min_distance: " << min_distance << std::endl;
            }
          }

          // falling
          if (num_contacts == 0 || contact_end) {
            //if (imp_->contact_time != 0) {
            ////侵出：修复回到接触初始位置
            //  std::cout << "contact out============================== " << std::endl;
            //  bool res = ModifyContactPosition(out, sphere_pq, sphere_vs, sphere_pq_old, sphere_vs_old, &sphere, &box, imp_->delta_t, 100);
            //}
            contact_end = false;
            sphere_vs_old = sphere_vs;
            sphere_pq_old = sphere_pq;
            imp_->contact_time = 0;
            Fall(sphere_pq, sphere_vs, imp_->delta_t);
          } else {
            if (imp_->contact_time == 0) {
              //侵入：修复回到接触初始位置
              //std::cout << "contact in============================== " << std::endl;
              bool res = ModifyContactPosition(in,sphere_pq, sphere_vs, sphere_pq_old, sphere_vs_old, &sphere, &box, imp_->delta_t, 10);
              if (imp_->contact_time > contact_time_max ) {
                //步长大于最大碰撞时间
                imp_->contact_time = contact_time_max - imp_->delta_t;
                //std::cout << "modify contact_time" << std::endl;
              }
            }

            imp_->contact_time += imp_->delta_t;
            if (imp_->contact_time > contact_time_max/*-imp_->delta_t*/) {
              //侵出：修复最多的碰撞时间
              imp_->contact_time = contact_time_max/* - imp_->delta_t*/;
              //std::cout << "modify contact_time" << std::endl;
              contact_end = true;
              //侵出：输出位置和速度
              {
                //std::cout << "位置" << std::endl;
                //for (int i = 0; i < imp_->position_ball.size(); i++) {
                //  std::cout << imp_->position_ball[i] << std::endl;
                //}
                ////imp_->position_ball.clear();
                //std::cout << "速度" << std::endl;
                //for (int i = 0; i < imp_->velocity_ball.size(); i++) {
                //  std::cout << imp_->velocity_ball[i] << std::endl;
                //}
                //imp_->velocity_ball.clear();

                //  std::cout << "位置" << std::endl;
                // for (int i = 0; i < imp_->position_contact.size(); i++) {
                //  std::cout << imp_->position_contact[i] << std::endl;
                //}
                ////imp_->position_ball.clear();
                // std::cout << "速度" << std::endl;
                // for (int i = 0; i < imp_->velocity_contact.size(); i++) {
                //  std::cout << imp_->velocity_contact[i] << std::endl;
                //}
                // //imp_->velocity_ball.clear();
                //std::cout << "加速度" << std::endl;
                //for (int i = 0; i < imp_->acceleration_contact.size(); i++) {
                //  std::cout << imp_->acceleration_contact[i] << std::endl;
                //}
                ////imp_->acceleration_contact.clear();
                //std::cout << "接触力" << std::endl;
                //for (int i = 0; i < imp_->force_contact.size(); i++) {
                //  std::cout << imp_->force_contact[i] << std::endl;
                //}
                ////imp_->force_contact.clear();

              }
            }
            sphere_vs_old = sphere_vs;
            sphere_pq_old = sphere_pq;
            CalPenaltyODE(imp_->contact_time, sphere_pq, sphere_vs, imp_->cr, imp_->m, imp_->k, imp_->delta_t);  // F = kx + dv
             //CalImpulse(sphere_pq, sphere_vs,delta_t);// v = -v
             //CalPenalty(sphere_pq, sphere_vs, min_distance, m, k,delta_t);// F = kx
             //CalPenaltyMaxwell(sphere_pq,sphere_vs, min_distance, cr, m, k,delta_t);// F = kx + d(cr)v

             if (contact_end == true) {
                //侵出：输出位置和速度
                // 
                // std::cout << "位置" << std::endl;
                // for (int i = 0; i < imp_->position_ball.size(); i++) {
                //  std::cout << imp_->position_ball[i] << std::endl;
                //}
                ////imp_->position_ball.clear();
                // std::cout << "速度" << std::endl;
                // for (int i = 0; i < imp_->velocity_ball.size(); i++) {
                //  std::cout << imp_->velocity_ball[i] << std::endl;
                //}
                // imp_->velocity_ball.clear();

                std::cout << "位置" << std::endl;
                for (int i = 0; i < imp_->position_contact.size(); i++) {
                  std::cout << imp_->position_contact[i] << std::endl;
                }
                // imp_->position_ball.clear();
                std::cout << "速度" << std::endl;
                for (int i = 0; i < imp_->velocity_contact.size(); i++) {
                  std::cout << imp_->velocity_contact[i] << std::endl;
                }
                // imp_->velocity_ball.clear();
                std::cout << "加速度" << std::endl;
                for (int i = 0; i < imp_->acceleration_contact.size(); i++) {
                  std::cout << imp_->acceleration_contact[i] << std::endl;
                }
                // imp_->acceleration_contact.clear();
                std::cout << "接触力" << std::endl;
                for (int i = 0; i < imp_->force_contact.size(); i++) {
                  std::cout << imp_->force_contact[i] << std::endl;
                }
                // imp_->force_contact.clear();
             }
          }
          //std::cout << "velocity:" << sphere_vs[2] << std::endl;
          //std::cout << "position:"
          //          << "x " << sphere_pq[0] << " y " << sphere_pq[1] << " z "
          //          << sphere_pq[2] << std::endl;

          std::this_thread::sleep_until(
              start + std::chrono::duration<double, std::milli>(imp_->delta_t * 1000));// 0.01*1000=10ms  1e-5+3 = 0.01ms
        }  // thread_while
      },
      std::ref(imp_->num_contacts), std::ref(imp_->sphere_pq),
      std::ref(imp_->sphere_vs), std::move(sphere), std::move(box),
      imp_->dynamic_tree_);
}

BallRebound::~BallRebound() = default;

// falling
auto BallRebound::Fall(std::array<double, 7>& sphere_pq,
                       std::array<double, 7>& sphere_vs, const double& delta_t)-> void {
  sphere_pq[2] += sphere_vs[2] * delta_t - 0.5 * imp_->g * delta_t * delta_t;
  sphere_vs[2] += -imp_->g * delta_t;
  //sphere_pq[2] += sphere_vs[2] * delta_t;
}

auto BallRebound::instance(const std::string& robot_stl_path)
    -> BallRebound& {
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
