#include "sire/simulator/force.hpp"
#include <aris.hpp>

namespace sire {
struct Force::Imp {
  Force* force_;
  aris::server::ControlServer& cs_;
  std::thread retrieve_rt_pm_thead_;
  std::array<double, 7 * 16> link_pm_{};
  std::array<double, 7 * 7> link_pq_{};
  std::array<double, 7 * 6> link_pe_{};
  std::array<double, 7 * 6> link_vs_{};
  std::array<double, 7 * 6> link_vs0_{};
  std::array<double, 7 * 6> link_as0_{};
  std::array<double, 7 * 6> link_as1_{};
  std::array<double, 7 * 6> link_as2_{};
  std::mutex mu_link_pm_;

  explicit Imp(Force* force)
      : force_(force), cs_(aris::server::ControlServer::instance()) {
    link_pm_ = {
        1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0,
        1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,
    };
    link_pq_ = {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
                0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1};
    link_pe_ = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    link_vs0_ = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    link_vs_ = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    link_as0_ = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    link_as1_ = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    link_as2_ = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  }

  Imp(const Imp&) = delete;
};

Force::Force(const std::string& cs_config_path) : imp_(new Imp(this)) {
  aris::core::fromXmlFile(imp_->cs_, cs_config_path);
  imp_->cs_.init();
  //  std::cout << aris::core::toXmlString(imp_->cs_) << std::endl;
  imp_->cs_.start();
  imp_->cs_.executeCmd("md");
  imp_->cs_.executeCmd("rc");


  imp_->retrieve_rt_pm_thead_ = std::thread(
      [](aris::server::ControlServer& cs,
         std::array<double, 7 * 16>& link_pm,
         std::array<double, 7 * 6>& link_pe,
         std::array<double, 7 * 6>& link_vs0,
         std::array<double, 7 * 6>& link_vs,
         std::array<double, 7 * 6>& link_as0,
         std::array<double, 7 * 6>& link_as1,
         std::array<double, 7 * 6>& link_as2, std::mutex& mu_link_pm) {


        const double delta = 0.01;
        static double ee[6]{0.393, 0.0, 0.642, 0.0, 1.5708, 0.0}, t{0.0};
        while (/*t <= 2 * aris::PI*/ true) {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));

          // 给定轨迹z=sin(t)
          t += delta;
          // ee[1] += t;
//          ee[2] += 0.01 * sin(t);

          auto m = dynamic_cast<aris::dynamic::Model*>(&cs.model());
          // 正动力学求解
          static double input_fce[6]{0.0,
                                     -1.8914000000000004,
                                     -2.2834000000000003,
                                     0.0000000000000194,
                                     2.4205999999999999,
                                     -0.0};
//          input_fce[0] += 0.1 * sin(t);
          m->setInputFce(input_fce);
          if (m->forwardDynamics())
            std::cout << "forward dynamic failed" << std::endl;

/*          double input_acc[6]{ 0, 0, 0, 0, 0, 0};
          m->setInputAcc(input_acc);
          double input_vel[6]{ 0, 0, 0, 0, 0, 0};
          m->setInputVel(input_vel);
          if ( m ->inverseDynamics())
            std::cout << "inverse dynamic failed" << std::endl;
          double result_Fce[6],result_FceIn[6];
          m->getOutputFce(result_Fce);
          std::cout << "Input Force of xb4:" << std::endl;
          aris::dynamic::dsp(1, 6, result_Fce);
          m->getInputFce(result_FceIn);
          std::cout << "Input Force of xb4:" << std::endl;
          aris::dynamic::dsp(1, 6, result_FceIn);*/

          std::any data;
          cs.getRtData(
              [&link_pm, &link_pe, &link_vs0, &link_vs, &link_as0, &link_as1, &link_as2, delta,m](
                  aris::server::ControlServer& cs, const aris::plan::Plan* p,
                  std::any& data) -> void {
                auto& motion = dynamic_cast<aris::dynamic::GeneralMotion&>(m->generalMotionPool().at(0));
                motion.setPoseType(aris::dynamic::GeneralMotion::PoseType::EULER123);
                // 获取杆件位姿
                int geoCount = 1;
                for (int i = 1; i < m->partPool().size(); ++i) {
                  auto& part = m->partPool().at(i);
                  // 没有geometry，直接复制part的位姿
                  if (part.geometryPool().size() == 0) {
                    //RK4
                  /*  part.getPe(link_pe.data() +
                               static_cast<long>(6) * geoCount);
                    part.getVs(link_vs.data() +
                               static_cast<long>(6) * geoCount);
                    part.getAs(link_as2.data() +
                               static_cast<long>(6) * geoCount);
                    static std::array<double, 6> kv1{0, 0, 0, 0, 0, 0},
                        kv2{0, 0, 0, 0, 0, 0}, kv3{0, 0, 0, 0, 0, 0},
                        kv4{0, 0, 0, 0, 0, 0};
                    static std::array<double, 6> kp1{0, 0, 0, 0, 0, 0},
                        kp2{0, 0, 0, 0, 0, 0}, kp3{0, 0, 0, 0, 0, 0},
                        kp4{0, 0, 0, 0, 0, 0};
                    for (int j = 0; j < 6; ++j) {
                      kv1[j] = link_as0[geoCount * 6 + j];
                      kv2[j] = link_as1[geoCount * 6 + j];
                      kv3[j] = link_as1[geoCount * 6 + j];
                      kv4[j] = link_as2[geoCount * 6 + j];
                      kp1[j] = link_vs[geoCount * 6 + j];
                      kp2[j] = link_vs[geoCount * 6 + j] + delta/2.0*kv1[j];
                      kp3[j] = link_vs[geoCount * 6 + j] + delta/2.0*kv2[j];
                      kp4[j] = link_vs[geoCount * 6 + j] + delta*kv3[j];
                      //下一时刻 i杆件 速度螺旋
                      link_vs[geoCount * 6 + j] +=delta / 6.0 *(kv1[j] + 2 * kv2[j] + 2 * kv3[j] + kv4[j]);
                      //下一时刻 i杆件 点位置和欧拉角
                      link_pe[geoCount * 6 + j] +=  delta/6.0*(kp1[j]+2*kp2[j]+2*kp3[j]+2*kp4[j]);
                    }

                    link_as0 = link_as1;
                    link_as1 = link_as2;

                    part.setVs(link_vs.data() +
                               static_cast<long>(6) * geoCount);
                    motion.updV();
                    part.setPe(link_pe.data() +
                               static_cast<long>(6) * geoCount);
                    motion.updP();
                    std::cout<<"link_pe"<<geoCount<<":";
                    aris::dynamic::dsp(1,6,link_pe.data() +
                                                 static_cast<long>(6) * geoCount);

                    //修正位姿
//                    m->forwardKinematics();*/
                   //RK
                   part.getPe(link_pe.data() +
                              static_cast<long>(6) * geoCount);
                   part.getVs(link_vs.data() +
                              static_cast<long>(6) * geoCount);
                   part.getAs(link_as1.data() +
                              static_cast<long>(6) * geoCount);
                   std::cout<<"link_as"<<geoCount<<":";
                   aris::dynamic::dsp(1,6,link_as1.data()+geoCount*6);
                   static std::array<double, 6> kv1{0, 0, 0, 0, 0, 0},
                       kv2{0, 0, 0, 0, 0, 0},kp1{0, 0, 0, 0, 0, 0},
                       kp2{0, 0, 0, 0, 0, 0};

                   for (int j = 0; j < 6; ++j) {
                     kv1[j] = link_as0[geoCount * 6 + j];
                     kv2[j] = link_as1[geoCount * 6 + j];
                     kp1[j] = link_vs0[geoCount * 6 + j];
                     kp2[j] = link_vs[geoCount * 6 + j];
                     //下一时刻 i杆件 速度螺旋
                     link_vs[geoCount * 6 + j] += delta/2.0*(kv1[j]+kv2[j]);
                     //下一时刻 i杆件 点位置和欧拉角
                     link_pe[geoCount * 6 + j] += delta/2.0*(kp1[j]+kp2[j]);
                   }
                   link_as0 = link_as1;
                   link_vs0 = link_vs;

                   part.setVs(link_vs.data() +
                              static_cast<long>(6) * geoCount);
                   part.setPe(link_pe.data() +
                              static_cast<long>(6) * geoCount);
                   std::cout<<"link_pe"<<geoCount<<":";
                   aris::dynamic::dsp(1,6,link_pe.data() +
                                                static_cast<long>(6) * geoCount);

                   //修正位姿
//                   m->forwardKinematicsVel();
                   motion.updV();
//                   m->forwardKinematics();
                   motion.updP();


                   part.getPm(link_pm.data() +
                             static_cast<long>(16) * geoCount);
                   ++geoCount;

                  } else {
                    // 获取Part位姿
                    std::array<double, 16> prtPm{1, 0, 0, 0, 0, 1, 0, 0,
                                                 0, 0, 1, 0, 0, 0, 0, 1};
                    part.getPm(prtPm.data());
                    for (int j = 0; j < part.geometryPool().size(); ++j) {
                      // 获取FileGeometry位姿
                      try {
                        auto geoPrtPm =
                            dynamic_cast<aris::dynamic::FileGeometry&>(
                                part.geometryPool().at(j))
                                .prtPm();
                        // T_part * inv(T_fg) = 3d模型真实位姿
                        aris::dynamic::s_pm_dot_inv_pm(
                            prtPm.data(), const_cast<double*>(*geoPrtPm),
                            link_pm.data() + static_cast<long>(16) * geoCount);
                        ++geoCount;
                      } catch (const std::bad_cast& e) {
                        std::cout << "part " << i << ", geometry " << j
                                  << " is not FileGeometry, continue"
                                  << std::endl;
                      }
                    }
                  }
                }
                data = link_pm;
              },
              data);
          std::lock_guard<std::mutex> guard(mu_link_pm);
          link_pm = std::any_cast<std::array<double, 16 * 7>>(data);
        }
      },
      std::ref(imp_->cs_), std::ref(imp_->link_pm_),std::ref(imp_->link_pe_),
      std::ref(imp_->link_vs0_), std::ref(imp_->link_vs_),
      std::ref(imp_->link_as0_), std::ref(imp_->link_as1_),
      std::ref(imp_->link_as2_), std::ref(imp_->mu_link_pm_));
}

Force::~Force() {
  imp_->cs_.stop();
  imp_->cs_.close();
}

auto Force::instance(const std::string& cs_config_path) -> Force& {
  static Force instance(cs_config_path);
  return instance;
}

auto Force::GetLinkPM(std::array<double, 7 * 16>& link_pm) -> void {
  std::lock_guard<std::mutex> guard(imp_->mu_link_pm_);
  link_pm = imp_->link_pm_;
}

auto Force::GetLinkPQ(std::array<double, 7 * 7>& link_pq) -> void {
  std::lock_guard<std::mutex> guard(imp_->mu_link_pm_);
  for (int i = 1; i < 7; ++i) {
    aris::dynamic::s_pm2pq(imp_->link_pm_.data() + i * 16,
                           imp_->link_pq_.data() + i * 7);
  }
  // pq: 7x1 点位置与四元数(position and quaternions) 三个虚部加一个实部\n
  link_pq = imp_->link_pq_;
}

auto Force::GetLinkPE(std::array<double, 7 * 6>& link_pe) -> void {
  std::lock_guard<std::mutex> guard(imp_->mu_link_pm_);
  for (int i = 1; i < 7; ++i) {
    aris::dynamic::s_pm2pe(imp_->link_pm_.data() + i * 16,
                           imp_->link_pe_.data() + i * 6);
  }
  link_pe = imp_->link_pe_;
}

auto Force::SimPlan() -> void {
  // 发送仿真轨迹
  if (imp_->retrieve_rt_pm_thead_.joinable()) {
    auto& cs = aris::server::ControlServer::instance();
    try {
      cs.executeCmd("ds");
      cs.executeCmd("md");
      cs.executeCmd("en");
      cs.executeCmd("mvj --pe={0.393, 0, 0.642, 0, 1.5708, 0}");
      cs.executeCmd("mvj --pe={0.393, -0.314, 0.652, 0, 1.5708, 0}");
      cs.executeCmd("mvj --pe={0.393, 0.314, 0.632, 0, 1.5708, 0}");
      cs.executeCmd("mvj --pe={0.393, 0, 0.642, 0, 1.5708, 0}");
    } catch (std::exception& e) {
      std::cout << "cs:" << e.what() << std::endl;
    }
    return;
  }
}

}  // namespace sire
