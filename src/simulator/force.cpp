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
      [](aris::server::ControlServer& cs, std::array<double, 7 * 16>& link_pm,
         std::mutex& mu_link_pm) {

        const double delta = 0.01;
        static double ee[6]{0.393,0.0,0.642,0.0,1.5708,0.0},t{0.0};
        while (t<=2*aris::PI) {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));






          // 给定轨迹z=sin(t)
          t += delta;
          // ee[1] += t;
          ee[2] += 0.01*sin(t);

          std::any data;
          cs.getRtData(
              [&link_pm](aris::server::ControlServer& cs,
                         const aris::plan::Plan* p, std::any& data) -> void {
                auto m = dynamic_cast<aris::dynamic::Model*>(&cs.model());

                //正动力学求解
                static double input_fce[6]{ 0.05, 0.1, 0.2, 0.1, 0.05, 0.1 };
                input_fce[0]+=0.01;
                m->setInputFce(input_fce);
                if (m->forwardDynamics())
                  std::cout << "forward dynamic failed" << std::endl;

                static double result_acc[6]{0,0,0,0,0,0};
                m->getOutputAcc(result_acc);



                for(int i=0;i<m->motionPool().size();i++){

                }



                std::cout << "Output acceleration of xb4:" << std::endl;
                aris::dynamic::dsp(1, 6, result_acc);

                //获取杆件位姿
                int geoCount = 1;
                for (int i = 1; i < m->partPool().size(); ++i) {
                  auto& part = m->partPool().at(i);
                  // 没有geometry，直接复制part的位姿
                  if (part.geometryPool().size() == 0) {
                    part.getPm(link_pm.data() +
                               static_cast<long>(16) * geoCount);
                    //                    part.get;
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
      std::ref(imp_->cs_), std::ref(imp_->link_pm_),
      std::ref(imp_->mu_link_pm_));
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
  //发送仿真轨迹
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
