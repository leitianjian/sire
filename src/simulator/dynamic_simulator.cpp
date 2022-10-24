
//
// Created by ZHOUYC on 2022/6/14.
//
#include "sire/simulator/dynamic_simulator.hpp"
#include <aris.hpp>
#include <algorithm>
#include <functional>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>
#include <algorithm>

namespace sire {
struct Simulator::Imp {
  Simulator* simulator_;
  aris::server::ControlServer& cs_;
  std::thread retrieve_rt_pm_thead_;
  std::array<double, 7 * 16> link_pm_;
  std::array<double, 7 * 7> link_pq_;
  std::array<double, 7 * 6> link_pe_;
  std::mutex mu_link_pm_;

  Imp(Simulator* simulator)
      : simulator_(simulator), cs_(aris::server::ControlServer::instance()) {
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

Simulator::Simulator(const std::string& cs_config_path) : imp_(new Imp(this)) {
  aris::core::fromXmlFile(imp_->cs_, cs_config_path);
  imp_->cs_.init();
  //std::cout << aris::core::toXmlString(imp_->cs_) << std::endl;
  try {
    imp_->cs_.start();
    imp_->cs_.executeCmd("md");
    imp_->cs_.executeCmd("rc");
  } catch (const std::exception& err) {
    std::cout << "error 启动ControlServer错误，请检查配置文件" << std::endl;
    exit(1);
  }

  imp_->retrieve_rt_pm_thead_ = std::thread(
      [](aris::server::ControlServer& cs, std::array<double, 7 * 16>& link_pm,
         std::mutex& mu_link_pm) {
        while (true) {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
          std::any data;
          cs.getRtData(
              [&link_pm](aris::server::ControlServer& cs,
                         const aris::plan::Plan* p, std::any& data) -> void {
                auto m = dynamic_cast<aris::dynamic::Model*>(&cs.model());
                //获取杆件位姿
                int geoCount = 1;
                for (int i = 1; i < m->partPool().size(); ++i) {
                  auto& part = m->partPool().at(i);
                  // 没有geometry，直接复制part的位姿
                  if (part.geometryPool().size() == 0) {
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
      std::ref(imp_->cs_), std::ref(imp_->link_pm_),
      std::ref(imp_->mu_link_pm_));
}

Simulator::~Simulator() {
  imp_->cs_.stop();
  imp_->cs_.close();
}

auto Simulator::GetLinkPM(std::array<double, 7 * 16>& link_pm) -> void {
  std::lock_guard<std::mutex> guard(imp_->mu_link_pm_);
  link_pm = imp_->link_pm_;
}

auto Simulator::GetLinkPQ(std::array<double, 7 * 7>& link_pq) -> void {
  std::lock_guard<std::mutex> guard(imp_->mu_link_pm_);
  for (int i = 1; i < 7; ++i) {
    aris::dynamic::s_pm2pq(imp_->link_pm_.data() + i * 16,
                           imp_->link_pq_.data() + i * 7);
  }
  // pq: 7x1 点位置与四元数(position and quaternions) 三个虚部加一个实部\n
  link_pq = imp_->link_pq_;
}

auto Simulator::GetLinkPE(std::array<double, 7 * 6>& link_pe) -> void {
  std::lock_guard<std::mutex> guard(imp_->mu_link_pm_);
  for (int i = 1; i < 7; ++i) {
    aris::dynamic::s_pm2pe(imp_->link_pm_.data() + i * 16,
                           imp_->link_pe_.data() + i * 6);
  }
  link_pe = imp_->link_pe_;
}

auto Simulator::instance(const std::string& cs_config_path) -> Simulator& {
  static Simulator instance(cs_config_path);
  return instance;
}

auto Simulator::SimPlan() -> void {
  //发送仿真轨迹
  if (imp_->retrieve_rt_pm_thead_.joinable()) {
    auto& cs = aris::server::ControlServer::instance();
    try {
      cs.executeCmd("ds");
      cs.executeCmd("md");
      cs.executeCmd("en");
      cs.executeCmd("sire_mvj --pe={0.393, 0, 0.642, 0, 1.5708, 0}");
      cs.executeCmd("sire_mvj --pe={0.480, 0, 0.700, 0, 1.5708, 0}");
      cs.executeCmd("sire_mvj --pe={0.580, 0, 0.642, 0, 1.2, 0}");
      cs.executeCmd("sire_mvj --pe={0.412470,-0.011717,0.481322,1.570796,-1.570796,0.350983}");
      cs.executeCmd("sire_mvj --pe={0.408609,-0.005380,0.481783,1.570796,-1.570796,0.201510}");
      cs.executeCmd(
          "sire_mvj --pe={0.403391,-0.000154,0.482324,1.570796,-1.570796,-0.027216}");
      cs.executeCmd(
          "sire_mvj --pe = {0.396442,0.002176,0.482930,1.570796,-1.570796,-0.329585}");
      cs.executeCmd(
          "sire_mvj --pe = {0.389133, 0.001068, 0.483480, 1.570796, -1.570796, -0.522616}");

      cs.executeCmd("mvj --pe={0.580, 0, 0.642, 0, 1.2, 0}");
      cs.executeCmd("mvj --pe={0.393, 0, 0.642, 0, 1.5708, 0}");
    } catch (std::exception& e) {
      std::cout << "cs:" << e.what() << std::endl;
    }
    return;
  }
}

auto Simulator::SimPlan(std::vector<std::array<double, 6>> track_points)
    -> void {
  //发送仿真轨迹
  if (imp_->retrieve_rt_pm_thead_.joinable()) {
    auto& cs = aris::server::ControlServer::instance();
    try {
      cs.executeCmd("ds");
      cs.executeCmd("md");
      cs.executeCmd("en");
      //初始位置
      cs.executeCmd("mvj --pe={0.393, 0, 0.642, 0, 1.5708, 0}");
      //打磨位置
      for (auto track_point : track_points) {
        std::string str_tmp;
        for (auto i : track_point) {
          str_tmp += std::to_string(i) + ',';
        }
        std::cout << "mvj --pe={" + str_tmp + "}" << std::endl;
        cs.executeCmd("mvj --pe={" + str_tmp + "}");
      }
      //恢复初始
      cs.executeCmd("mvj --pe={0.393, 0, 0.642, 0, 1.5708, 0}");

    } catch (std::exception& e) {
      std::cout << "cs:" << e.what() << std::endl;
    }
    return;
  }
}

// ARIS_REGISTRATION { aris::core::class_<Simulator>("Simulator"); }
}  // namespace sire
