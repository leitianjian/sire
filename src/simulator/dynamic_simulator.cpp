
//
// Created by ZHOUYC on 2022/6/14.
//

#include <iostream>
#include <thread>
#include <mutex>
#include <functional>
#include <vector>
#include <aris.hpp>
#include <array>
#include <algorithm>    // std::copy

#include "simulator/dynamic_simulator.hpp"

using namespace std;
using namespace aris::dynamic;


namespace ArisSim
{
    std::thread sim_thread_;
    std::mutex  sim_mutex_;

    static std::array<double ,7*16> link_pm{
            1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,
            1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,
            1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,
            1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,
            1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,
            1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,
            1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,
    };
    std::array<double, 16> temp{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
    const double ee[4][4]{{0.0  , 0.0  , 1.0 , 393.0},
                          {0.0  , 1.0  , 0.0 ,   0.0},
                          {-1.0 , 0.0  , 0.0 , 642.0},
                          {0.0  , 0.0  , 0.0 ,   1.0},};

    [[noreturn]] auto SimThreadGetData()->void{
      //初始化建立puma模型
      aris::dynamic::PumaParam param;
      param.a1 = 40;
      param.a2 = 275;
      param.a3 = 25;
      param.d1 = 342;
      param.d3 = 0.0;
      param.d4 = 280;
      param.tool0_pe[2] = 73;
      auto m = aris::dynamic::createModelPuma(param);
      auto &cs = aris::server::ControlServer::instance();
      cs.resetModel(m.release());
      cs.resetMaster(aris::control::createDefaultEthercatMaster(6, 0, 0).release());
      cs.resetController(
              aris::control::createDefaultEthercatController(
                      6, 0, 0, dynamic_cast<aris::control::EthercatMaster&>(cs.master())).release());
      for (int i = 0; i < 6; ++i) {
        cs.controller().motorPool()[i].setMaxPos(3.14);
        cs.controller().motorPool()[i].setMinPos(-3.14);
        cs.controller().motorPool()[i].setMaxVel(3.14);
        cs.controller().motorPool()[i].setMinVel(-3.14);
        cs.controller().motorPool()[i].setMaxAcc(100);
        cs.controller().motorPool()[i].setMinAcc(-100);
      }
      cs.resetPlanRoot(aris::plan::createDefaultPlanRoot().release());
      cs.init();
      cs.start();
      std::cout << aris::core::toXmlString(cs) << std::endl; //print the control server state

      //线程sim_thread_中getRtData每100ms读取link位姿
      while(true){
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::any data;
        cs.getRtData([](aris::server::ControlServer& cs, const aris::plan::Plan* p, std::any &data)->void{
            auto m = dynamic_cast<aris::dynamic::Model*>(&cs.model());
            //获取杆件位姿
            for(int i=1; i<m->partPool().size(); ++i){
              m->partPool().at(i).getPm(link_pm.data()+16*(i));
            }
            //转换末端位姿
            aris::dynamic::s_pm_dot_inv_pm(link_pm.data()+16*6, *ee, temp.data());
            std::copy(temp.begin(), temp.end(), link_pm.data()+16*6);
            data =link_pm;
        }, data);
        link_pm = std::any_cast<std::array<double,16*7>>(data);
      }
    }

    auto InitSimulator()->void {
      if (sim_thread_.joinable()) {
        return;
      } else {
        //开启sim_thread_线程，每100ms读取模型link位姿
        sim_thread_ = std::thread(SimThreadGetData);
      }
    }

    auto SimPlan()->void{
      //发送仿真轨迹
      if (sim_thread_.joinable()) {
        auto &cs = aris::server::ControlServer::instance();
        try {
          cs.executeCmd("ds");
          cs.executeCmd("md");
          cs.executeCmd("en");
          cs.executeCmd("mvj --pe={393,0,642,0,1.5708,0}");
          cs.executeCmd("mvj --pe={580,0,642,0,1.5708,0}");
        }
        catch (std::exception &e) {
          std::cout << "cs:" << e.what() << std::endl;
        }
        return;
      }
    }

    void DynamicSimulator(std::array<double,7*16> &link_pm_){
      std::lock_guard<std::mutex> guard(sim_mutex_);//guard为局部变量，分配在栈上，超出作用域即调用析构函数
      link_pm_ = link_pm;
    }

}//namespace

