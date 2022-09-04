
//
// Created by ZHOUYC on 2022/6/14.
//

#include <iostream>
#include <thread>
#include <mutex>
#include <functional>
#include <map>
#include <memory>
#include <utility>
#include <vector>
#include <Eigen/Dense>
#include <aris.hpp>
#include <iostream>
#include <array>

#include "simulator/dynamic_simulator.hpp"

using namespace std;
using namespace aris::dynamic;
using namespace Eigen;

namespace zyc
{
    std::thread sim_thread_;
    std::mutex  sim_mutex_;

    static array<double, 7 * 16> link_pm{
      1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,
      1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,
      1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,
      1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,
      1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,
      1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,
      1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,
    };
    static double output_inv[6]{ 393.0,0.0,642.0,0.0,1.5708,0.0 };

    [[noreturn]] auto SimThreadGetData()->void {
        //初始化建立puma模型
        PumaParam param;
        param.a1 = 40;
        param.a2 = 275;
        param.a3 = 25;
        param.d1 = 342;
        param.d3 = 0.0;
        param.d4 = 280;
        param.tool0_pe[2] = 73;
        auto m = aris::dynamic::createModelPuma(param);
        auto& cs = aris::server::ControlServer::instance();
        cs.resetModel(m.release());
        cs.resetMaster(aris::control::createDefaultEthercatMaster(6, 0, 0).release());
        cs.resetController(
            aris::control::createDefaultEthercatController(6, 0, 0, dynamic_cast<aris::control::EthercatMaster&>(cs.master())
            ).release()
        );
        for (int i = 0; i < 6; ++i) {
            cs.controller().motorPool()[i].setMaxPos(3.14);
            cs.controller().motorPool()[i].setMinPos(-3.14);
            cs.controller().motorPool()[i].setMaxVel(3.14);
            cs.controller().motorPool()[i].setMinVel(-3.14);
            cs.controller().motorPool()[i].setMaxAcc(100);
            cs.controller().motorPool()[i].setMinAcc(-100);
        }
        cs.resetPlanRoot(aris::plan::createDefaultPlanRoot().release());
        std::cout << aris::core::toXmlString(cs) << std::endl; //print the control server state
        cs.init();
        cs.start();

        //线程sim_thread_中getRtData每100ms读取link位姿
        while (true) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            std::any data;
            cs.getRtData([](aris::server::ControlServer& cs, const aris::plan::Plan* p, std::any& data)->void {
                array<double, 7 * 16> link_pm;
                double link_inv[16]{ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
                auto m = dynamic_cast<aris::dynamic::Model*>(&cs.model());
                for (int i = 0; i < 7; ++i) {
                    m->partPool().at(i).getPm(link_inv);
                    for (int j = 0; j < 16; ++j) {
                        link_pm[j + i * 16] = link_inv[j];
                    }
                }
                //转换末端位姿
                m->partPool().at(6).getPm(link_inv);
                Matrix4d Matrix4d_pm;
                //一维数组改为二维
                int a = 0;
                int b = 0;
                for (int m = 0; m < 16; m++)
                {
                    a = m / 4;
                    b = m % 4;
                    Matrix4d_pm(a, b) = link_inv[m];
                }
                //     Matrix4d_pm = trsf.inverse() * Matrix4d_pm;
                Matrix4d Matrix4d_pm_new = Matrix4d::Zero();
                Matrix4d trsf, trsf_inv, temp;
                trsf << 0.0, 0.0, 1.0, 393.0,
                    0.0, 1.0, 0.0, 0.0,
                    -1.0, 0.0, 0.0, 642.0,
                    0.0, 0.0, 0.0, 1.0;
                trsf_inv = trsf.inverse();
                for (auto q = 0; q < 4; q++)
                {
                    for (auto w = 0; w < 4; w++)
                    {
                        Matrix4d_pm_new(q, w) = 0;
                        for (auto k = 0; k < 4; k++)
                        {
                            Matrix4d_pm_new(q, w) += Matrix4d_pm(q, k) * trsf_inv(k, w);
                        }
                    }
                }
                //二维变一维 获取
                for (int c = 0; c < 4; ++c) {
                    for (int d = 0; d < 4; ++d) {
                        link_pm[96 + c * 4 + d] = Matrix4d_pm_new(c, d);
                    }
                }
                data = link_pm;
                }, data);
            link_pm = std::any_cast<std::array<double, 16 * 7>>(data);
        }
    }

    auto InitSimulator()->void {
        if (sim_thread_.joinable()) {
            return;
        }
        else {
            //开启sim_thread_线程，每100ms读取模型link位姿
            sim_thread_ = std::thread(SimThreadGetData);
        }
    }

    auto SimPlan()->void {
        //发送仿真轨迹
        if (sim_thread_.joinable()) {
            auto& cs = aris::server::ControlServer::instance();
            try {
                cs.executeCmd("ds");
                cs.executeCmd("md");
                cs.executeCmd("en");
                cs.executeCmd("mvj --pe={393,0,642,0,1.5708,0}");
                cs.executeCmd("mvj --pe={580,0,642,0,1.5708,0}");
            }
            catch (std::exception& e) {
                std::cout << "cs:" << e.what() << std::endl;
            }
            return;
        }
    }

    void DynamicSimulator(std::array<double, 7 * 16>& link_pm_) {
        //guard为局部变量，分配在栈上，超出作用域即调用析构函数
        std::lock_guard<std::mutex> guard(sim_mutex_);
        link_pm_ = link_pm;
    }

}//namespace

