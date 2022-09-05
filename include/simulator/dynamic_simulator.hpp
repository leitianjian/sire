//
// Created by ZHOUYC on 2022/6/14.
//
#ifndef DYNAMIC_H_
#define DYNAMIC_H_

#include <iostream>
#include <thread>
#include <mutex>
#include <functional>
#include <vector>
#include <array>
#include <algorithm>

#include <aris.hpp>

namespace aris_sim {
    auto InitSimulator()->void;

    [[noreturn]] auto SimThreadGetData()->void;
    auto DynamicSimulator(std::array<double, 7 * 16>& link_pm_)->void;
    auto SimPlan()->void;

}

#endif
