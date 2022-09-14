//
// Created by ZHOUYC on 2022/6/14.
//
#ifndef DYNAMIC_H_
#define DYNAMIC_H_

#include <array>

namespace aris_sim {
auto InitSimulator() -> void;
auto DynamicSimulator(std::array<double, 7 * 16>& link_pm_) -> void;
auto SimPlan() -> void;
}  // namespace aris_sim

#endif
