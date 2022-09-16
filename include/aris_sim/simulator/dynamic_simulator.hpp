//
// Created by ZHOUYC on 2022/6/14.
//
#ifndef DYNAMIC_H_
#define DYNAMIC_H_

#include <aris_sim_lib_export.h>
#include <array>

namespace aris_sim {
auto ARIS_SIM_API InitSimulator() -> void;
auto ARIS_SIM_API DynamicSimulator(std::array<double, 7 * 16>& link_pm_) -> void;
auto ARIS_SIM_API SimPlan() -> void;
}  // namespace aris_sim

#endif
