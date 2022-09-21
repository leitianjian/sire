//
// Created by ZHOUYC on 2022/6/14.
//
#ifndef DYNAMIC_H_
#define DYNAMIC_H_

#include <sire_lib_export.h>
#include <aris.hpp>
#include <array>

namespace sire {
class SIRE_API Simulator {
 private:
  struct Imp;
  std::unique_ptr<Imp> imp_;
  aris::server::ControlServer& cs_;
  const std::string& cs_config_path_;
  const std::string& env_config_path_;

 private:
  Simulator();
  Simulator(const std::string& cs_config_path);
  ~Simulator();
  Simulator(const Simulator&) = delete;
  Simulator &operator=(const Simulator&) = delete;
 public:
  static auto instance(const std::string& cs_config_path = "./sire.xml") -> Simulator&;
  
};
auto SIRE_API InitSimulator() -> void;
auto SIRE_API DynamicSimulator(std::array<double, 7 * 16>& link_pm_) -> void;
auto SIRE_API SimPlan() -> void;
}  // namespace sire

#endif
