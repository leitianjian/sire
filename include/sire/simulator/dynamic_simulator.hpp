//
// Created by ZHOUYC on 2022/6/14.
//
#ifndef DYNAMIC_H_
#define DYNAMIC_H_

#include <aris.hpp>
#include <array>
#include <sire_lib_export.h>

namespace sire {
class SIRE_API Simulator {
 private:
  struct Imp;
  std::unique_ptr<Imp> imp_;

 private:
  Simulator();
  ~Simulator();
  Simulator(const std::string& cs_config_path);
  Simulator(const Simulator&) = delete;
  Simulator& operator=(const Simulator&) = delete;

 public:
  static auto instance(const std::string& cs_config_path = "./sire.xml")
      -> Simulator&;
  auto GetLinkPM(std::array<double, 7 * 16>& link_pm) -> void;
  auto SimPlan() -> void;
};
}  // namespace sire

#endif
