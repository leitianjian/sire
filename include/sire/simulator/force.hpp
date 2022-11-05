
#ifndef ARIS_SIM_FORCE_HPP
#define ARIS_SIM_FORCE_HPP

#include <aris.hpp>
#include <sire_lib_export.h>

namespace sire {
class SIRE_API Force {
 private:
  struct Imp;
  std::unique_ptr<Imp> imp_;

 private:
  Force();
  ~Force();
  explicit Force(const std::string& cs_config_path);
  Force(const Force&) = delete;
  Force& operator=(const Force&) = delete;

 public:
  static auto instance(const std::string& cs_config_path = "./sire.xml")
      -> Force&;
  auto GetLinkPM(std::array<double, 7 * 16>& link_pm) -> void;
  auto GetLinkPQ(std::array<double, 7 * 7>& link_pq) -> void;
  auto GetLinkPE(std::array<double, 7 * 6>& link_pe) -> void;
  auto SimPlan() -> void;

 private:


};
}  // namespace sire

#endif  // ARIS_SIM_FORCE_HPP
