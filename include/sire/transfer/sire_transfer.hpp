#ifndef SIRE_TRANSFER_HPP_
#define SIRE_TRANSFER_HPP_

#include <sire_lib_export.h>
#include <aris.hpp>

namespace sire::transfer {
class SIRE_API SireTransferModelController
    : public aris::server::TransferModelController {
 public:
  auto updateDataController2Model(const std::vector<std::uint64_t>& options,
                                  const aris::control::Controller* controller,
                                  aris::dynamic::ModelBase* model)
      -> void override;
  auto updateDataModel2Controller(const std::vector<std::uint64_t>& options,
                                  const aris::dynamic::ModelBase* model,
                                  aris::control::Controller* controller)
      -> void override;
  auto integrateAs2Ps(double* vs_in[3], double* as_in[3],
                      double* old_ps, double* ps_out) -> void;
  SireTransferModelController();

 private:
  aris::Size part_pool_length_;
  aris::Size motion_pool_length_;
  double dt_seconds_;
  std::vector<double> motion_force_;
  std::array<std::vector<double>, 2> parts_ps_array_;
  double* old_parts_ps_;
  double* now_parts_ps_;
  std::array<std::vector<double>, 3> parts_vs_array_;
  std::array<double*, 3> parts_vs_ptrs_;
  std::array<std::vector<double>, 3> parts_as_array_;
  std::array<double*, 3> parts_as_ptrs_;
  std::vector<aris::core::Matrix> parts_pm_;
};
}  // namespace sire::transfer

#endif