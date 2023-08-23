#ifndef PART_PQ_TRANSFER_HPP_
#define PART_PQ_TRANSFER_HPP_

#include <sire_lib_export.h>

#include <aris/server/control_server.hpp>

#include "sire/core/constants.hpp"
#include "sire/integrator/integrator_base.hpp"

namespace sire::transfer {
class SIRE_API PartPQTransfer : public aris::server::TransferModelController {
 public:
  auto updateDataController2Model(const std::vector<std::uint64_t>& options,
                                  const aris::control::Controller* controller,
                                  aris::dynamic::ModelBase* model)
      -> void override;
  auto updateDataModel2Controller(const std::vector<std::uint64_t>& options,
                                  const aris::dynamic::ModelBase* model,
                                  aris::control::Controller* controller)
      -> void override;
  inline std::atomic<double*>& getPartsPq();
  inline const std::atomic<double*>& getPartsPq() const;
  inline void setPartsPq(double* ptr);
  inline double* exchange(double* ptr);
  PartPQTransfer();

 private:
  sire::Size part_pool_length_;
  sire::Size motion_pool_length_;
  sire::Size general_motion_pool_length_;
  std::vector<double> parts_pq_;
  std::atomic<double*> parts_pq_atomic_ptr_;
};
}  // namespace sire::transfer

#endif