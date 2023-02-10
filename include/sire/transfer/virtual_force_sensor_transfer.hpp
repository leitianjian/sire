#ifndef VIRTUAL_FORCE_SENSOR_TRANSFER_HPP_
#define VIRTUAL_FORCE_SENSOR_TRANSFER_HPP_

#include <sire_lib_export.h>

#include <aris.hpp>

namespace sire::transfer {
class SIRE_API VirtualForceSensorTransfer
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
};
}  // namespace sire::transfer
#endif