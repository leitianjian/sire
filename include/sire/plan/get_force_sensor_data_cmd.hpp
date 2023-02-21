#ifndef SIRE_GET_FORCE_SENSOR_DATA_COMMAND_H_
#define SIRE_GET_FORCE_SENSOR_DATA_COMMAND_H_

#include <aris.hpp>

namespace sire::plan {

class GetForceSensorData
    : public aris::core::CloneObject<GetForceSensorData, aris::plan::Plan> {
 public:
  auto virtual prepareNrt() -> void override;
  auto virtual collectNrt() -> void override;
  explicit GetForceSensorData(
      const std::string& name = "GetForceSensorData_plan");
};

}  // namespace sire::plan
#endif