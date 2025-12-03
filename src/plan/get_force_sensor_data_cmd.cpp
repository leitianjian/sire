#include "sire/plan/get_force_sensor_data_cmd.hpp"

#include "sire/controller/controller_sensor.hpp"
#include "sire/ext/json.hpp"
#include "sire/server/interface.hpp"

namespace sire::plan {
struct ForceDataConfig {};
struct ForceDataContainer {
  std::vector<std::vector<std::unique_ptr<aris::control::SensorData>>>
      data_sensor_force_;
  ForceDataContainer(sire::Size sensor_size = 6)
      : data_sensor_force_(sensor_size) {}
};
struct DataProperty {
  std::string name;
  std::string description;
  std::string unit;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(DataProperty, name, description);
};
// steam of SensorData
struct StreamStructure {
  std::string name;
  std::string description;
  std::vector<DataProperty> properties;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(StreamStructure, name, description,
                                 properties);
};
struct ForceDataRetStructure {
  std::string name;
  std::string description;
  std::vector<StreamStructure> data_streams;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(ForceDataRetStructure, name, description,
                                 data_streams);
};
auto GetForceSensorData::prepareNrt() -> void {
  option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_PRINT_CMD_INFO;
  for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
  bool show_info = true;
  bool show_config = false;
  for (const auto& cmd_param : cmdParams()) {
    if (cmd_param.first == "info") {
      show_info = true;
      show_config = false;
    }
    if (cmd_param.first == "config") {
      show_info = false;
      show_config = true;
    }
  }
  auto& sensor_pool = controller()->sensorPool();
  sire::Size sensor_pool_size = sensor_pool.size();
  sire::Size num_sensor_data_field = 1;
  if (show_info) {
    ForceDataContainer param;
    param.data_sensor_force_.resize(sensor_pool_size);
    std::vector<std::vector<double>> result_force(sensor_pool_size);
    for (int i = 0; i < sensor_pool_size; ++i) {
      try {
        sire::Size data_count = 0;
        auto& virtual_force_sensor =
            dynamic_cast<controller::BufferedMotorForceVirtualSensor&>(
                sensor_pool.at(i));
        param.data_sensor_force_[i].resize(virtual_force_sensor.bufferSize());
        virtual_force_sensor.retrieveBufferData(param.data_sensor_force_[i],
                                                data_count);
        result_force[i].resize(data_count);
        for (int j = 0; j < data_count; ++j) {
          result_force[i][j] = static_cast<controller::MotorForceData*>(
                                   param.data_sensor_force_[i][j].release())
                                   ->force_;
        }
      } catch (std::bad_cast& err) {
        std::cout << "sensor at " << i << " is not a virtual force sensor"
                  << std::endl;
        continue;
      }
    }
    std::vector<std::pair<std::string, std::any>> out_param;
    out_param.push_back(std::make_pair<std::string, std::any>(
        "motors_force", nlohmann::json(result_force)));
    ret() = out_param;
    return;
  }
  if (show_config) {
    auto& retStructure = ForceDataRetStructure();
    retStructure.name = "Force data structure";
    retStructure.description = "motor virtual force sensor data streams";
    retStructure.data_streams.resize(sensor_pool_size);
    for (int i = 0; i < sensor_pool_size; ++i) {
      auto& data_stream = StreamStructure();
      data_stream.description = "stream of sensor data";
      data_stream.name = "stream";
      data_stream.properties.resize(num_sensor_data_field);
      for (int j = 0; j < num_sensor_data_field; ++j) {
        auto& dataProperty = DataProperty();
        dataProperty.name = "force";
        dataProperty.description = "force magnitude";
        data_stream.properties[j] = dataProperty;
      }
      retStructure.data_streams[i] = data_stream;
    }
    std::vector<std::pair<std::string, std::any>> out_param;
    out_param.push_back(std::make_pair<std::string, std::any>(
        "motors_force_stream_config", nlohmann::json(retStructure)));
    ret() = out_param;
    return;
  }
}

auto GetForceSensorData::collectNrt() -> void {}

GetForceSensorData::GetForceSensorData(const std::string& name) {
  aris::core::fromXmlString(command(),
                            "<Command name=\"get_force\">"
                            "  <UniqueParam default=\"info\">"
                            "    <Param name=\"info\" abbreviation=\"i\"/>"
                            "    <Param name=\"config\" abbreviation=\"c\"/>"
                            "  </UniqueParam>"
                            "</Command>");
}

ARIS_REGISTRATION {
  aris::core::class_<GetForceSensorData>("SireGetForceSensorData")
      .inherit<aris::plan::Plan>();
}
}  // namespace sire::plan