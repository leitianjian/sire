#include "sire/plan/get_geometry_cmd.hpp"

#include <tuple>

#include "sire/controller/controller_sensor.hpp"
#include "sire/core/geometry/geometry_base.hpp"
#include "sire/ext/json.hpp"
#include "sire/server/interface.hpp"

namespace sire::plan {
auto GetGeometry::prepareNrt() -> void {
  option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;
  //option() |= NOT_PRINT_CMD_INFO;
  for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
  // get control server config of geometry in part pool
  aris::dynamic::Model& model =
      dynamic_cast<aris::dynamic::Model&>(controlServer()->model());
  nlohmann::json geo_pool;
  for (sire::Size i = 0; i < model.partPool().size(); ++i) {
    nlohmann::json json;
    aris::dynamic::Part& part = model.partPool().at(i);
    for (sire::Size j = 0; j < part.geometryPool().size(); ++j) {
      dynamic_cast<geometry::GeometryBase&>(part.geometryPool().at(j))
          .to_json(json);
      geo_pool.push_back(json);
    }
  }
  std::vector<std::pair<std::string, std::any>> out_param;

  out_param.push_back(
      std::make_pair<std::string, std::any>("geometry_pool", geo_pool));
  ret() = out_param;
}

auto GetGeometry::collectNrt() -> void {}

GetGeometry::GetGeometry(const std::string& name) {
  aris::core::fromXmlString(command(), "<Command name=\"get_geometry\"/>");
}

ARIS_REGISTRATION {
  aris::core::class_<GetGeometry>("SireGetGeometry")
      .inherit<aris::plan::Plan>();
}
}  // namespace sire::plan