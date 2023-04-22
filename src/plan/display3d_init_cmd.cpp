#include "sire/plan/display3d_init_cmd.hpp"

#include <tuple>

#include "sire/core/constants.hpp"
#include "sire/core/geometry/geometry_base.hpp"
#include "sire/core/sire_log.hpp"
#include "sire/ext/json.hpp"
#include "sire/server/interface.hpp"

namespace sire::plan {
auto Display3dInit::prepareNrt() -> void {
  option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;
  // option() |= NOT_PRINT_CMD_INFO;
  for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
  // get control server config of geometry in part pool
  aris::dynamic::Model& model =
      dynamic_cast<aris::dynamic::Model&>(controlServer()->model());
  nlohmann::json geo_pool;
  // 取出 Part下面的每一个geometry
  for (sire::Size i = 0; i < model.partPool().size(); ++i) {
    nlohmann::json json;
    aris::dynamic::Part& part = model.partPool().at(i);
    for (sire::Size j = 0; j < part.geometryPool().size(); ++j) {
      dynamic_cast<geometry::GeometryBase&>(part.geometryPool().at(j))
          .to_json(json);
      geo_pool.push_back(json);
    }
  }
  // 设置part相关初始化的信息
  // ground 默认使用wobj0作为0点坐标系，下标为1
  aris::dynamic::Marker& ground = model.partPool().at(0).markerPool().at(1);
  nlohmann::json part_init_config;
  part_init_config.push_back(
      std::array<double, sire::kPosQuatSize>({0, 0, 0, 0, 0, 0, 1}));
  for (sire::Size i = 1; i < model.partPool().size(); ++i) {
    nlohmann::json json;
    aris::dynamic::Part& part = model.partPool().at(i);
    std::array<double, sire::kPosQuatSize> part_pq_buffer;
    part.markerPool().at(0).getPq(part_pq_buffer.data());
    //part.markerPool().at(0).getPq(ground, part_pq_buffer.data());
    part_init_config.push_back(part_pq_buffer);
  }

  std::vector<std::pair<std::string, std::any>> out_param;
  SIRE_LOG << geo_pool << "yes" << std::endl;
  out_param.push_back(
      std::make_pair<std::string, std::any>("geometry_pool", geo_pool));
  out_param.push_back(
      std::make_pair<std::string, std::any>("part_pq_init", part_init_config));
  ret() = out_param;
}

auto Display3dInit::collectNrt() -> void {}

Display3dInit::Display3dInit(const std::string& name) {
  aris::core::fromXmlString(command(), "<Command name=\"display3d_init\"/>");
}

ARIS_REGISTRATION {
  aris::core::class_<Display3dInit>("Display3dInit")
      .inherit<aris::plan::Plan>();
}
}  // namespace sire::plan