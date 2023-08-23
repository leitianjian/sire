#include "sire/cam_backend/command.hpp"

#include "sire/cam_backend/cam_backend.hpp"
#include "sire/core/constants.hpp"
#include "sire/ext/json.hpp"
#include "sire/middleware/sire_middleware.hpp"
#include "sire/server/interface.hpp"

namespace sire::cam_backend {
struct CptCollisionMapParam {
  WobjToolInstallMethod tool_install;
  // 计算模式，指计算图显示的是前倾角和侧倾角(-90 - 90)或工具轴角(360度旋转）
  int option;
  // 和SprutCam的含义相同，即为计算碰撞的分辨率
  sire::Size resolution;
  // 刀具路径点的数量
  sire::Size point_size;                    // n
  // 每个路径点的空间位置
  std::vector<double> points;               // n * 3
  // 工具轴角度，每个点有一个
  std::vector<double> tool_axis_angles;     // n * 1
  // 侧倾角，每个点有一个
  std::vector<double> side_tilt_angles;     // n * 1
  // 前倾角，每个点有一个
  std::vector<double> forward_tilt_angles;  // n * 1
  // 路径点的法向
  std::vector<double> normals;              // n * 3
  // 路径点的前进方向（切向）
  std::vector<double> tangents;             // n * 3
};

auto CptCollisionMap::prepareNrt() -> void {
  option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_PRINT_CMD_INFO;
  // option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;
  for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;

  CptCollisionMapParam cpt_collision_map_param;
  // find points number first
  for (const auto& cmd_param : cmdParams()) {
    auto c = controller();
    if (cmd_param.first == "option") {
      cpt_collision_map_param.option = int32Param(cmd_param.first);
    } else if (cmd_param.first == "tool_install") {
      int temp = int32Param(cmd_param.first);
      if (temp == 0) {
        cpt_collision_map_param.tool_install =
            WobjToolInstallMethod::EX_WOBJ_HAND_TOOL;
      } else if (temp == 1) {
        cpt_collision_map_param.tool_install =
            WobjToolInstallMethod::HAND_WOBJ_EX_TOOL;
      }
    } else if (cmd_param.first == "point_size") {
      cpt_collision_map_param.point_size = uint64Param(cmd_param.first);
    } else if (cmd_param.first == "resolution") {
      cpt_collision_map_param.resolution = uint64Param(cmd_param.first);
    }
  }
  sire::Size pSize = cpt_collision_map_param.point_size;
  cpt_collision_map_param.points.resize(pSize * 3, 0.0);
  cpt_collision_map_param.normals.resize(pSize * 3, 0.0);
  cpt_collision_map_param.tangents.resize(pSize * 3, 0.0);
  cpt_collision_map_param.tool_axis_angles.resize(pSize, 0.0);
  cpt_collision_map_param.side_tilt_angles.resize(pSize, 0.0);
  cpt_collision_map_param.forward_tilt_angles.resize(pSize, 0.0);
  // find points normals tangents forward/tilt_angles
  for (const auto& cmd_param : cmdParams()) {
    auto c = controller();
    if (cmd_param.first == "points") {
      auto points_mat = matrixParam(cmd_param.first);
      if (points_mat.size() == pSize * 3)
        std::copy(points_mat.begin(), points_mat.end(),
                  cpt_collision_map_param.points.begin());
      else
        THROW_FILE_LINE("parameter points size unmatch");

    } else if (cmd_param.first == "tool_angles") {
      auto tool_mat = matrixParam(cmd_param.first);
      if (tool_mat.size() == pSize)
        std::copy(tool_mat.begin(), tool_mat.end(),
                  cpt_collision_map_param.tool_axis_angles.begin());
      else
        THROW_FILE_LINE("parameter tilt_angles size unmatch");

    } else if (cmd_param.first == "side_angles") {
      auto side_mat = matrixParam(cmd_param.first);
      if (side_mat.size() == pSize)
        std::copy(side_mat.begin(), side_mat.end(),
                  cpt_collision_map_param.side_tilt_angles.begin());
      else
        THROW_FILE_LINE("parameter tilt_angles size unmatch");
    } else if (cmd_param.first == "forward_angles") {
      auto forward_mat = matrixParam(cmd_param.first);
      if (forward_mat.size() == pSize)
        std::copy(forward_mat.begin(), forward_mat.end(),
                  cpt_collision_map_param.forward_tilt_angles.begin());
      else
        THROW_FILE_LINE("parameter forward_angles size unmatch");
    } else if (cmd_param.first == "normals") {
      auto normal_mat = matrixParam(cmd_param.first);
      if (normal_mat.size() == pSize * 3)
        std::copy(normal_mat.begin(), normal_mat.end(),
                  cpt_collision_map_param.normals.begin());
      else
        THROW_FILE_LINE("parameter normals size unmatch");
    } else if (cmd_param.first == "tangents") {
      auto tangent_mat = matrixParam(cmd_param.first);
      if (tangent_mat.size() == pSize * 3)
        std::copy(tangent_mat.begin(), tangent_mat.end(),
                  cpt_collision_map_param.tangents.begin());
      else
        THROW_FILE_LINE("parameter tangents size unmatch");
    }
  }

  auto& cs = *controlServer();
  auto& interface =
      dynamic_cast<server::ProgramWebInterface&>(cs.interfacePool().at(0));
  // auto& camBackend = dynamic_cast<CamBackend&>(
  //     dynamic_cast<middleware::SireMiddleware&>(cs.middleWare())
  //         .simulatorModules()
  //         .at(0));
  //
  // camBackend.cptCollisionMap(
  //     WobjToolInstallMethod::EX_WOBJ_HAND_TOOL,
  //     cpt_collision_map_param.option, cpt_collision_map_param.resolution,
  //     cpt_collision_map_param.point_size,
  //     cpt_collision_map_param.points.data(),
  //     cpt_collision_map_param.tool_axis_angles.data(),
  //     cpt_collision_map_param.side_tilt_angles.data(),
  //     cpt_collision_map_param.forward_tilt_angles.data(),
  //     cpt_collision_map_param.normals.data(),
  //     cpt_collision_map_param.tangents.data());
  //
  // std::vector<std::pair<std::string, std::any>> out_param;
  //
  // out_param.push_back(std::make_pair<std::string, std::any>(
  //     "collision_map_result",
  //     nlohmann::json(camBackend.getCollisionMapResult())));
  // out_param.push_back(std::make_pair<std::string, std::any>(
  //     "collided_objects_result",
  //     nlohmann::json(camBackend.getCollidedObjectsResult())));
  // ret() = out_param;
}

auto CptCollisionMap::collectNrt() -> void {}

CptCollisionMap::CptCollisionMap(const std::string& name) {
  aris::core::fromXmlString(
      command(),
      "<Command name=\"cpt_collision_map\">"
      "	<GroupParam>"
      "		<Param name=\"tool_install\" default=\"0\"/>"
      "		<Param name=\"option\" default=\"0\"/>"
      "		<Param name=\"point_size\"/>"
      "		<Param name=\"resolution\"/>"
      "		<Param name=\"points\"/>"
      "		<Param name=\"tool_angles\"/>"
      "		<Param name=\"side_angles\"/>"
      "		<Param name=\"forward_angles\"/>"
      "		<Param name=\"normals\"/>"
      "		<Param name=\"tangents\"/>"
      "	</GroupParam>"
      "</Command>");
}

ARIS_REGISTRATION {
  aris::core::class_<CptCollisionMap>("CptCollisionMap")
      .inherit<aris::plan::Plan>();
}
}  // namespace sire::cam_backend