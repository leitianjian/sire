#ifndef SIRE_CAM_BACKEND_HPP_
#define SIRE_CAM_BACKEND_HPP_

#include <map>
#include <string>

#include <sire_lib_export.h>

#include <aris/core/expression_calculator.hpp>
#include <aris/dynamic/model.hpp>

#include "sire/core/constants.hpp"
#include "sire/core/module_base.hpp"
#include "sire/physics/physics_engine.hpp"
#include "sire/physics/collision/collided_objects_callback.hpp"

namespace sire::cam_backend {
using namespace std;
using namespace hpp;

// 工具工件安装方式
// EX_WOBJ_HAND_TOOL: 正常安装方式，末端工具，固定工件
// HAND_WOBJ_EX_TOOL: 末端（手持）工件，固定工具
enum class WobjToolInstallMethod {
  EX_WOBJ_HAND_TOOL,
  HAND_WOBJ_EX_TOOL,
};

// 之后用CAM_Backend取代CS，然后统一model
class SIRE_API CamBackend : public core::SireModuleBase {
 public:
  // @param[in] cpt_option  compute collision map option with two case.
  // AxisA6, tilt angle.
  //
  // @param[in] resolution  number of slot to divide range of angle can turn
  //
  // @param[in] pSize  number of tool path points
  //
  // @param[in] points  tool path point cartesian coordinate value
  //  with size 3 * pSize
  //
  // @param[in] tool_axis_angles  angles rotate about tool axis
  //
  // @param[in] side_tilt_angles  Additional tilt angle in the plane
  // perpendicular to the tool motion
  //
  // @param[in] forward_tilt_angles  Additional tilt angle in the direction of
  // tool motion
  //
  // @param[in] normal  number of tool path points
  // @param[in] tangent  number of tool path points
  auto cptCollisionMap(
      WobjToolInstallMethod install_method, int option, sire::Size resolution,
      sire::Size pSzie, double* points, double* tool_axis_angles,
      double* side_tilt_angles, double* forward_tilt_angles, double* normal,
      double* tangent, std::vector<bool>& collision_result,
      vector<set<physics::CollisionObjectsPair>>& collided_objects_result)
      -> void;

  // @param[in] cpt_option  compute collision map option with two case.
  // AxisA6, tilt angle.
  //
  // @param[in] resolution  number of slot to divide range of angle can turn
  //
  // @param[in] pSize  number of tool path points
  //
  // @param[in] points_pm  tool path points position matrix
  //  with size 16 * pSize
  //
  // @param[in] side_tilt_angles  Additional tilt angle in the plane
  // perpendicular to the tool motion
  //
  // @param[in] forward_tilt_angles  Additional tilt angle in the direction of
  // tool motion
  auto cptCollisionMap(
      WobjToolInstallMethod install_method, int cpt_option,
      sire::Size resolution, sire::Size pSzie, double* points_pm,
      double* tool_axis_angles, double* side_tilt_angles,
      double* forward_tilt_angles, std::vector<bool>& collision_result,
      vector<set<physics::CollisionObjectsPair>>& collided_objects_resultf)
      -> void;
  auto cptCollisionByEEPose(
      double* ee_pe, physics::collision::CollidedObjectsCallback& callback)
      -> void;
  auto physicsEngine() -> physics::PhysicsEngine*;
  // auto getCollisionDetection() -> physics::collision::CollisionDetection&;
  // auto resetCollisionDetection(physics::collision::CollisionDetection*
  // engine)
  //     -> void;
  // auto getCollisionMapResult() -> const vector<bool>&;
  // auto getCollidedObjectsResult()
  // ->const vector<set<physics::CollisionObjectsPair>>&;
  // initial CAM backend by two config file
  auto doInit(double* init_motor_p) -> void;
  auto init(double* init_motor_p) -> void;
  auto init(string model_config_path, string engine_config_path,
            double* init_motor_p) -> void;
  auto init(aris::dynamic::Model* model_ptr, double* init_motor_p) -> void;
  auto init(physics::PhysicsEngine* engine_ptr, double* init_motor_p) -> void;
  // initial CAM backend by control server default
  // auto init() -> void;
  CamBackend();
  ~CamBackend();
  SIRE_DECLARE_MOVE_CTOR(CamBackend);

 private:
  // @param[in] ee_pe  end effector pose with [position, EULER321] in double[6]

  // @param[in] install_method  wobj/tool install method
  //
  // @param[in] cpt_option  compute option of which collision map to compute
  // 0 => AxisA6
  // 1 => side_tilt_angle
  // reference spurtCAM 冗余轴优化界面选项卡
  //
  // @param[in] angle  (radians) The angle of axisA6/side/forward.
  //
  // @param[in] tool_path_point_pm  tool path point position matrix 4 * 4
  // using normal vector of processing plane and direction of tool motion
  // to construct cartesian coordinate.
  //
  // @param[out] target_ee_pose
  auto cptEEPose(WobjToolInstallMethod install_method, int cpt_option,
                 double angle, double* tool_path_point_pm,
                 double tool_axis_angle, double side_tilt_angle,
                 double forward_tilt_angle, double* target_ee_pe) -> void;

  struct Imp;
  unique_ptr<Imp> imp_;
};

auto mapAngleToSymRange(double angle, double range) -> double;
auto vectorCross(double* in_1, double* in_2, double* out) -> void;
auto vectorNormalize(sire::Size n, double* in) -> void;
auto xyz2pm(double* x, double* y, double* z, double* out) -> void;

auto tiltAngle2pm(double side_tilt_angle, double forward_tilt_angle,
                  double* out) -> void;

}  // namespace sire::cam_backend
#endif