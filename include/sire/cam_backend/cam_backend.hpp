#ifndef CAM_BACKEND_H
#define CAM_BACKEND_H

#include "sire/collision/collided_objects_callback.hpp"
#include "sire/collision/geometry/collision_geometry.hpp"
#include <aris/core/expression_calculator.hpp>
#include <hpp/fcl/broadphase/broadphase_callbacks.h>
#include <hpp/fcl/broadphase/broadphase_collision_manager.h>
#include <hpp/fcl/broadphase/default_broadphase_callbacks.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/collision_object.h>
#include <map>
#include <string>

namespace sire::cam_backend {
using namespace std;
using namespace hpp;
/*
 * 之后用CAM_Backend取代CS，然后统一model
 *
 */
class CamBackend {
 public:
  /*
  option: compute Axis6, external axis, forward_angle, lateral_angle
  resolution: number of slot to divide range of angle can turn
  step: length to calculate data along tool path.
  需要具体的加工路径和加工面吗
  **/
  void cptCollisionMap(int option, aris::Size resolution, aris::Size pSzie,
                       double* points, double* tilt_angles,
                       double* forward_angles, double* normal, double* tangent);
  void init(string model_xml_path = ".", string collision_xml_path = ".");
  CamBackend();
  ~CamBackend();

 private:
  // ee_pe: EULER321
  void cptCollisionByEEPose(double* ee_pe,
                            collision::CollidedObjectsCallback& callback);
  struct Imp;
  unique_ptr<Imp> imp_;
};

}  // namespace sire::cam_backend
#endif