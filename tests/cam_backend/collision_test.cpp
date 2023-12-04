#include <iostream>

#include <gtest/gtest.h>

#include <aris.hpp>

#include "sire/cam_backend/cam_backend.hpp"
#include "sire/core/constants.hpp"
using namespace aris::dynamic;
using namespace sire::cam_backend;
GTEST_TEST(CAMBackend, collisionTest) {
  const double PI = 3.1415926535897932384626433;
  aris::dynamic::Model model;
  auto config_path =
      std::filesystem::absolute(".");  // 获取当前可执行文件所在的路径
  const std::string model_config_name = "cam_model.xml";
  auto model_config_path = config_path / model_config_name;
  aris::core::fromXmlFile(model, model_config_path);
  model.init();

  sire::physics::PhysicsEngine engine;
  const std::string physics_engine_name = "physics_engine.xml";
  auto physics_engine_path = config_path / physics_engine_name;
  aris::core::fromXmlFile(engine, physics_engine_path);

  // 这个点有问题，
  double ee_pe[6]{0.12182754516601563,   -0.024801206588745118,
                  0.65125018310546878,   4.8339357759369026,
                  -0.020351199375015640, 6.2667637480970368};
  model.setOutputPos(ee_pe);

  if (model.inverseKinematics())
    throw std::runtime_error("inverse kinematic failed");

  int partSize = model.partPool().size();

  std::vector<double> part_pq(partSize * 7);
  for (int i = 0; i < partSize; ++i) {
    model.partPool().at(i).getPq(part_pq.data() + i * 7);
    aris::dynamic::dsp(1, 7, part_pq.data() + i * 7);
  }

  sire::physics::collision::CollidedObjectsCallback callback(
      &engine.collisionFilter());
  engine.collisionDetection().updateLocation(part_pq.data());
  engine.collisionDetection().collidedObjects(callback);
}