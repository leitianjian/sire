#include <array>

#include <gtest/gtest.h>

#include <aris/core/core.hpp>
#include <aris/dynamic/math_matrix.hpp>
#include <aris/dynamic/model.hpp>
#include <aris/dynamic/model_force.hpp>
#include <aris/dynamic/model_solver.hpp>

#include "sire/core/constants.hpp"
#include "sire/core/force_screw.hpp"
#include "sire/core/geometry/geometry_base.hpp"
#include "sire/core/sire_fixed_joint.hpp"

using namespace sire::geometry;
class MultiContactMassMatrixTest : public ::testing::Test {
 private:
  // Object 1 parameter with sphere foot
  const double planeMass = 3;
  const double planeX = 8, planeY = 8;
  const double planeZ = 0.1;
  // collide surface parameter
  const double surfaceMass = 6;
  const double surfaceX = 10, surfaceY = 10;
  const double surfaceZ = 2;
  // Sphere foot parameters
  const double sphereMass = 1;
  const double sphereRadius = 1;

  double plane_position_and_euler321[6]{0, 0, 0, 0, 0, 0};
  double link1_position_and_euler321[6]{planeX * 0.5, planeY * 0.5, 0, 0, 0, 0};
  double link2_position_and_euler321[6]{
      planeX * 0.5, -planeY * 0.5, 0, 0, 0, 0};
  double link3_position_and_euler321[6]{
      -planeX * 0.5, planeY * 0.5, 0, 0, 0, 0};
  double link4_position_and_euler321[6]{
      -planeX * 0.5, -planeY * 0.5, 0, 0, 0, 0};
  double surface_position_and_euler321[6]{0, 0, -2, 0, 0, 0};
  double plane_inertia_vector[10]{0};
  double link1_inertia_vector[10]{0};
  double link2_inertia_vector[10]{0};
  double link3_inertia_vector[10]{0};
  double link4_inertia_vector[10]{0};
  double surface_inertia_vector[10]{0};

  const double joint1_position[3]{planeX * 0.5, planeY * 0.5, 0.0};
  const double joint1_axis[3]{0.0, 0.0, 1.0};
  const double joint2_position[3]{planeX * 0.5, -planeY * 0.5, 0.0};
  const double joint2_axis[3]{0.0, 0.0, 1.0};
  const double joint3_position[3]{-planeX * 0.5, planeY * 0.5, 0.0};
  const double joint3_axis[3]{0.0, 0.0, 1.0};
  const double joint4_position[3]{-planeX * 0.5, -planeY * 0.5, 0.0};
  const double joint4_axis[3]{0.0, 0.0, 1.0};

  aris::dynamic::Model table_model_;
  double leg_accel_[4]{0};
  const int num_leg_{4};
  std::vector<std::array<double, 3>> ori_vec{0};
  std::vector<std::array<double, 16>> T_vec{0};

 protected:
  // iv -> inertia vector;
  auto calcSphereInertia(double mass, double radius, double* iv) -> void {
    iv[0] = mass;
    double ixyz = 0.4 * mass * radius * radius;
    iv[4] = iv[5] = iv[6] = ixyz;
    return;
  }
  auto calcBoxInertia(double mass, double x, double y, double z,
                      double* iv) -> void {
    iv[0] = mass;
    iv[4] = mass * (y * y + z * z) / 12;  // ix
    iv[5] = mass * (x * x + z * z) / 12;  // iy
    iv[6] = mass * (x * x + y * y) / 12;  // iz
    return;
  }
  // 使用类似桌子的模型，在每个桌子脚上放置一个球形的脚取代长方体脚，方便建模。
  //
  // 计算这个模型在四个角上给力时候对应的加速度以及对其他四个脚的影响。
  //    3 ------------- 1
  //    |      ^  y     |
  //    |      |        |  8m
  //    |      ---> x   |
  //    |               |  with a radius 1m sphere
  //    |               |    at every corner.
  //    4 ------------- 2
  //           8m
  auto initTableModel() -> void {
    // auto xmlpath = std::filesystem::absolute(".");  // 获取当前工程所在的路径
    // const std::string xmlfile = "table_model.xml";
    // xmlpath = xmlpath / xmlfile;
    // aris::core::fromXmlFile(table_model_, xmlpath);
    double gravityAs[6]{0, 0, 0, 0, 0, 0};
    calcBoxInertia(planeMass, planeX, planeY, planeZ, plane_inertia_vector);
    calcSphereInertia(sphereMass, sphereRadius, link1_inertia_vector);
    calcSphereInertia(sphereMass, sphereRadius, link2_inertia_vector);
    calcSphereInertia(sphereMass, sphereRadius, link3_inertia_vector);
    calcSphereInertia(sphereMass, sphereRadius, link4_inertia_vector);
    calcBoxInertia(surfaceMass, surfaceX, surfaceY, surfaceZ,
                   surface_inertia_vector);
    table_model_.environment().setGravity(gravityAs);
    table_model_.ground().addMarker("ground_mkr");
    auto& plane = table_model_.addPartByPe(plane_position_and_euler321, "313",
                                           plane_inertia_vector);
    auto& link1 = table_model_.addPartByPe(link1_position_and_euler321, "313",
                                           link1_inertia_vector);
    auto& link2 = table_model_.addPartByPe(link2_position_and_euler321, "313",
                                           link2_inertia_vector);
    auto& link3 = table_model_.addPartByPe(link3_position_and_euler321, "313",
                                           link3_inertia_vector);
    auto& link4 = table_model_.addPartByPe(link4_position_and_euler321, "313",
                                           link4_inertia_vector);
    auto& surface = table_model_.addPartByPe(surface_position_and_euler321,
                                             "313", surface_inertia_vector);
    surface.addMarker("surface_mkr");
    // auto& joint1 = table_model_.addRevoluteJoint(link1, plane,
    // joint1_position,
    //                                              joint1_axis);
    // auto& joint2 = table_model_.addRevoluteJoint(link2, plane,
    // joint2_position,
    //                                              joint2_axis);
    // auto& joint3 = table_model_.addRevoluteJoint(link3, plane,
    // joint3_position,
    //                                              joint3_axis);
    // auto& joint4 = table_model_.addRevoluteJoint(link4, plane,
    // joint4_position,
    //                                              joint4_axis);
    sire::core::FixedJoint::add2ModelRelative(&table_model_, link1, plane,
                                      joint1_position);
    sire::core::FixedJoint::add2ModelRelative(&table_model_, link2, plane,
                                      joint2_position);
    sire::core::FixedJoint::add2ModelRelative(&table_model_, link3, plane,
                                      joint3_position);
    sire::core::FixedJoint::add2ModelRelative(&table_model_, link4, plane,
                                      joint4_position);

    auto& force11 = table_model_.forcePool().add<aris::dynamic::GeneralForce>(
        "gf11", &link1.markerPool().at(0),
        &table_model_.ground().markerPool().at(0));
    auto& force12 = table_model_.forcePool().add<aris::dynamic::GeneralForce>(
        "gf12", &surface.markerPool().at(0),
        &table_model_.ground().markerPool().at(0));
    auto& force2 = table_model_.forcePool().add<aris::dynamic::GeneralForce>(
        "gf2", &link2.markerPool().at(0), &surface.markerPool().at(0));
    auto& force3 = table_model_.forcePool().add<aris::dynamic::GeneralForce>(
        "gf3", &link3.markerPool().at(0), &surface.markerPool().at(0));
    auto& force4 = table_model_.forcePool().add<aris::dynamic::GeneralForce>(
        "gf4", &link4.markerPool().at(0), &surface.markerPool().at(0));

    table_model_.solverPool().add<aris::dynamic::InverseKinematicSolver>();
    table_model_.solverPool().add<aris::dynamic::ForwardKinematicSolver>();
    table_model_.solverPool().add<aris::dynamic::InverseDynamicSolver>();
    table_model_.solverPool().add<aris::dynamic::ForwardDynamicSolver>();
    std::cout << aris::core::toXmlString(table_model_) << std::endl;
    table_model_.init();

    T_vec.resize(num_leg_);
    ori_vec.resize(num_leg_);
    ori_vec[0] = {planeX * 0.5, planeY * 0.5, -sphereRadius};
    ori_vec[1] = {planeX * 0.5, -planeY * 0.5, -sphereRadius};
    ori_vec[2] = {-planeX * 0.5, planeY * 0.5, -sphereRadius};
    ori_vec[3] = {-planeX * 0.5, -planeY * 0.5, -sphereRadius};
    double z_vec[3]{0, 0, 1};
    double x_vec[3]{1, 0, 0};

    for (int i = 0; i < num_leg_; ++i) {
      aris::dynamic::s_sov_axes2pm(ori_vec[i].data(), z_vec, x_vec,
                                   T_vec.at(i).data(), "zx");
    }
  }

  // 用于解决surface求解的加速度旋量对不上的问题
  auto initSurfaceModel() -> void {
    double gravityAs[6]{0, 0, 0, 0, 0, 0};
    calcBoxInertia(planeMass, planeX, planeY, planeZ, plane_inertia_vector);
    calcSphereInertia(sphereMass, sphereRadius, link1_inertia_vector);
    calcSphereInertia(sphereMass, sphereRadius, link2_inertia_vector);
    calcSphereInertia(sphereMass, sphereRadius, link3_inertia_vector);
    calcSphereInertia(sphereMass, sphereRadius, link4_inertia_vector);
    calcBoxInertia(surfaceMass, surfaceX, surfaceY, surfaceZ,
                   surface_inertia_vector);
    table_model_.environment().setGravity(gravityAs);
    double pm[16]{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, -2, 0, 0, 0, 1};
    table_model_.ground().addMarker("ground_mkr", pm);
    auto& surface = table_model_.addPartByPe(surface_position_and_euler321,
                                             "313", surface_inertia_vector);
    surface.addMarker("surface_mkr");
  }
  auto applyLegForce2LegAccel(double* forces, double* accel) -> void {
    double fs[6];
    std::array<double, 3> fce{10, 0, 0};
    sire::core::screw::s_fpm2fs(fce.data(), T_vec[0].data(), fs);
    dynamic_cast<aris::dynamic::GeneralForce&>(table_model_.forcePool().at(0))
        .setFce(fs);
    dynamic_cast<aris::dynamic::GeneralForce&>(table_model_.forcePool().at(1))
        .setFce(fs);
    aris::dynamic::dsp(1, 6, fs);
    if (table_model_.forwardDynamics()) {
      std::cout << "forward dynamic failed" << std::endl;
    }
    double as_buffer[6]{0};
    double vs[6]{0};
    std::vector<std::array<double, 3>> accel_buffer{0};
    accel_buffer.resize(num_leg_ * 2);
    for (sire::Size i = 0; i < num_leg_; ++i) {
      auto& part = table_model_.partPool()[i + 2];
      part.getAs(as_buffer);
      aris::dynamic::dsp(1, 6, as_buffer);
      aris::dynamic::s_as2ap(vs, as_buffer, ori_vec[i].data(),
                             accel_buffer[i].data());
      accel[i] = accel_buffer[i][2];
    }
    auto& surface = table_model_.partPool()[num_leg_ + 2];
    surface.getAs(as_buffer);
    aris::dynamic::dsp(1, 6, as_buffer);
    for (sire::Size i = 0; i < num_leg_; ++i) {
      aris::dynamic::s_as2ap(vs, as_buffer, ori_vec[i].data(),
                             accel_buffer[i + 4].data());
      accel[i + 4] = accel_buffer[i + 4][2];
    }
    aris::dynamic::dsp(num_leg_ * 2, 3, &accel_buffer[0][0]);
    for (int i = 0; i < num_leg_ * 2; ++i) {
      for (int j = 0; j < 3; ++j) {
        std::cout << 10 / accel_buffer[i][j] << " ";
      }
      std::cout << std::endl;
    }
  }
};

// 用于解决surface求解的加速度旋量对不上的问题
class SurfaceContactMassMatrixTest : public ::testing::Test {
 private:
  // collide surface parameter
  const double surfaceMass = 6;
  const double surfaceX = 10, surfaceY = 10;
  const double surfaceZ = 2;

  double surface_position_and_euler321[6]{0, 0, -2, 0, 0, 0};
  double plane_inertia_vector[10]{0};
  double link1_inertia_vector[10]{0};
  double link2_inertia_vector[10]{0};
  double link3_inertia_vector[10]{0};
  double link4_inertia_vector[10]{0};
  double surface_inertia_vector[10]{0};

  aris::dynamic::Model surface_model_;
  double leg_accel_[4]{0};
  const int num_leg_{4};
  std::vector<std::array<double, 3>> ori_vec{0};
  std::vector<std::array<double, 16>> T_vec{0};

 protected:
  auto calcBoxInertia(double mass, double x, double y, double z,
                      double* iv) -> void {
    iv[0] = mass;
    iv[4] = mass * (y * y + z * z) / 12;  // ix
    iv[5] = mass * (x * x + z * z) / 12;  // iy
    iv[6] = mass * (x * x + y * y) / 12;  // iz
    return;
  }
  // 使用类似桌子的模型，在每个桌子脚上放置一个球形的脚取代长方体脚，方便建模。
  //
  // 计算这个模型在四个角上给力时候对应的加速度以及对其他四个脚的影响。
  //    3 ------------- 1
  //    |      ^  y     |
  //    |      |        |  8m
  //    |      ---> x   |
  //    |               |  with a radius 1m sphere
  //    |               |    at every corner.
  //    4 ------------- 2
  //           8m
  auto initSurfaceModel() -> void {
    // auto xmlpath = std::filesystem::absolute(".");  // 获取当前工程所在的路径
    // const std::string xmlfile = "table_model.xml";
    // xmlpath = xmlpath / xmlfile;
    // aris::core::fromXmlFile(table_model_, xmlpath);
    double gravityAs[6]{0, 0, 0, 0, 0, 0};
    calcBoxInertia(surfaceMass, surfaceX, surfaceY, surfaceZ,
                   surface_inertia_vector);
    aris::dynamic::dsp(1, 10, surface_inertia_vector);
    surface_model_.environment().setGravity(gravityAs);
    surface_model_.ground().addMarker("ground_mkr");
    double surface_position_and_euler3211[6]{0, 0, -2, 0, 0, 0};

    auto& surface = surface_model_.addPartByPe(surface_position_and_euler3211,
                                               "313", surface_inertia_vector);
    surface.addMarker("surface_mkr");

    auto& force12 = surface_model_.forcePool().add<aris::dynamic::GeneralForce>(
        "gf12", &surface.markerPool().at(0),
        &surface_model_.ground().markerPool().at(0));

    surface_model_.solverPool().add<aris::dynamic::InverseKinematicSolver>();
    surface_model_.solverPool().add<aris::dynamic::ForwardKinematicSolver>();
    surface_model_.solverPool().add<aris::dynamic::InverseDynamicSolver>();
    surface_model_.solverPool().add<aris::dynamic::ForwardDynamicSolver>();
    std::cout << aris::core::toXmlString(surface_model_) << std::endl;
    surface_model_.init();

    T_vec.resize(num_leg_);
    ori_vec.resize(num_leg_);
    ori_vec[0] = {4, 4, -1};
    ori_vec[1] = {4, -4, -1};
    ori_vec[2] = {-4, 4, -1};
    ori_vec[3] = {-4, -4, -1};
    double z_vec[3]{0, 0, 1};
    double x_vec[3]{1, 0, 0};

    for (int i = 0; i < num_leg_; ++i) {
      aris::dynamic::s_sov_axes2pm(ori_vec[i].data(), z_vec, x_vec,
                                   T_vec.at(i).data(), "zx");
    }
  }

  auto applyLegForce2LegAccel() -> void {
    double fs[6];
    std::array<double, 3> fce{10, 0, 0};
    sire::core::screw::s_fpm2fs(fce.data(), T_vec[0].data(), fs);
    dynamic_cast<aris::dynamic::GeneralForce&>(surface_model_.forcePool().at(0))
        .setFce(fs);
    aris::dynamic::dsp(1, 6, fs);
    if (surface_model_.forwardDynamics()) {
      std::cout << "forward dynamic failed" << std::endl;
    }
    double as_buffer[6]{0};
    double vs[6]{0};
    std::vector<std::array<double, 3>> accel_buffer{0};
    accel_buffer.resize(num_leg_);

    auto& surface = surface_model_.partPool()[1];
    surface.getAs(as_buffer);
    aris::dynamic::dsp(1, 6, as_buffer);
    double pm[16]{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, -2, 0, 0, 0, 1};
    double r_as[6]{0}, r_vs[6]{0}, to_as[6]{0};
    aris::dynamic::s_as2as(pm, r_vs, r_as, r_vs, as_buffer, to_as);
    aris::dynamic::dsp(1, 6, to_as);
    for (sire::Size i = 0; i < num_leg_; ++i) {
      aris::dynamic::s_as2ap(vs, as_buffer, ori_vec[i].data(),
                             accel_buffer[i].data());
    }
    aris::dynamic::dsp(num_leg_, 3, &accel_buffer[0][0]);
    for (int i = 0; i < num_leg_; ++i) {
      for (int j = 0; j < 3; ++j) {
        std::cout << 10 / accel_buffer[i][j] << " ";
      }
      std::cout << std::endl;
    }
  }
};

TEST_F(MultiContactMassMatrixTest, CalcContactMassMatrix) {
  initTableModel();
  // for (int i = 0; i < 20; ++i) {
  double i = 10;
  double fces[4]{(double)i, 0, 0, 0};
  double accel[8]{0};
  applyLegForce2LegAccel(fces, accel);

  std::cout << i / accel[0] << " " << i / accel[1] << " " << i / accel[2] << " "
            << i / accel[3] << std::endl;
  std::cout << accel[0] << " " << accel[1] << " " << accel[2] << " " << accel[3]
            << std::endl;
  std::cout << i / accel[4] << " " << i / accel[5] << " " << i / accel[6] << " "
            << i / accel[7] << std::endl;
  std::cout << accel[4] << " " << accel[5] << " " << accel[6] << " " << accel[7]
            << std::endl;
  std::cout << std::endl;
  // }
}

TEST_F(SurfaceContactMassMatrixTest, SurfaceContactMassMatrix) {
  initSurfaceModel();
  // for (int i = 0; i < 20; ++i) {

  applyLegForce2LegAccel();
  // }
}
