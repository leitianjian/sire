#include "sire/physics/physics_engine.hpp"

#include <memory>

#include <coal/data_types.h>
#include <eigen3/Eigen/Eigenvalues>
#include <gtest/gtest.h>

#include <aris/dynamic/math_matrix.hpp>

#include "sire/core/constants.hpp"
#include "sire/core/force_screw.hpp"
#include "sire/core/geometry/geometry_base.hpp"
#include "sire/core/sire_assert.hpp"
#include "sire/core/sire_fixed_joint.hpp"
#include "sire/physics/collision/collision_filter.hpp"
#include "sire/physics/collision/penetration_as_point_pair_callback.hpp"
#include "sire/physics/common/penetration_as_point_pair.hpp"
#include "sire/physics/contact/avg_force_contact_solver.hpp"
#include "sire/physics/geometry/sphere_collision_geometry.hpp"

using namespace sire::geometry;
using namespace sire::physics;
using namespace coal;
using namespace std;
using sire::physics::common::PenetrationAsPointPair;
class SimplePenetrationTest : public ::testing::Test {
 protected:
  // Moves the dynamic sphere to either a penetrating or non-penetrating
  // position. The sphere is indicated by its `id` which belongs to the given
  // `source_id`. If `is_colliding` is true, the sphere is placed in a colliding
  // configuration.
  //
  // r = 0.5
  // Non-colliding state
  //       y = 0       x = free_x_ = 1.25
  //        │          │
  //       *│*         o o
  //    *   │   *   o       o
  //   *    │    * o         o
  // ──*────┼────*─o─────────o───────── x
  //   *    │    * o         o
  //    *   │   *   o       o
  //       *│*         o o
  //
  // Colliding state
  //       y = 0   x = colliding_x_ = 0.75
  //        │      │
  //       *│*    o o
  //    *   │  o*      o
  //   *    │ o  *      o
  // ──*────┼─o──*──────o────────────── x
  //   *    │ o  *      o
  //    *   │  o*      o
  //       *│*    o o

  // Updates a pose in X_WGs_ to be colliding or non-colliding. Then updates the
  // position of all dynamic geometries.
  void moveDynamicSphere(GeometryId id, bool is_colliding,
                         PhysicsEngine* engine = nullptr) {
    engine = (engine == nullptr) ? engine_.get() : engine;

    SIRE_DEMAND(engine->numGeometries() == 2);

    const double x_pos = is_colliding ? colliding_x_ : free_x_;
    // set dynamic sphere position
    X_WGs_[7] = x_pos;

    engine->collisionDetection().updateLocation(X_WGs_);
  }

  // Compute penetration and confirm that a single penetration with the
  // expected1 properties was found. Provide the geometry ids of the sphere
  // located at the origin and the sphere positioned to be in collision.
  void expectPenetration(GeometryId origin_sphere, GeometryId colliding_sphere,
                         PhysicsEngine* engine = nullptr) {
    std::vector<PenetrationAsPointPair> penetration_results =
        engine->computePointPairPenetration();
    ASSERT_EQ(penetration_results.size(), 1);
    const PenetrationAsPointPair& penetration = penetration_results[0];

    // There are no guarantees as to the ordering of which element is A and
    // which is B. This test enforces an order for validation.

    // First confirm membership
    EXPECT_TRUE((penetration.id_A == origin_sphere &&
                 penetration.id_B == colliding_sphere) ||
                (penetration.id_A == colliding_sphere &&
                 penetration.id_B == origin_sphere));

    // Assume A => origin_sphere and b => colliding_sphere1
    // NOTE: In this current version, penetration is only reported in double.
    PenetrationAsPointPair expected;
    // This implicitly tests the *ordering* of the two reported ids. It must
    // always be in *this* order.
    bool origin_is_A = origin_sphere < colliding_sphere;
    expected.id_A = origin_is_A ? origin_sphere : colliding_sphere;
    expected.id_B = origin_is_A ? colliding_sphere : origin_sphere;
    expected.depth = 2 * radius_ - colliding_x_;
    // Contact point on the origin_sphere.
    Vec3s p_WCo{radius_, 0, 0};
    // Contact point on the colliding_sphere1.
    Vec3s p_WCc{colliding_x_ - radius_, 0, 0};
    expected.p_WCa = origin_is_A ? p_WCo : p_WCc;
    expected.p_WCb = origin_is_A ? p_WCc : p_WCo;
    Vec3s norm_into_B = Vec3s::UnitX();
    expected.nhat_AB_W = origin_is_A ? norm_into_B : -norm_into_B;

    // Check penetration result and expection result
    EXPECT_EQ(penetration.id_A, expected.id_A);
    EXPECT_EQ(penetration.id_B, expected.id_B);
    EXPECT_EQ(penetration.depth, expected.depth);
    EXPECT_TRUE(aris::dynamic::s_is_equal(3, penetration.p_WCa.data(),
                                          expected.p_WCa.data(), 1e-13));
    EXPECT_TRUE(aris::dynamic::s_is_equal(3, penetration.p_WCb.data(),
                                          expected.p_WCb.data(), 1e-13));
    EXPECT_TRUE(aris::dynamic::s_is_equal(3, penetration.nhat_AB_W.data(),
                                          expected.nhat_AB_W.data(), 1e-13));
  }

  // The two spheres collides, but are ignored due to the setting in the
  // collision filter.
  auto expectIgnoredPenetration(GeometryId origin_sphere,
                                GeometryId colliding_sphere,
                                PhysicsEngine* engine) -> void {
    std::vector<PenetrationAsPointPair> penetration_results =
        engine->computePointPairPenetration();
    EXPECT_EQ(penetration_results.size(), 0);
  }

  // Compute penetration and confirm that none were found.
  void expectNoPenetration(GeometryId origin_sphere,
                           GeometryId colliding_sphere, PhysicsEngine* engine) {
    std::vector<PenetrationAsPointPair> penetration_results =
        engine->computePointPairPenetration();
    EXPECT_EQ(penetration_results.size(), 0);
  }

  std::unique_ptr<PhysicsEngine> engine_{std::make_unique<PhysicsEngine>()};
  collision::CollisionDetection* collision_engine_;

  double X_WGs_[14]{
      0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1,
  };
  const double radius_{0.5};
  const double free_x_{2.5 * radius_};
  const double colliding_x_{1.5 * radius_};
};

class MultipleObjectsPenetrationTest : public ::testing::Test {
 protected:
  // Moves the dynamic sphere to either a penetrating or non-penetrating
  // position. The sphere is indicated by its `id` which belongs to the given
  // `source_id`. If `is_colliding` is true, the sphere is placed in a colliding
  // configuration.
  //
  // r = 0.5
  // Non-colliding state
  //
  //        y         x = free_x_ = 1.25
  //  2     │           |
  //       o│o          |
  //    o   │   o       |
  //   o    │    o      |
  //   o────┼────o──────|────────────  y = free_y_ = 1.25
  //   o    │    o      |
  //    o   │   o       |
  //       o│o          |
  //        │           │
  //       *│*         o o
  //    *   │   *   o       o
  //   *    │    * o         o
  // ──*────┼────*─o─────────o───────── x
  //   *    │    * o         o
  //    *   │   *   o       o
  //  0    *│*      1  o o
  //
  // Dynamic sphere colliding with static sphere state
  //
  //        y     x = colliding_x_ = 0.75
  //        │      |
  //  2    o│o     |
  //    o   │   o  |
  //   o    │    o |
  //   o────┼────o─|────────────  y = colliding_y_ = 0.75
  //   o    │    o |
  //    o  *│*  o o│o
  //    *  o│o o*  │   o
  //   *    │ o  * │    o
  // ──*────┼─o──*─│────o────────────── x
  //   *    │ o  * │    o
  //    *   │  o*  │   o
  //   0   *│*    o│o   1
  //
  // All Colliding with each other state
  //
  //            x = colliding_x1_ = 0.375
  //        y   | x = colliding_x_ = 0.75
  //        │ 2 |  │
  //        │  o│o │
  //        o   │  │o
  //       o│   │  │ o
  //     ──o┼───┼──┼─o──────────  y = colliding_y_ = 0.5
  //       o│   │  │ o
  //       *o*  │ o│o
  //    *   │  o*o │   o
  //   *    │ o  * │    o
  // ──*────┼─o──*─│────o────────────── x
  //   *    │ o  * │    o
  //    *   │  o*  │   o
  //   0   *│*    o│o   1
  //
  //
  enum class CollisionType { NON_COLLISION, TWO_COLLISION, ALL_COLLISION };

  // Updates a pose in X_WGs_ to be colliding or non-colliding. Then updates the
  // position of all dynamic geometries.
  void moveDynamicSphere(CollisionType collision_type,
                         PhysicsEngine* engine = nullptr) {
    engine = (engine == nullptr) ? engine_.get() : engine;

    SIRE_DEMAND(engine->numGeometries() == 3);

    double x1_pos{0};
    double x2_pos{0};
    double y2_pos{0};
    switch (collision_type) {
      case CollisionType::NON_COLLISION:
        x1_pos = free_x1_;
        x2_pos = free_x2_;
        y2_pos = free_y_;
        break;
      case CollisionType::TWO_COLLISION:
        x1_pos = colliding_x1_;
        x2_pos = free_x2_;
        y2_pos = colliding_y1_;
        break;
      case CollisionType::ALL_COLLISION:
        x1_pos = colliding_x1_;
        x2_pos = colliding_x2_;
        y2_pos = colliding_y2_;
        break;
    }
    // set dynamic sphere position
    X_WGs_[7] = x1_pos;
    X_WGs_[14] = x2_pos;
    X_WGs_[15] = y2_pos;

    engine->collisionDetection().updateLocation(X_WGs_);
  }

  // The two spheres collides, but are ignored due to the setting in the
  // collision filter.
  auto expectIgnoredPenetration(GeometryId origin_sphere,
                                GeometryId colliding_sphere,
                                PhysicsEngine* engine) -> void {
    std::vector<PenetrationAsPointPair> penetration_results =
        engine->computePointPairPenetration();
    EXPECT_EQ(penetration_results.size(), 0);
  }

  // Compute penetration and confirm that none were found.
  void expectNoPenetration(PhysicsEngine* engine) {
    std::vector<PenetrationAsPointPair> penetration_results =
        engine->computePointPairPenetration();
    EXPECT_EQ(penetration_results.size(), 0);
  }

  // Compute penetration and confirm that a single penetration with the
  // expected1 properties was found. Provide the geometry ids of the sphere
  // located at the origin and the sphere positioned to be in collision.
  void expectTwoPenetration(GeometryId origin_sphere,
                            GeometryId colliding_sphere1,
                            GeometryId colliding_sphere2,
                            PhysicsEngine* engine = nullptr) {
    std::vector<PenetrationAsPointPair> penetrations =
        engine->computePointPairPenetration();
    ASSERT_EQ(penetrations.size(), 2);
    // check for two penetration order
    PenetrationAsPointPair* colliding_0_1_result;
    PenetrationAsPointPair* colliding_0_2_result;
    if (penetrations[0].id_B == 1) {
      colliding_0_1_result = &penetrations[0];
      colliding_0_2_result = &penetrations[1];
    } else {
      colliding_0_1_result = &penetrations[1];
      colliding_0_2_result = &penetrations[0];
    }

    // The order of colliding sphere is guaranteed,
    // see the picture of test case above

    // Setting expected penetration result of geometry 0 and 1
    PenetrationAsPointPair expected1;
    expected1.id_A = origin_sphere;
    expected1.id_B = colliding_sphere1;
    expected1.depth = 2 * radius_ - colliding_x1_;
    // Contact point on the origin_sphere.
    Vec3s p_WCo1{radius_, 0, 0};
    // Contact point on the colliding_sphere1.
    Vec3s p_WCc1{colliding_x1_ - radius_, 0, 0};
    expected1.p_WCa = p_WCo1;
    expected1.p_WCb = p_WCc1;
    Vec3s norm_into_B = Vec3s::UnitX();
    expected1.nhat_AB_W = norm_into_B;

    // Setting expected penetration result of geometry 0 and 2
    PenetrationAsPointPair expected2;
    expected2.id_A = origin_sphere;
    expected2.id_B = colliding_sphere2;
    expected2.depth = 2 * radius_ - colliding_y1_;
    // Contact point on the origin_sphere.
    Vec3s p_WCo2{0, radius_, 0};
    // Contact point on the colliding_sphere2.
    Vec3s p_WCc2{0, colliding_x1_ - radius_, 0};
    expected2.p_WCa = p_WCo2;
    expected2.p_WCb = p_WCc2;
    Vec3s norm_into_C = Vec3s::UnitY();
    expected2.nhat_AB_W = norm_into_C;

    // Check penetration result and expection result of colliding 0 and 1
    EXPECT_EQ(colliding_0_1_result->id_A, expected1.id_A);
    EXPECT_EQ(colliding_0_1_result->id_B, expected1.id_B);
    EXPECT_EQ(colliding_0_1_result->depth, expected1.depth);
    EXPECT_TRUE(aris::dynamic::s_is_equal(3, colliding_0_1_result->p_WCa.data(),
                                          expected1.p_WCa.data(), 1e-13));
    EXPECT_TRUE(aris::dynamic::s_is_equal(3, colliding_0_1_result->p_WCb.data(),
                                          expected1.p_WCb.data(), 1e-13));
    EXPECT_TRUE(
        aris::dynamic::s_is_equal(3, colliding_0_1_result->nhat_AB_W.data(),
                                  expected1.nhat_AB_W.data(), 1e-13));

    // Check penetration result and expection result of colliding 0 and 2
    EXPECT_EQ(colliding_0_2_result->id_A, expected2.id_A);
    EXPECT_EQ(colliding_0_2_result->id_B, expected2.id_B);
    EXPECT_EQ(colliding_0_2_result->depth, expected2.depth);
    EXPECT_TRUE(aris::dynamic::s_is_equal(3, colliding_0_2_result->p_WCa.data(),
                                          expected2.p_WCa.data(), 1e-13));
    EXPECT_TRUE(aris::dynamic::s_is_equal(3, colliding_0_2_result->p_WCb.data(),
                                          expected2.p_WCb.data(), 1e-13));
    EXPECT_TRUE(
        aris::dynamic::s_is_equal(3, colliding_0_2_result->nhat_AB_W.data(),
                                  expected2.nhat_AB_W.data(), 1e-13));
  }

  // Compute penetration and confirm that a single penetration with the
  // expected1 properties was found. Provide the geometry ids of the sphere
  // located at the origin and the sphere positioned to be in collision.
  void expectAllPenetration(GeometryId origin_sphere,
                            GeometryId colliding_sphere1,
                            GeometryId colliding_sphere2,
                            PhysicsEngine* engine = nullptr) {
    std::vector<PenetrationAsPointPair> penetrations =
        engine->computePointPairPenetration();
    ASSERT_EQ(penetrations.size(), 3);
    // check for two penetration order
    PenetrationAsPointPair* colliding_0_1_result;
    PenetrationAsPointPair* colliding_1_2_result;
    PenetrationAsPointPair* colliding_0_2_result;
    for (auto& penetration : penetrations) {
      switch (penetration.id_A + penetration.id_B) {
        case 1:
          colliding_0_1_result = &penetration;
          break;
        case 2:
          colliding_0_2_result = &penetration;
          break;
        case 3:
          colliding_1_2_result = &penetration;
          break;
      }
    }

    // The order of colliding sphere is guaranteed,
    // see the picture of test case above

    // Setting expected penetration result of geometry 0 and 1
    PenetrationAsPointPair expected1;
    expected1.id_A = origin_sphere;
    expected1.id_B = colliding_sphere1;
    expected1.depth = 2 * radius_ - colliding_x1_;
    // Contact point on the origin_sphere.
    Vec3s p_WCo1{radius_, 0, 0};
    // Contact point on the colliding_sphere1.
    Vec3s p_WCc1{colliding_x1_ - radius_, 0, 0};
    expected1.p_WCa = p_WCo1;
    expected1.p_WCb = p_WCc1;
    Vec3s norm_into_B = Vec3s::UnitX();
    expected1.nhat_AB_W = norm_into_B;

    // Setting expected penetration result of geometry 0 and 2
    PenetrationAsPointPair expected2;
    expected2.id_A = origin_sphere;
    expected2.id_B = colliding_sphere2;
    expected2.depth = 0.375;
    // Contact point on the origin_sphere.
    Vec3s p_WCo2{0.3, 0.4, 0};
    // Contact point on the colliding_sphere2.
    Vec3s p_WCc2{0.075, 0.1, 0};
    expected2.p_WCa = p_WCo2;
    expected2.p_WCb = p_WCc2;
    Vec3s norm_into_C1{0.6, 0.8, 0};
    expected2.nhat_AB_W = norm_into_C1;

    // Setting expected penetration result of geometry 1 and 2
    PenetrationAsPointPair expected3;
    expected3.id_A = colliding_sphere1;
    expected3.id_B = colliding_sphere2;
    expected3.depth = 0.375;
    // Contact point on the colliding_sphere1.
    Vec3s p_WCo3{0.45, 0.4, 0};
    // Contact point on the colliding_sphere2.
    Vec3s p_WCc3{0.675, 0.1, 0};
    expected3.p_WCa = p_WCo3;
    expected3.p_WCb = p_WCc3;
    Vec3s norm_into_C2{-0.6, 0.8, 0};
    expected3.nhat_AB_W = norm_into_C2;

    // Check penetration result and expection result of colliding 0 and 1
    EXPECT_EQ(colliding_0_1_result->id_A, expected1.id_A);
    EXPECT_EQ(colliding_0_1_result->id_B, expected1.id_B);
    EXPECT_EQ(colliding_0_1_result->depth, expected1.depth);
    EXPECT_TRUE(aris::dynamic::s_is_equal(3, colliding_0_1_result->p_WCa.data(),
                                          expected1.p_WCa.data(), 1e-13));
    EXPECT_TRUE(aris::dynamic::s_is_equal(3, colliding_0_1_result->p_WCb.data(),
                                          expected1.p_WCb.data(), 1e-13));
    EXPECT_TRUE(
        aris::dynamic::s_is_equal(3, colliding_0_1_result->nhat_AB_W.data(),
                                  expected1.nhat_AB_W.data(), 1e-13));

    // Check penetration result and expection result of colliding 1 and 2
    EXPECT_EQ(colliding_0_2_result->id_A, expected2.id_A);
    EXPECT_EQ(colliding_0_2_result->id_B, expected2.id_B);
    EXPECT_EQ(colliding_0_2_result->depth, expected2.depth);
    EXPECT_TRUE(aris::dynamic::s_is_equal(3, colliding_0_2_result->p_WCa.data(),
                                          expected2.p_WCa.data(), 1e-13));
    EXPECT_TRUE(aris::dynamic::s_is_equal(3, colliding_0_2_result->p_WCb.data(),
                                          expected2.p_WCb.data(), 1e-13));
    EXPECT_TRUE(
        aris::dynamic::s_is_equal(3, colliding_0_2_result->nhat_AB_W.data(),
                                  expected2.nhat_AB_W.data(), 1e-13));

    // Check penetration result and expection result of colliding 0 and 2
    EXPECT_EQ(colliding_1_2_result->id_A, expected3.id_A);
    EXPECT_EQ(colliding_1_2_result->id_B, expected3.id_B);
    EXPECT_EQ(colliding_1_2_result->depth, expected3.depth);
    EXPECT_TRUE(aris::dynamic::s_is_equal(3, colliding_1_2_result->p_WCa.data(),
                                          expected3.p_WCa.data(), 1e-13));
    EXPECT_TRUE(aris::dynamic::s_is_equal(3, colliding_1_2_result->p_WCb.data(),
                                          expected3.p_WCb.data(), 1e-13));
    EXPECT_TRUE(
        aris::dynamic::s_is_equal(3, colliding_1_2_result->nhat_AB_W.data(),
                                  expected3.nhat_AB_W.data(), 1e-13));
  }

  unique_ptr<PhysicsEngine> engine_{std::make_unique<PhysicsEngine>()};
  collision::CollisionDetection* collision_engine_;

  double X_WGs_[21]{0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                    0, 0, 1, 0, 0, 0, 0, 0, 0, 1};
  const double radius_{0.5};
  const double free_x1_{2.5 * radius_};
  const double free_x2_{0 * radius_};
  const double free_y_{2.5 * radius_};
  const double colliding_x1_{1.5 * radius_};
  const double colliding_y1_{1.5 * radius_};
  const double colliding_y2_{1.0 * radius_};
  const double colliding_x2_{0.75 * radius_};
};

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
const double plane_position_and_euler321[6]{0, 0, 0, 0, 0, 0};
const double link1_position_and_euler321[6]{
    planeX * 0.5, planeY * 0.5, 0, 0, 0, 0};
const double link2_position_and_euler321[6]{
    planeX * 0.5, -planeY * 0.5, 0, 0, 0, 0};
const double link3_position_and_euler321[6]{
    -planeX * 0.5, planeY * 0.5, 0, 0, 0, 0};
const double link4_position_and_euler321[6]{
    -planeX * 0.5, -planeY * 0.5, 0, 0, 0, 0};
const double surface_position_and_euler321[6]{0, 0, -2, 0, 0, 0};

const double joint1_position[3]{planeX * 0.5, planeY * 0.5, 0.0};
const double joint1_axis[3]{0.0, 0.0, 1.0};
const double joint2_position[3]{planeX * 0.5, -planeY * 0.5, 0.0};
const double joint2_axis[3]{0.0, 0.0, 1.0};
const double joint3_position[3]{-planeX * 0.5, planeY * 0.5, 0.0};
const double joint3_axis[3]{0.0, 0.0, 1.0};
const double joint4_position[3]{-planeX * 0.5, -planeY * 0.5, 0.0};
const double joint4_axis[3]{0.0, 0.0, 1.0};

class TableSurfaceModelTestor : public ::testing::Test {
  using SireGeometryPool =
      aris::core::PointerArray<geometry::CollidableGeometry,
                               aris::dynamic::Geometry>;

 private:
  double plane_inertia_vector[10]{0};
  double link1_inertia_vector[10]{0};
  double link2_inertia_vector[10]{0};
  double link3_inertia_vector[10]{0};
  double link4_inertia_vector[10]{0};
  double surface_inertia_vector[10]{0};

 protected:
  double leg_accel_[4]{0};
  const int num_leg_{4};
  std::vector<std::array<double, 3>> ori_vec{0};
  std::vector<std::array<double, 16>> T_vec{0};
  std::vector<common::PenetrationAsPointPair> penetration_pairs;
  aris::dynamic::Model table_model_;
  sire::core::MaterialManager manager_;
  unique_ptr<PhysicsEngine> engine_{std::make_unique<PhysicsEngine>()};
  SireGeometryPool* geometry_pool_;
  collision::CollisionFilter* filter_;
  collision::CollisionDetection* collision_engine_;

  // iv -> inertia vector;
  auto calcSphereInertia(double mass, double radius, double* iv) -> void {
    iv[0] = mass;
    double ixyz = 0.4 * mass * radius * radius;
    iv[4] = iv[5] = iv[6] = ixyz;
    return;
  }
  auto calcBoxInertia(double mass, double x, double y, double z, double* iv)
      -> void {
    iv[0] = mass;
    iv[4] = mass * (y * y + z * z) / 12;  // ix
    iv[5] = mass * (x * x + z * z) / 12;  // iy
    iv[6] = mass * (x * x + y * y) / 12;  // iz
    return;
  }
  void initTablePlaneModel() {
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
    sire::core::FixedJoint::add2ModelRelative(&table_model_, link1, plane,
                                              joint1_position);
    sire::core::FixedJoint::add2ModelRelative(&table_model_, link2, plane,
                                              joint2_position);
    sire::core::FixedJoint::add2ModelRelative(&table_model_, link3, plane,
                                              joint3_position);
    sire::core::FixedJoint::add2ModelRelative(&table_model_, link4, plane,
                                              joint4_position);

    auto& force1 = table_model_.forcePool().add<aris::dynamic::GeneralForce>(
        "gf1", &link1.markerPool().at(0), &surface.markerPool().at(0));
    auto& force2 = table_model_.forcePool().add<aris::dynamic::GeneralForce>(
        "gf2", &link2.markerPool().at(0), &surface.markerPool().at(0));
    auto& force3 = table_model_.forcePool().add<aris::dynamic::GeneralForce>(
        "gf3", &link3.markerPool().at(0), &surface.markerPool().at(0));
    auto& force4 = table_model_.forcePool().add<aris::dynamic::GeneralForce>(
        "gf4", &link4.markerPool().at(0), &surface.markerPool().at(0));
    engine_->setContactForceIdxSize(0, 4);

    table_model_.solverPool().add<aris::dynamic::InverseKinematicSolver>();
    table_model_.solverPool().add<aris::dynamic::ForwardKinematicSolver>();
    table_model_.solverPool().add<aris::dynamic::InverseDynamicSolver>();
    table_model_.solverPool().add<aris::dynamic::ForwardDynamicSolver>();
    // std::cout << aris::core::toXmlString(table_model_) << std::endl;
    table_model_.init();

    // T_vec.resize(num_leg_);
    ori_vec.resize(num_leg_);
    ori_vec[0] = {planeX * 0.5, planeY * 0.5, -sphereRadius};
    ori_vec[1] = {planeX * 0.5, -planeY * 0.5, -sphereRadius};
    ori_vec[2] = {-planeX * 0.5, planeY * 0.5, -sphereRadius};
    ori_vec[3] = {-planeX * 0.5, -planeY * 0.5, -sphereRadius};
    double z_vec[3]{0, 0, 1};
    double x_vec[3]{1, 0, 0};

    common::PenetrationAsPointPair c;
    c.id_A = link1.id();
    c.id_B = surface.id();
    c.p_WC = {planeX * 0.5, planeY * 0.5, -1};
    c.depth = 0.0001;
    c.nhat_AB_W = {0, 0, 1};
    penetration_pairs.push_back(c);
    c.id_A = link2.id();
    c.p_WC = {planeX * 0.5, -planeY * 0.5, -1};
    penetration_pairs.push_back(c);
    c.id_A = link3.id();
    c.p_WC = {-planeX * 0.5, planeY * 0.5, -1};
    penetration_pairs.push_back(c);
    c.id_A = link4.id();
    c.p_WC = {-planeX * 0.5, -planeY * 0.5, -1};
    penetration_pairs.push_back(c);
    T_vec.push_back({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1});
    T_vec.push_back({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1});
    T_vec.push_back({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1});
    T_vec.push_back({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1});
    aris::dynamic::s_pp2pm(penetration_pairs[0].p_WC.data(), T_vec[0].data());
    aris::dynamic::s_pp2pm(penetration_pairs[1].p_WC.data(), T_vec[1].data());
    aris::dynamic::s_pp2pm(penetration_pairs[2].p_WC.data(), T_vec[2].data());
    aris::dynamic::s_pp2pm(penetration_pairs[3].p_WC.data(), T_vec[3].data());
  }
  void initPhysicsEngine() {
    collision_engine_ = new collision::CollisionDetection();
    engine_->resetCollisionDetection(collision_engine_);
    engine_->setCollisionDetectionFlag(true);
    using namespace sire::physics::geometry;
    auto& geoPool = engine_->geometryPool();
    geoPool.add<SphereCollisionGeometry>(sphereRadius, 0, true,
                                         sire::default_pm);
    geoPool.back().setMaterial("m1");
    geoPool.add<SphereCollisionGeometry>(sphereRadius, 1, true,
                                         sire::default_pm);
    geoPool.back().setMaterial("m1");
    geoPool.add<SphereCollisionGeometry>(sphereRadius, 2, true,
                                         sire::default_pm);
    geoPool.back().setMaterial("m1");
    geoPool.add<SphereCollisionGeometry>(sphereRadius, 3, true,
                                         sire::default_pm);
    geoPool.back().setMaterial("m1");
    geoPool.add<SphereCollisionGeometry>(sphereRadius, 4, true,
                                         sire::default_pm);
    geoPool.back().setMaterial("m1");
    geoPool.add<SphereCollisionGeometry>(sphereRadius, 5, true,
                                         sire::default_pm);
    geoPool.back().setMaterial("m1");
    geoPool.add<SphereCollisionGeometry>(sphereRadius, 6, true,
                                         sire::default_pm);
    geoPool.back().setMaterial("m1");
    aris::core::Matrix filter_state{1, 0, 0, 0, 1, 0, 0, 0, 1};
    engine_->collisionFilter().setStateMat(filter_state);
    sire::core::PropMap property("k:2e8,d:1000,cr:0.2");
    manager_.addProp(sire::core::SortedPair<std::string>("m1", "m1"), property);
  }
  void addFixedJointUsingModelInit() {
    auto& jointPool = table_model_.jointPool();
    auto& partPool = table_model_.partPool();
    EXPECT_EQ(table_model_.forwardKinematics(), 0);
    const double pq_result1[4][7]{{4, 4, 0, 0, 0, 0, 1},
                                  {4, -4, 0, 0, 0, 0, 1},
                                  {-4, 4, 0, 0, 0, 0, 1},
                                  {-4, -4, 0, 0, 0, 0, 1}};
    for (int i = 0; i < 4; ++i) {
      double pq[7];
      partPool[i + 2].getPq(pq);
      EXPECT_TRUE(aris::dynamic::s_is_equal(1, 7, pq, pq_result1[i], 1e-10));
      // aris::dynamic::dsp(1, 7, pq);
    }
    EXPECT_EQ(jointPool.size(), 4);
    // 假设需要增加四个约束
    const sire::Size numNewFixedJoint = 4;
    const double* newFixedJointPosition[numNewFixedJoint] = {
        joint2_position, joint4_position, joint1_position, joint3_position};
    std::map<sire::Size, sire::Size> updatedMakerPoolSize;
    for (sire::Size i = 0; i < numNewFixedJoint; ++i) {
      sire::core::FixedJoint::add2ModelRelative(&table_model_, partPool[i + 2],
                                                partPool[1],
                                                newFixedJointPosition[i]);
      auto lowerBoundPrt = updatedMakerPoolSize.lower_bound(1);
      if (lowerBoundPrt != updatedMakerPoolSize.end() &&
          !(updatedMakerPoolSize.key_comp()(lowerBoundPrt->first, 1))) {
        ++lowerBoundPrt->second;
      } else {
        updatedMakerPoolSize.insert(
            lowerBoundPrt, std::map<sire::Size, sire::Size>::value_type(1, 1));
      }
      lowerBoundPrt = updatedMakerPoolSize.lower_bound(i + 2);
      if (lowerBoundPrt != updatedMakerPoolSize.end() &&
          !(updatedMakerPoolSize.key_comp()(lowerBoundPrt->first, i + 2))) {
        ++lowerBoundPrt->second;
      } else {
        updatedMakerPoolSize.insert(
            lowerBoundPrt,
            std::map<sire::Size, sire::Size>::value_type(i + 2, 1));
      }
    }
    EXPECT_EQ(partPool[1].markerPool().size(), 8);
    EXPECT_EQ(partPool[2].markerPool().size(), 2);
    EXPECT_EQ(partPool[3].markerPool().size(), 2);
    EXPECT_EQ(partPool[4].markerPool().size(), 2);
    EXPECT_EQ(partPool[5].markerPool().size(), 2);
    EXPECT_EQ(jointPool.size(), 8);
    // std::cout << aris::core::toXmlString(table_model_) << std::endl;

    table_model_.init();
    EXPECT_EQ(table_model_.forwardKinematics(), 0);
    EXPECT_EQ(jointPool.size(), 8);
    const double pq_result2[4][7]{{4, 4, 0, 0, 0, 0, 1},
                                  {4, -4, 0, 0, 0, 0, 1},
                                  {-4, 4, 0, 0, 0, 0, 1},
                                  {-4, -4, 0, 0, 0, 0, 1}};
    for (int i = 0; i < 4; ++i) {
      double pq[7];
      partPool[i + 2].getPq(pq);
      EXPECT_TRUE(aris::dynamic::s_is_equal(1, 7, pq, pq_result2[i], 1e-10));
      // aris::dynamic::dsp(1, 7, pq);
    }

    // TODO: 为什么Interaction的prt与maker不能用指针直接记录而是使用了变量名，
    //       具体用的话还需要两次查找
    for (sire::Size i = 0; i < numNewFixedJoint; ++i) {
      jointPool.pop_back();
    }
    for (auto const& [prtIdx, jntNum] : updatedMakerPoolSize) {
      for (sire::Size i = 0; i < jntNum; ++i) {
        partPool[prtIdx].markerPool().pop_back();
      }
    }
    // std::cout << aris::core::toXmlString(table_model_) << std::endl;
    EXPECT_EQ(partPool[1].markerPool().size(), 4);
    EXPECT_EQ(partPool[2].markerPool().size(), 1);
    EXPECT_EQ(partPool[3].markerPool().size(), 1);
    EXPECT_EQ(partPool[4].markerPool().size(), 1);
    EXPECT_EQ(partPool[5].markerPool().size(), 1);
    EXPECT_EQ(jointPool.size(), 4);
    table_model_.init();
    EXPECT_EQ(table_model_.forwardKinematics(), 0);
  }
  void addFixedJointUsingInitInteraction() {
    auto init_interaction = [](aris::dynamic::Interaction& interaction,
                               aris::dynamic::Model* m) -> void {
      if (interaction.prtNameM().empty() && interaction.prtNameN().empty() &&
          interaction.makNameI().empty() && interaction.makNameJ().empty())
        return;

      auto find_part = [m](std::string_view name) -> aris::dynamic::Part* {
        auto found = std::find_if(
            m->partPool().begin(), m->partPool().end(),
            [name](const auto& part) -> bool { return part.name() == name; });
        return found == m->partPool().end() ? nullptr : &*found;
      };

      auto find_marker = [](aris::dynamic::Part* part,
                            std::string_view name) -> aris::dynamic::Marker* {
        auto found =
            std::find_if(part->markerPool().begin(), part->markerPool().end(),
                         [name](const auto& marker) -> bool {
                           return marker.name() == name;
                         });
        return found == part->markerPool().end() ? nullptr : &*found;
      };

      auto prt_m = find_part(interaction.prtNameM());
      auto mak_i = find_marker(prt_m, interaction.makNameI());
      auto prt_n = find_part(interaction.prtNameN());
      auto mak_j = find_marker(prt_n, interaction.makNameJ());

      interaction.setMakI(&*mak_i);
      interaction.setMakJ(&*mak_j);
    };

    auto& jointPool = table_model_.jointPool();
    auto& partPool = table_model_.partPool();
    sire::Size tempJointIdxOffset = jointPool.size();
    EXPECT_EQ(table_model_.forwardKinematics(), 0);
    // for (int i = 0; i < 4; ++i) {
    //   double pq[7];
    //   partPool[i + 2].getPq(pq);
    //   aris::dynamic::dsp(1, 7, pq);
    // }
    EXPECT_EQ(jointPool.size(), 4);
    // 假设需要增加四个约束
    const sire::Size numNewFixedJoint = 4;
    const double* newFixedJointPosition[numNewFixedJoint] = {
        joint2_position, joint4_position, joint1_position, joint3_position};
    std::map<sire::Size, sire::Size> updatedMakerPoolSize;
    for (sire::Size i = 0; i < numNewFixedJoint; ++i) {
      sire::core::FixedJoint::add2ModelRelative(&table_model_, partPool[i + 2],
                                                partPool[1],
                                                newFixedJointPosition[i]);
      auto lowerBoundPrt = updatedMakerPoolSize.lower_bound(1);
      if (lowerBoundPrt != updatedMakerPoolSize.end() &&
          !(updatedMakerPoolSize.key_comp()(lowerBoundPrt->first, 1))) {
        ++lowerBoundPrt->second;
      } else {
        updatedMakerPoolSize.insert(
            lowerBoundPrt, std::map<sire::Size, sire::Size>::value_type(1, 1));
      }
      lowerBoundPrt = updatedMakerPoolSize.lower_bound(i + 2);
      if (lowerBoundPrt != updatedMakerPoolSize.end() &&
          !(updatedMakerPoolSize.key_comp()(lowerBoundPrt->first, i + 2))) {
        ++lowerBoundPrt->second;
      } else {
        updatedMakerPoolSize.insert(
            lowerBoundPrt,
            std::map<sire::Size, sire::Size>::value_type(i + 2, 1));
      }
    }
    for (sire::Size i = 0; i < 4; ++i) {
      jointPool[i + tempJointIdxOffset].resetModel(&table_model_);
      jointPool[i + tempJointIdxOffset].setId(i + tempJointIdxOffset);
      init_interaction(jointPool[i + tempJointIdxOffset], &table_model_);
    }
    // 调用 FK solver 的内存分配方法重新分配内存
    table_model_.solverPool().at(1).allocateMemory();

    EXPECT_EQ(table_model_.forwardKinematics(), 0);
    // dynamic_cast<aris::dynamic::UniversalSolver&>(
    //     table_model_.solverPool().at(1))
    //     .kinPosUpdateModel();
    EXPECT_EQ(jointPool.size(), 8);
    // for (int i = 0; i < 4; ++i) {
    //   double pq[7];
    //   partPool[i + 2].getPq(pq);
    //   aris::dynamic::dsp(1, 7, pq);
    // }

    for (sire::Size i = 0; i < 4; ++i) {
      jointPool.pop_back();
    }
    for (auto const& [prtIdx, jntNum] : updatedMakerPoolSize) {
      for (sire::Size i = 0; i < jntNum; ++i) {
        partPool[prtIdx].markerPool().pop_back();
      }
    }
    // std::cout << aris::core::toXmlString(table_model_) << std::endl;
    EXPECT_EQ(partPool[1].markerPool().size(), 4);
    EXPECT_EQ(partPool[2].markerPool().size(), 1);
    EXPECT_EQ(partPool[3].markerPool().size(), 1);
    EXPECT_EQ(partPool[4].markerPool().size(), 1);
    EXPECT_EQ(partPool[5].markerPool().size(), 1);
    EXPECT_EQ(jointPool.size(), 4);
    table_model_.init();
    EXPECT_EQ(table_model_.forwardKinematics(), 0);
  }
  void addFixedJointAtOtherCorner2TestFK() {
    auto& jointPool = table_model_.jointPool();
    auto& partPool = table_model_.partPool();
    EXPECT_EQ(table_model_.forwardKinematics(), 0);
    // for (int i = 0; i < 4; ++i) {
    //   double pq[7];
    //   partPool[i + 2].getPq(pq);
    //   aris::dynamic::dsp(1, 7, pq);
    // }
    EXPECT_EQ(jointPool.size(), 4);
    // 假设需要增加四个约束
    const sire::Size numNewFixedJoint = 4;
    const double* newFixedJointPosition[numNewFixedJoint] = {
        joint2_position, joint4_position, joint1_position, joint3_position};
    const double* fixedJointPosition[numNewFixedJoint] = {
        joint1_position, joint2_position, joint3_position, joint4_position};
    std::map<sire::Size, sire::Size> updatedMakerPoolSize;
    for (sire::Size i = 2; i < 3; ++i) {
      sire::core::FixedJoint::add2ModelAbs(&table_model_, partPool[i + 2],
                                           partPool[1], fixedJointPosition[i],
                                           newFixedJointPosition[i]);
      auto lowerBoundPrt = updatedMakerPoolSize.lower_bound(1);
      if (lowerBoundPrt != updatedMakerPoolSize.end() &&
          !(updatedMakerPoolSize.key_comp()(lowerBoundPrt->first, 1))) {
        ++lowerBoundPrt->second;
      } else {
        updatedMakerPoolSize.insert(
            lowerBoundPrt, std::map<sire::Size, sire::Size>::value_type(1, 1));
      }
      lowerBoundPrt = updatedMakerPoolSize.lower_bound(i + 2);
      if (lowerBoundPrt != updatedMakerPoolSize.end() &&
          !(updatedMakerPoolSize.key_comp()(lowerBoundPrt->first, i + 2))) {
        ++lowerBoundPrt->second;
      } else {
        updatedMakerPoolSize.insert(
            lowerBoundPrt,
            std::map<sire::Size, sire::Size>::value_type(i + 2, 1));
      }
    }
    // std::cout << aris::core::toXmlString(table_model_) << std::endl;
    EXPECT_EQ(jointPool.size(), 5);
    // rel_vec.size() = 4，其中prt4 与 prt1 的 cst_pool.size() = 2; 4个 relation
    // 的cst_pool分别为 [2 1 1 1] r_size_ = rel_vec.size() - prt_vec.size() + 1
    // < 0 这里认为整个机器人不是超定的，但是中间有超定的部分怎么办，比如
    // 两个fixedJoint约束一个prt，所以老师这边实际上并没有用最小二乘求解最小，如果他认为不是超定的就不会调用求解，fm_,
    // fn_全是零 那老师的代码要怎么用？
    // 老师的代码中，提到，“只有不是串联机械臂才会有非零”，显然，对于缩短步长步进，老师认为串联机械臂的位置更新不会碰到需要缩短步长步进的情况
    // 随着循环次数增大，总能找到error小于max_error的解
    // 当 A = [I6x6; I6x6] b = [0 0 0 0 0 0 8 0 0 0 0 0]'; 可以求得 dp = [4 0 0
    // 0 0 0]; p_new = p_old + dp = [0 4 0 0 0 0]
    // 刚好就是最小二乘的最优点，现在的问题是 updF 与
    // sovXp没有正常求解这个问题，也就是更新的数据不对，
    // 老师的最小二乘在这种情况下为啥updF的 fm_ fn_都是零？
    // 好像是把motion当可调整子系统了，没有把 ps当作直接可调的 首先 bc_
    // 更新的数值是对的，所以要看 F' 更新了个啥
    table_model_.init();
    EXPECT_NE(table_model_.forwardKinematics(), 0);
    EXPECT_EQ(dynamic_cast<aris::dynamic::UniversalSolver&>(
                  table_model_.solverPool().at(1))
                  .error(),
              8.0);
    // auto& uniSolver = dynamic_cast<aris::dynamic::UniversalSolver&>(
    //     table_model_.solverPool().at(1));
    // std::cout << "Error: " << uniSolver.error() << std::endl;
    // std::cout << "iterCount: " << uniSolver.iterCount() << std::endl;
    // std::cout << "maxIterCount: " << uniSolver.maxIterCount() << std::endl;
    // uniSolver.kinPosUpdateModel();
    // std::cout << "try to set directly" << std::endl;
    // std::cout << aris::core::toXmlString(table_model_) << std::endl;
    // for (int i = 0; i < 6; ++i) {
    //   double pq[7];
    //   partPool[i].getPq(pq);
    //   aris::dynamic::dsp(1, 7, pq);
    // }

    // TODO: 为什么Interaction的prt与maker不能用指针直接记录而是使用了变量名，
    //       具体用的话还需要两次查找
    for (sire::Size i = 0; i < 1; ++i) {
      jointPool.pop_back();
    }
    for (auto const& [prtIdx, jntNum] : updatedMakerPoolSize) {
      for (sire::Size i = 0; i < jntNum; ++i) {
        partPool[prtIdx].markerPool().pop_back();
      }
    }
    // std::cout << aris::core::toXmlString(table_model_) << std::endl;
    EXPECT_EQ(partPool[1].markerPool().size(), 4);
    EXPECT_EQ(partPool[2].markerPool().size(), 1);
    EXPECT_EQ(partPool[3].markerPool().size(), 1);
    EXPECT_EQ(partPool[4].markerPool().size(), 1);
    EXPECT_EQ(partPool[5].markerPool().size(), 1);
    EXPECT_EQ(jointPool.size(), 4);
    table_model_.init();
    EXPECT_EQ(table_model_.forwardKinematics(), 0);
  }
};

class TableGroundModelTestor : public ::testing::Test {
  using SireGeometryPool =
      aris::core::PointerArray<geometry::CollidableGeometry,
                               aris::dynamic::Geometry>;

 private:
  double plane_inertia_vector[10]{0};
  double link1_inertia_vector[10]{0};
  double link2_inertia_vector[10]{0};
  double link3_inertia_vector[10]{0};
  double link4_inertia_vector[10]{0};
  double surface_inertia_vector[10]{0};

 protected:
  double leg_accel_[4]{0};
  const int num_leg_{4};
  std::vector<std::array<double, 3>> ori_vec{0};
  std::vector<std::array<double, 16>> T_vec{0};
  std::vector<common::PenetrationAsPointPair> penetration_pairs;
  aris::dynamic::Model table_model_;
  sire::core::MaterialManager manager_;
  unique_ptr<PhysicsEngine> engine_{std::make_unique<PhysicsEngine>()};
  SireGeometryPool* geometry_pool_;
  collision::CollisionFilter* filter_;
  collision::CollisionDetection* collision_engine_;

  // iv -> inertia vector;
  auto calcSphereInertia(double mass, double radius, double* iv) -> void {
    iv[0] = mass;
    double ixyz = 0.4 * mass * radius * radius;
    iv[4] = iv[5] = iv[6] = ixyz;
    return;
  }
  auto calcBoxInertia(double mass, double x, double y, double z, double* iv)
      -> void {
    iv[0] = mass;
    iv[4] = mass * (y * y + z * z) / 12;  // ix
    iv[5] = mass * (x * x + z * z) / 12;  // iy
    iv[6] = mass * (x * x + y * y) / 12;  // iz
    return;
  }
  void initTablePlaneModel() {
    double gravityAs[6]{0, 0, 0, 0, 0, 0};
    calcBoxInertia(planeMass, planeX, planeY, planeZ, plane_inertia_vector);
    calcSphereInertia(sphereMass, sphereRadius, link1_inertia_vector);
    calcSphereInertia(sphereMass, sphereRadius, link2_inertia_vector);
    calcSphereInertia(sphereMass, sphereRadius, link3_inertia_vector);
    calcSphereInertia(sphereMass, sphereRadius, link4_inertia_vector);
    calcBoxInertia(surfaceMass, surfaceX, surfaceY, surfaceZ,
                   surface_inertia_vector);
    table_model_.environment().setGravity(gravityAs);
    auto& ground = table_model_.ground();
    ground.addMarker("ground_mkr");
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
    // auto& surface = table_model_.addPartByPe(surface_position_and_euler321,
    //                                          "313", surface_inertia_vector);
    // surface.addMarker("surface_mkr");
    sire::core::FixedJoint::add2ModelRelative(&table_model_, link1, plane,
                                              joint1_position);
    sire::core::FixedJoint::add2ModelRelative(&table_model_, link2, plane,
                                              joint2_position);
    sire::core::FixedJoint::add2ModelRelative(&table_model_, link3, plane,
                                              joint3_position);
    sire::core::FixedJoint::add2ModelRelative(&table_model_, link4, plane,
                                              joint4_position);

    auto& force1 = table_model_.forcePool().add<aris::dynamic::GeneralForce>(
        "gf1", &link1.markerPool().at(0), &ground.markerPool().at(0));
    auto& force2 = table_model_.forcePool().add<aris::dynamic::GeneralForce>(
        "gf2", &link2.markerPool().at(0), &ground.markerPool().at(0));
    auto& force3 = table_model_.forcePool().add<aris::dynamic::GeneralForce>(
        "gf3", &link3.markerPool().at(0), &ground.markerPool().at(0));
    auto& force4 = table_model_.forcePool().add<aris::dynamic::GeneralForce>(
        "gf4", &link4.markerPool().at(0), &ground.markerPool().at(0));
    engine_->setContactForceIdxSize(0, 4);

    table_model_.solverPool().add<aris::dynamic::InverseKinematicSolver>();
    table_model_.solverPool().add<aris::dynamic::ForwardKinematicSolver>();
    table_model_.solverPool().add<aris::dynamic::InverseDynamicSolver>();
    table_model_.solverPool().add<aris::dynamic::ForwardDynamicSolver>();
    // std::cout << aris::core::toXmlString(table_model_) << std::endl;
    table_model_.init();

    // T_vec.resize(num_leg_);
    ori_vec.resize(num_leg_);
    ori_vec[0] = {planeX * 0.5, planeY * 0.5, -sphereRadius};
    ori_vec[1] = {planeX * 0.5, -planeY * 0.5, -sphereRadius};
    ori_vec[2] = {-planeX * 0.5, planeY * 0.5, -sphereRadius};
    ori_vec[3] = {-planeX * 0.5, -planeY * 0.5, -sphereRadius};
    double z_vec[3]{0, 0, 1};
    double x_vec[3]{1, 0, 0};

    common::PenetrationAsPointPair c;
    c.id_A = ground.id();
    c.id_B = link1.id();
    c.depth = 0.0001;
    c.p_WC = {planeX * 0.5, planeY * 0.5, -1};
    c.nhat_AB_W = {0, 0, 1};
    penetration_pairs.push_back(c);
    c.id_B = link2.id();
    c.p_WC = {planeX * 0.5, -planeY * 0.5, -1};
    penetration_pairs.push_back(c);
    c.id_B = link3.id();
    c.p_WC = {-planeX * 0.5, planeY * 0.5, -1};
    penetration_pairs.push_back(c);
    c.id_B = link4.id();
    c.p_WC = {-planeX * 0.5, -planeY * 0.5, -1};
    penetration_pairs.push_back(c);
    T_vec.push_back({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1});
    T_vec.push_back({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1});
    T_vec.push_back({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1});
    T_vec.push_back({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1});
    aris::dynamic::s_pp2pm(penetration_pairs[0].p_WC.data(), T_vec[0].data());
    aris::dynamic::s_pp2pm(penetration_pairs[1].p_WC.data(), T_vec[1].data());
    aris::dynamic::s_pp2pm(penetration_pairs[2].p_WC.data(), T_vec[2].data());
    aris::dynamic::s_pp2pm(penetration_pairs[3].p_WC.data(), T_vec[3].data());
  }
  void initPhysicsEngine() {
    collision_engine_ = new collision::CollisionDetection();
    engine_->resetCollisionDetection(collision_engine_);
    using namespace sire::physics::geometry;
    auto& geoPool = engine_->geometryPool();
    geoPool.add<SphereCollisionGeometry>(sphereRadius, 0, true,
                                         sire::default_pm);
    geoPool.back().setMaterial("m1");
    geoPool.add<SphereCollisionGeometry>(sphereRadius, 1, true,
                                         sire::default_pm);
    geoPool.back().setMaterial("m1");
    geoPool.add<SphereCollisionGeometry>(sphereRadius, 2, true,
                                         sire::default_pm);
    geoPool.back().setMaterial("m1");
    geoPool.add<SphereCollisionGeometry>(sphereRadius, 3, true,
                                         sire::default_pm);
    geoPool.back().setMaterial("m1");
    geoPool.add<SphereCollisionGeometry>(sphereRadius, 4, true,
                                         sire::default_pm);
    geoPool.back().setMaterial("m1");
    geoPool.add<SphereCollisionGeometry>(sphereRadius, 5, true,
                                         sire::default_pm);
    geoPool.back().setMaterial("m1");
    geoPool.add<SphereCollisionGeometry>(sphereRadius, 6, true,
                                         sire::default_pm);
    geoPool.back().setMaterial("m1");
    aris::core::Matrix filter_state{1, 0, 0, 0, 1, 0, 0, 0, 1};
    engine_->collisionFilter().setStateMat(filter_state);
    sire::core::PropMap property("k:2e8,d:1000,cr:0.2");
    manager_.addProp(sire::core::SortedPair<std::string>("m1", "m1"), property);
  }
};

class SurfaceBallConstraintTester : public ::testing::Test {
  using SireGeometryPool =
      aris::core::PointerArray<geometry::CollidableGeometry,
                               aris::dynamic::Geometry>;

 private:
  // Object 1 parameter with sphere foot
  const double planeMass = 3;
  const double planeX = 8, planeY = 8;
  const double planeZ = 0.1;
  // Sphere foot parameters
  const double sphereMass = 1;
  const double sphereRadius = 1;

  double plane_position_and_euler321[6]{0, 0, 0, 0, 0, 0};
  double link1_position_and_euler321[6]{0, 4, 0, 0, 0, 0};

  double plane_inertia_vector[10]{0};
  double link1_inertia_vector[10]{0};

 protected:
  aris::dynamic::Model model_;
  unique_ptr<PhysicsEngine> engine_{std::make_unique<PhysicsEngine>()};
  SireGeometryPool* geometry_pool_;
  collision::CollisionFilter* filter_;
  collision::CollisionDetection* collision_engine_;

  // iv -> inertia vector;
  auto calcSphereInertia(double mass, double radius, double* iv) -> void {
    iv[0] = mass;
    double ixyz = 0.4 * mass * radius * radius;
    iv[4] = iv[5] = iv[6] = ixyz;
    return;
  }
  auto calcBoxInertia(double mass, double x, double y, double z, double* iv)
      -> void {
    iv[0] = mass;
    iv[4] = mass * (y * y + z * z) / 12;  // ix
    iv[5] = mass * (x * x + z * z) / 12;  // iy
    iv[6] = mass * (x * x + y * y) / 12;  // iz
    return;
  }
  void initTablePlaneModel() {
    double gravityAs[6]{0, 0, 0, 0, 0, 0};
    calcBoxInertia(planeMass, planeX, planeY, planeZ, plane_inertia_vector);
    calcSphereInertia(sphereMass, sphereRadius, link1_inertia_vector);
    model_.environment().setGravity(gravityAs);
    model_.ground().addMarker("ground_mkr");
    auto& plane = model_.addPartByPe(plane_position_and_euler321, "313",
                                     plane_inertia_vector);
    auto& link1 = model_.addPartByPe(link1_position_and_euler321, "313",
                                     link1_inertia_vector);

    model_.solverPool().add<aris::dynamic::InverseKinematicSolver>();
    model_.solverPool().add<aris::dynamic::ForwardKinematicSolver>();
    model_.solverPool().add<aris::dynamic::InverseDynamicSolver>();
    model_.solverPool().add<aris::dynamic::ForwardDynamicSolver>();
    // std::cout << aris::core::toXmlString(model_) << std::endl;
    model_.init();
  }
  void initPhysicsEngine() {
    collision_engine_ = new collision::CollisionDetection();
    engine_->resetCollisionDetection(collision_engine_);
    aris::core::Matrix filter_state{1, 0, 0, 0, 1, 0, 0, 0, 1};
    engine_->collisionFilter().setStateMat(filter_state);
  }

  void addFixedAndPrismaticJointFKPosVel() {
    auto& jointPool = model_.jointPool();
    auto& partPool = model_.partPool();
    EXPECT_EQ(partPool.size(), 3);
    EXPECT_EQ(jointPool.size(), 0);
    const double pe[3]{0, 4, 0};
    const double axis[3]{1, 0, 0};
    model_.addPrismaticJoint(partPool[1], partPool[2], pe, axis);
    const double pe1[3]{4, 4, 0};
    const double pe2[3]{0, 4, 0};
    sire::core::FixedJoint::add2ModelAbs(&model_, partPool[1], partPool[2], pe1,
                                         pe2);
    // 先给放在正确位置的球添加 PrismaticJoint，FK之后判断位置正确性
    jointPool[1].activate(false);
    model_.solverPool().at(1).allocateMemory();
    // std::cout << aris::core::toXmlString(model_) << std::endl;
    EXPECT_EQ(model_.forwardKinematics(), 0);
    const double result_pq[3][7]{
        {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 4, 0, 0, 0, 0, 1}};
    for (int i = 0; i < partPool.size(); ++i) {
      double pq[7];
      partPool[i].getPq(pq);
      // aris::dynamic::dsp(1, 7, pq);
      EXPECT_TRUE(aris::dynamic::s_is_equal(1, 7, pq, result_pq[i], 1e-12));
    }
    // 给球一个向第四象限的平动速度，在添加 x 方向的平动约束后，FKVel的正确性
    double vs[6]{-1, -1, 0, 0, 0, 0};
    partPool[2].setVs(vs);
    EXPECT_EQ(model_.forwardKinematicsVel(), 0);
    double result_vs[3][6]{
        {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {-1, 0, 0, 0, 0, 0}};
    for (int i = 0; i < partPool.size(); ++i) {
      double vs[6];
      partPool[i].getVs(vs);
      // aris::dynamic::dsp(1, 6, vs);
      EXPECT_TRUE(aris::dynamic::s_is_equal(1, 6, vs, result_vs[i], 1e-12));
    }
    // 在 4 4 点添加FixedJoint，看看能不能正确将位置平移过去。
    jointPool[1].activate(true);
    model_.solverPool().at(1).allocateMemory();
    EXPECT_EQ(model_.forwardKinematics(), 0);
    double result2[3][7]{
        {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {4, 4, 0, 0, 0, 0, 1}};
    for (int i = 0; i < partPool.size(); ++i) {
      double pq[7];
      partPool[i].getPq(pq);
      // aris::dynamic::dsp(1, 7, pq);
      EXPECT_TRUE(aris::dynamic::s_is_equal(1, 7, pq, result2[i], 1e-12));
    }
  }
};

TEST_F(SimplePenetrationTest, PenetrationDynamicAndAnchored) {
  // Setup collision detection engine
  collision_engine_ = new collision::CollisionDetection();
  engine_->resetCollisionDetection(collision_engine_);
  engine_->setCollisionDetectionFlag(true);
  using namespace sire::physics::geometry;
  auto& geoPool = engine_->geometryPool();
  geoPool.add<SphereCollisionGeometry>(radius_, 0, false, sire::default_pm);
  geoPool.add<SphereCollisionGeometry>(radius_, 1, true, sire::default_pm);
  aris::core::Matrix filter_state{1, 0, 0, 1};
  engine_->collisionFilter().setStateMat(filter_state);
  engine_->init();

  cout << "Testing non-colliding case" << endl;
  moveDynamicSphere(1, false);
  expectNoPenetration(0, 1, engine_.get());
  cout << "Finish non-colliding test" << endl;

  cout << "Testing colliding case" << endl;
  moveDynamicSphere(1, true);
  expectPenetration(0, 1, engine_.get());
  cout << "Finish colliding test" << endl;
}

// 测试多物体但碰撞点检测结果与法线获取
TEST_F(MultipleObjectsPenetrationTest, PenetrationTwoDynamicAndAnchored) {
  // Setup collision detection engine
  collision_engine_ = new collision::CollisionDetection();
  engine_->resetCollisionDetection(collision_engine_);
  engine_->setCollisionDetectionFlag(true);
  using namespace sire::physics::geometry;
  auto& geoPool = engine_->geometryPool();
  geoPool.add<SphereCollisionGeometry>(radius_, 0, false, sire::default_pm);
  geoPool.add<SphereCollisionGeometry>(radius_, 1, true, sire::default_pm);
  geoPool.add<SphereCollisionGeometry>(radius_, 2, true, sire::default_pm);
  aris::core::Matrix filter_state{1, 0, 0, 0, 1, 0, 0, 0, 1};
  engine_->collisionFilter().setStateMat(filter_state);
  engine_->init();

  cout << "Testing non-colliding case" << endl;
  moveDynamicSphere(CollisionType::NON_COLLISION);
  expectNoPenetration(engine_.get());
  cout << "Finish non-colliding test" << endl;

  cout << "Testing two colliding case" << endl;
  moveDynamicSphere(CollisionType::TWO_COLLISION);
  expectTwoPenetration(0, 1, 2, engine_.get());
  cout << "Finish two colliding test" << endl;

  cout << "Testing all colliding case" << endl;
  moveDynamicSphere(CollisionType::ALL_COLLISION);
  expectAllPenetration(0, 1, 2, engine_.get());
  cout << "Finish all colliding test" << endl;
}

TEST_F(TableSurfaceModelTestor, testContactPointInertiaMatrix) {
  // 第一个桌子的加速度矩阵对不上感觉是我们的关节使用的虽然是z轴向上的情况，但是数值上可能带来误差，
  // 尝试实现一下全锁住的约束来测试一下。
  initTablePlaneModel();
  initPhysicsEngine();
  engine_->init(&table_model_);
  std::vector<double> cpi;
  EXPECT_EQ(table_model_.forcePool().size(), 4);
  sire::physics::contact::AverageForceContactSolver solver;
  solver.init(engine_.get());
  // std::cout << aris::core::toXmlString(table_model_) << std::endl;
  solver.cptContactPointInertiaMatrix(penetration_pairs, T_vec, cpi);
  EXPECT_EQ(table_model_.forcePool().size(), 4);
  // std::cout << aris::core::toXmlString(table_model_) << std::endl;
  double matlab_result[576] = {
      3.935124,   -10.100000, 20.400625,  0.000000,   0.000000,   0.000000,
      17.824750,  -10.100000, 20.400625,  0.000000,   0.000000,   0.000000,
      3.935124,   10.100000,  -20.400625, 0.000000,   0.000000,   0.000000,
      17.824750,  10.100000,  -20.400625, 0.000000,   0.000000,   0.000000,
      -10.100000, 3.935124,   20.400625,  0.000000,   0.000000,   0.000000,
      10.100000,  3.935124,   -20.400625, 0.000000,   0.000000,   0.000000,
      -10.100000, 17.824750,  20.400625,  0.000000,   0.000000,   0.000000,
      10.100000,  17.824750,  -20.400625, 0.000000,   0.000000,   0.000000,
      20.400625,  20.400625,  1.869152,   0.000000,   0.000000,   0.000000,
      20.400625,  20.400625,  7.000000,   0.000000,   0.000000,   0.000000,
      20.400625,  20.400625,  7.000000,   0.000000,   0.000000,   0.000000,
      20.400625,  20.400625,  -4.011429,  0.000000,   0.000000,   0.000000,
      0.000000,   0.000000,   0.000000,   2.891030,   -6.250000,  -13.000000,
      0.000000,   0.000000,   0.000000,   38.613861,  -6.250000,  -13.000000,
      0.000000,   0.000000,   0.000000,   2.891030,   6.250000,   13.000000,
      0.000000,   0.000000,   0.000000,   38.613861,  6.250000,   13.000000,
      0.000000,   0.000000,   0.000000,   -6.250000,  2.891030,   -13.000000,
      0.000000,   0.000000,   0.000000,   6.250000,   2.891030,   13.000000,
      0.000000,   0.000000,   0.000000,   -6.250000,  38.613861,  -13.000000,
      0.000000,   0.000000,   0.000000,   6.250000,   38.613861,  13.000000,
      0.000000,   0.000000,   0.000000,   -13.000000, -13.000000, 1.278689,
      0.000000,   0.000000,   0.000000,   -13.000000, -13.000000, 6.000000,
      0.000000,   0.000000,   0.000000,   -13.000000, -13.000000, 6.000000,
      0.000000,   0.000000,   0.000000,   -13.000000, -13.000000, -2.228571,
      17.824750,  10.100000,  20.400625,  0.000000,   0.000000,   0.000000,
      3.935124,   10.100000,  20.400625,  0.000000,   0.000000,   0.000000,
      17.824750,  -10.100000, -20.400625, 0.000000,   0.000000,   0.000000,
      3.935124,   -10.100000, -20.400625, 0.000000,   0.000000,   0.000000,
      -10.100000, 3.935124,   20.400625,  0.000000,   0.000000,   0.000000,
      10.100000,  3.935124,   -20.400625, 0.000000,   0.000000,   0.000000,
      -10.100000, 17.824750,  20.400625,  0.000000,   0.000000,   0.000000,
      10.100000,  17.824750,  -20.400625, 0.000000,   0.000000,   0.000000,
      20.400625,  -20.400625, 7.000000,   0.000000,   0.000000,   0.000000,
      20.400625,  -20.400625, 1.869152,   0.000000,   0.000000,   0.000000,
      20.400625,  -20.400625, -4.011429,  0.000000,   0.000000,   0.000000,
      20.400625,  -20.400625, 7.000000,   0.000000,   0.000000,   0.000000,
      0.000000,   0.000000,   0.000000,   38.613861,  6.250000,   -13.000000,
      0.000000,   0.000000,   0.000000,   2.891030,   6.250000,   -13.000000,
      0.000000,   0.000000,   0.000000,   38.613861,  -6.250000,  13.000000,
      0.000000,   0.000000,   0.000000,   2.891030,   -6.250000,  13.000000,
      0.000000,   0.000000,   0.000000,   -6.250000,  2.891030,   -13.000000,
      0.000000,   0.000000,   0.000000,   6.250000,   2.891030,   13.000000,
      0.000000,   0.000000,   0.000000,   -6.250000,  38.613861,  -13.000000,
      0.000000,   0.000000,   0.000000,   6.250000,   38.613861,  13.000000,
      0.000000,   0.000000,   0.000000,   -13.000000, 13.000000,  6.000000,
      0.000000,   0.000000,   0.000000,   -13.000000, 13.000000,  1.278689,
      0.000000,   0.000000,   0.000000,   -13.000000, 13.000000,  -2.228571,
      0.000000,   0.000000,   0.000000,   -13.000000, 13.000000,  6.000000,
      3.935124,   -10.100000, 20.400625,  0.000000,   0.000000,   0.000000,
      17.824750,  -10.100000, 20.400625,  0.000000,   0.000000,   0.000000,
      3.935124,   10.100000,  -20.400625, 0.000000,   0.000000,   0.000000,
      17.824750,  10.100000,  -20.400625, 0.000000,   0.000000,   0.000000,
      10.100000,  17.824750,  20.400625,  0.000000,   0.000000,   0.000000,
      -10.100000, 17.824750,  -20.400625, 0.000000,   0.000000,   0.000000,
      10.100000,  3.935124,   20.400625,  0.000000,   0.000000,   0.000000,
      -10.100000, 3.935124,   -20.400625, 0.000000,   0.000000,   0.000000,
      -20.400625, 20.400625,  7.000000,   0.000000,   0.000000,   0.000000,
      -20.400625, 20.400625,  -4.011429,  0.000000,   0.000000,   0.000000,
      -20.400625, 20.400625,  1.869152,   0.000000,   0.000000,   0.000000,
      -20.400625, 20.400625,  7.000000,   0.000000,   0.000000,   0.000000,
      0.000000,   0.000000,   0.000000,   2.891030,   -6.250000,  -13.000000,
      0.000000,   0.000000,   0.000000,   38.613861,  -6.250000,  -13.000000,
      0.000000,   0.000000,   0.000000,   2.891030,   6.250000,   13.000000,
      0.000000,   0.000000,   0.000000,   38.613861,  6.250000,   13.000000,
      0.000000,   0.000000,   0.000000,   6.250000,   38.613861,  -13.000000,
      0.000000,   0.000000,   0.000000,   -6.250000,  38.613861,  13.000000,
      0.000000,   0.000000,   0.000000,   6.250000,   2.891030,   -13.000000,
      0.000000,   0.000000,   0.000000,   -6.250000,  2.891030,   13.000000,
      0.000000,   0.000000,   0.000000,   13.000000,  -13.000000, 6.000000,
      0.000000,   0.000000,   0.000000,   13.000000,  -13.000000, -2.228571,
      0.000000,   0.000000,   0.000000,   13.000000,  -13.000000, 1.278689,
      0.000000,   0.000000,   0.000000,   13.000000,  -13.000000, 6.000000,
      17.824750,  10.100000,  20.400625,  0.000000,   0.000000,   0.000000,
      3.935124,   10.100000,  20.400625,  0.000000,   0.000000,   0.000000,
      17.824750,  -10.100000, -20.400625, 0.000000,   0.000000,   0.000000,
      3.935124,   -10.100000, -20.400625, 0.000000,   0.000000,   0.000000,
      10.100000,  17.824750,  20.400625,  0.000000,   0.000000,   0.000000,
      -10.100000, 17.824750,  -20.400625, 0.000000,   0.000000,   0.000000,
      10.100000,  3.935124,   20.400625,  0.000000,   0.000000,   0.000000,
      -10.100000, 3.935124,   -20.400625, 0.000000,   0.000000,   0.000000,
      -20.400625, -20.400625, -4.011429,  0.000000,   0.000000,   0.000000,
      -20.400625, -20.400625, 7.000000,   0.000000,   0.000000,   0.000000,
      -20.400625, -20.400625, 7.000000,   0.000000,   0.000000,   0.000000,
      -20.400625, -20.400625, 1.869152,   0.000000,   0.000000,   0.000000,
      0.000000,   0.000000,   0.000000,   38.613861,  6.250000,   -13.000000,
      0.000000,   0.000000,   0.000000,   2.891030,   6.250000,   -13.000000,
      0.000000,   0.000000,   0.000000,   38.613861,  -6.250000,  13.000000,
      0.000000,   0.000000,   0.000000,   2.891030,   -6.250000,  13.000000,
      0.000000,   0.000000,   0.000000,   6.250000,   38.613861,  -13.000000,
      0.000000,   0.000000,   0.000000,   -6.250000,  38.613861,  13.000000,
      0.000000,   0.000000,   0.000000,   6.250000,   2.891030,   -13.000000,
      0.000000,   0.000000,   0.000000,   -6.250000,  2.891030,   13.000000,
      0.000000,   0.000000,   0.000000,   13.000000,  13.000000,  -2.228571,
      0.000000,   0.000000,   0.000000,   13.000000,  13.000000,  6.000000,
      0.000000,   0.000000,   0.000000,   13.000000,  13.000000,  6.000000,
      0.000000,   0.000000,   0.000000,   13.000000,  13.000000,  1.278689};
  // aris::dynamic::dsp(penetration_pairs.size() * 6, penetration_pairs.size() *
  // 6,
  //                    cpi.data());
  // aris::dynamic::dsp(penetration_pairs.size() * 6, penetration_pairs.size() *
  // 6,
  //                    matlab_result);
  // // 这里为了让手动输入的matlab结果和 c++的计算结果相等，只能取 1e-6的
  // tolerance for (int i = 0; i < 576; ++i) {
  //   std::cout << aris::dynamic::s_is_equal(1, matlab_result + i, cpi.data() +
  //   i,
  //                                          1e-6)
  //             << " " << matlab_result[i] << " " << cpi[i] << std::endl;
  // }
  EXPECT_TRUE(aris::dynamic::s_is_equal(576, matlab_result, cpi.data(), 1e-6));
}

TEST_F(TableSurfaceModelTestor, testNormalInertiaMatrixAndPostProcess) {
  // 第一个桌子的加速度矩阵对不上感觉是我们的关节使用的虽然是z轴向上的情况，但是数值上可能带来误差，
  // 尝试实现一下全锁住的约束来测试一下。
  // 整体公式记录
  // x(t) = e^(At)(x(0) - A\b) + A\b
  // 因为要每个接触点的法向的位移相减，同时，一组接触变量控制了两个物体的状态
  // 需要使用下面的矩阵对 I 进行缩小，并与接触点的刚度与阻尼相乘，得到矩阵 K D
  // T = [1 -1 0 ... 0 0  0]
  //     [0  0 1 -1 ...0  0]
  //     [.  . .  . ....  .]
  //     [0 0 0 0 .... 1 -1] n * 2n
  // K = T * I^-1 * T' * diag(k)  k 与 d 都是 1 * n 的向量
  // D = T * I^-1 * T' * diag(d)
  // 矩阵微分方程的状态转移矩阵 A 可以由三块组成，如下图所示
  // A = [0 I]
  //     [K D] 2n * 2n 的矩阵
  // 接触点状态如下
  // x = [d1 ... dn d1' ... dn'] 2 * n  d表示接触距离（穿深）
  // x' = [d1' ... dn' d1'' ... dn''] 2 * n
  // x(0) = [由积分截断时记录的 v 与 p 决定]
  // 其中，关于非齐次方程的常数项 b，需要经过如下计算
  // F = [FeA1 FeB1 ... FeAn FeBn] 1 * 2n vector
  // f = T * I^-1 * F' -> 1 * n vector
  // b = [0 f] -> 1 * 2n vector
  // 其中的 A \ b 方法 使用基于Householder方法的QR分解计算
  initTablePlaneModel();
  initPhysicsEngine();
  engine_->init(&table_model_);

  EXPECT_EQ(table_model_.forcePool().size(), 4);
  // ------------------------------------------------------
  // ----------- 无重力情况测试接触惯量矩阵 -----------------
  // ------------------------------------------------------
  sire::physics::contact::AverageForceContactSolver solver;
  solver.init(engine_.get());
  const sire::Size nContact = penetration_pairs.size();
  std::vector<double> cpi;
  std::vector<double> extInvCpi(64);
  double k[4]{2e8, 2e8, 2e8, 2e8};
  double d[4]{1e3, 1e3, 1e3, 1e3};
  // 计算 k 与 d 相乘后的矩阵，组成大矩阵 A 并与 matlab结果验证
  std::vector<double> A1(4 * nContact * nContact), b(2 * nContact);
  solver.cptContactPointNormalInertiaMatrix(penetration_pairs, T_vec, k, d, cpi,
                                            extInvCpi, A1);

  EXPECT_EQ(table_model_.forcePool().size(), 4);
  double matlab_result[]{
      1.869152, 0.000000,  7.000000, 0.000000, 7.000000, 0.000000, -4.011429,
      0.000000, 0.000000,  1.278689, 0.000000, 6.000000, 0.000000, 6.000000,
      0.000000, -2.228571, 7.000000, 0.000000, 1.869152, 0.000000, -4.011429,
      0.000000, 7.000000,  0.000000, 0.000000, 6.000000, 0.000000, 1.278689,
      0.000000, -2.228571, 0.000000, 6.000000, 7.000000, 0.000000, -4.011429,
      0.000000, 1.869152,  0.000000, 7.000000, 0.000000, 0.000000, 6.000000,
      0.000000, -2.228571, 0.000000, 1.278689, 0.000000, 6.000000, -4.011429,
      0.000000, 7.000000,  0.000000, 7.000000, 0.000000, 1.869152, 0.000000,
      0.000000, -2.228571, 0.000000, 6.000000, 0.000000, 6.000000, 0.000000,
      1.278689};
  // 这里为了让手动输入的matlab结果和 c++的计算结果相等，只能取 1e-6的
  EXPECT_TRUE(aris::dynamic::s_is_equal(4 * nContact * nContact, matlab_result,
                                        cpi.data(), 1e-6));
  // ------------------------------------------------------
  // ----------- 添加重力情况测试接触惯量矩阵 ---------------
  // ------------------------------------------------------
  double gravity[6]{0, 0, -9.8, 0, 0, 0};
  table_model_.environment().setGravity(gravity);
  table_model_.init();
  solver.cptContactPointNormalInertiaMatrix(penetration_pairs, T_vec, k, d, cpi,
                                            extInvCpi, A1);
  // 这里为了让手动输入的matlab结果和 c++的计算结果相等，只能取 1e-6的
  EXPECT_TRUE(aris::dynamic::s_is_equal(4 * nContact * nContact, matlab_result,
                                        cpi.data(), 1e-6));

  // ----------------------------------------------------
  // ------------ 接触点惯量矩阵求逆验证 ------------------
  // ----------------------------------------------------
  std::vector<double> inv_cpi(4 * nContact * nContact),
      u(4 * nContact * nContact), tau(2 * nContact), tau2(2 * nContact);
  std::vector<aris::Size> p(2 * nContact);
  aris::Size rank;
  aris::dynamic::s_householder_utp(2 * nContact, 2 * nContact, cpi.data(),
                                   u.data(), tau.data(), p.data(), rank);
  if (rank != 2 * nContact)
    THROW_FILE_LINE("Invalid singular matrix for normal contact inertia");

  aris::dynamic::s_householder_utp2pinv(2 * nContact, 2 * nContact, rank,
                                        u.data(), tau.data(), p.data(),
                                        inv_cpi.data(), tau2.data());

  double inv_cpi_matlab[] = {
      0.090622,  0.000000,  0.036571,  0.000000, 0.036571,  0.000000, -0.079430,
      -0.000000, 0.000000,  0.145880,  0.000000, 0.041929,  0.000000, 0.041929,
      0.000000,  -0.139242, 0.036571,  0.000000, 0.090622,  0.000000, -0.079430,
      -0.000000, 0.036571,  -0.000000, 0.000000, 0.041929,  0.000000, 0.145880,
      0.000000,  -0.139242, 0.000000,  0.041929, 0.036571,  0.000000, -0.079430,
      0.000000,  0.090622,  0.000000,  0.036571, -0.000000, 0.000000, 0.041929,
      -0.000000, -0.139242, 0.000000,  0.145880, 0.000000,  0.041929, -0.079430,
      0.000000,  0.036571,  0.000000,  0.036571, 0.000000,  0.090622, -0.000000,
      -0.000000, -0.139242, -0.000000, 0.041929, -0.000000, 0.041929, -0.000000,
      0.145880};
  // TODO(ltj): 数值上，最小只能取1e-6的error，会不会给后面的 e^AT
  // 带来误差的放大？
  EXPECT_TRUE(aris::dynamic::s_is_equal(4 * nContact * nContact, inv_cpi_matlab,
                                        inv_cpi.data(), 1e-6));
  EXPECT_TRUE(aris::dynamic::s_is_equal(4 * nContact * nContact, inv_cpi_matlab,
                                        extInvCpi.data(), 1e-6));
  // --------------------------------------------
  // T = [1 -1 0 ... 0 0  0]
  //     [0  0 1 -1 ...0  0]
  //     [.  . .  . ....  .]
  //     [0 0 0 0 .... 1 -1] n * 2n
  // shrinked inverse cpi matrix = T * I^-1 * T'
  // ------计算缩小后的 cpi 矩阵  n x n -----------
  // ---------------------------------------------
  std::vector<double> shrink_cpi(nContact * nContact);
  for (int i{0}; i < nContact; ++i) {
    for (int j{0}; j < nContact; ++j) {
      shrink_cpi[nContact * i + j] =
          inv_cpi[2 * nContact * 2 * i + 2 * j] +
          inv_cpi[2 * nContact * (2 * i + 1) + 2 * j + 1] -
          inv_cpi[2 * nContact * 2 * i + 2 * j + 1] -
          inv_cpi[2 * nContact * (2 * i + 1) + 2 * j];
    }
  }
  // aris::dynamic::dsp(4, 4, shrink_cpi.data());
  double shrink_i_matlab[16] = {0.236502,  0.078500,  0.078500,  -0.218672,
                                0.078500,  0.236502,  -0.218672, 0.078500,
                                0.078500,  -0.218672, 0.236502,  0.078500,
                                -0.218672, 0.078500,  0.078500,  0.236502};
  EXPECT_TRUE(aris::dynamic::s_is_equal(nContact * nContact, shrink_i_matlab,
                                        shrink_cpi.data(), 1e-7));
  // 计算 k 与 d 相乘后的矩阵，组成大矩阵 A 并与 matlab结果验证
  std::vector<double> k_dot(nContact * nContact), d_dot(nContact * nContact),
      A(4 * nContact * nContact);
  for (int i{0}; i < nContact; ++i) {
    for (int j{0}; j < nContact; ++j) {
      // 穿深加速度方向与接触力的方向永远反向
      k_dot[nContact * i + j] = -k[i] * shrink_cpi[nContact * i + j];
      d_dot[nContact * i + j] = -d[i] * shrink_cpi[nContact * i + j];
      A[2 * nContact * i + nContact + j] = (i == j);
      A[2 * nContact * (i + nContact) + j] = k_dot[nContact * i + j];
      A[2 * nContact * (i + nContact) + nContact + j] = d_dot[nContact * i + j];
    }
  }
  double k_dot_matlab[16] = {
      -47300413.271530, -15699995.366906, -15699995.366906, 43734403.156956,
      -15699995.366906, -47300413.271530, 43734403.156956,  -15699995.366906,
      -15699995.366906, 43734403.156956,  -47300413.271530, -15699995.366906,
      43734403.156956,  -15699995.366906, -15699995.366906, -47300413.271530};
  double d_dot_matlab[16] = {
      -236.502066, -78.499977, -78.499977, 218.672016, -78.499977,  -236.502066,
      218.672016,  -78.499977, -78.499977, 218.672016, -236.502066, -78.499977,
      218.672016,  -78.499977, -78.499977, -236.502066};
  double A_matlab[64] = {
      0.000000,         0.000000,         0.000000,         0.000000,
      1.000000,         0.000000,         0.000000,         0.000000,
      0.000000,         0.000000,         0.000000,         0.000000,
      0.000000,         1.000000,         0.000000,         0.000000,
      0.000000,         0.000000,         0.000000,         0.000000,
      0.000000,         0.000000,         1.000000,         0.000000,
      0.000000,         0.000000,         0.000000,         0.000000,
      0.000000,         0.000000,         0.000000,         1.000000,
      -47300413.271530, -15699995.366906, -15699995.366906, 43734403.156956,
      -236.502066,      -78.499977,       -78.499977,       218.672016,
      -15699995.366906, -47300413.271530, 43734403.156956,  -15699995.366906,
      -78.499977,       -236.502066,      218.672016,       -78.499977,
      -15699995.366906, 43734403.156956,  -47300413.271530, -15699995.366906,
      -78.499977,       218.672016,       -236.502066,      -78.499977,
      43734403.156956,  -15699995.366906, -15699995.366906, -47300413.271530,
      218.672016,       -78.499977,       -78.499977,       -236.502066};
  EXPECT_TRUE(aris::dynamic::s_is_equal(nContact * nContact, k_dot_matlab,
                                        k_dot.data(), 1e-6));
  EXPECT_TRUE(aris::dynamic::s_is_equal(nContact * nContact, d_dot_matlab,
                                        d_dot.data(), 1e-6));
  // aris::dynamic::dsp(8, 8, A.data());
  EXPECT_TRUE(aris::dynamic::s_is_equal(4 * nContact * nContact, A_matlab,
                                        A.data(), 1e-6));
  EXPECT_TRUE(aris::dynamic::s_is_equal(4 * nContact * nContact, A_matlab,
                                        A1.data(), 1e-6));
  aris::dynamic::s_nm(2 * nContact, 2 * nContact, 1e-5, A.data());
  std::vector<double> eAt(4 * nContact * nContact);
  sire::core::screw::matrix_exp_pade(2 * nContact, A.data(), eAt.data());
  double eAtMatlab[64] = {
      0.997641,    -0.000785,   -0.000785,   0.002182,    0.000010,
      -0.000000,   -0.000000,   0.000000,    -0.000785,   0.997641,
      0.002182,    -0.000785,   -0.000000,   0.000010,    0.000000,
      -0.000000,   -0.000785,   0.002182,    0.997641,    -0.000785,
      -0.000000,   0.000000,    0.000010,    -0.000000,   0.002182,
      -0.000785,   -0.000785,   0.997641,    0.000000,    -0.000000,
      -0.000000,   0.000010,    -471.073048, -156.953446, -156.953446,
      435.828965,  0.995285,    -0.001570,   -0.001570,   0.004361,
      -156.953446, -471.073048, 435.828965,  -156.953446, -0.001570,
      0.995285,    0.004361,    -0.001570,   -156.953446, 435.828965,
      -471.073048, -156.953446, -0.001570,   0.004361,    0.995285,
      -0.001570,   435.828965,  -156.953446, -156.953446, -471.073048,
      0.004361,    -0.001570,   -0.001570,   0.995285};
  EXPECT_TRUE(aris::dynamic::s_is_equal(2 * nContact, 2 * nContact, eAtMatlab,
                                        eAt.data(), 1e-6));

  aris::dynamic::s_nm(2 * nContact, 2 * nContact, 10, A.data());
  sire::core::screw::matrix_exp_pade(2 * nContact, A.data(), eAt.data());
  double expA_1e4_matlab[64] = {
      0.785846,     -0.077977,    -0.077977,    0.201294,     0.000092,
      -0.000003,    -0.000003,    0.000008,     -0.077977,    0.785846,
      0.201294,     -0.077977,    -0.000003,    0.000092,     0.000008,
      -0.000003,    -0.077977,    0.201294,     0.785846,     -0.077977,
      -0.000003,    0.000008,     0.000092,     -0.000003,    0.201294,
      -0.077977,    -0.077977,    0.785846,     0.000008,     -0.000003,
      -0.000003,    0.000092,     -3887.921009, -1550.604497, -1550.604497,
      3721.414081,  0.766406,     -0.085730,    -0.085730,    0.219901,
      -1550.604497, -3887.921009, 3721.414081,  -1550.604497, -0.085730,
      0.766406,     0.219901,     -0.085730,    -1550.604497, 3721.414081,
      -3887.921009, -1550.604497, -0.085730,    0.219901,     0.766406,
      -0.085730,    3721.414081,  -1550.604497, -1550.604497, -3887.921009,
      0.219901,     -0.085730,    -0.085730,    0.766406};
  // 由于验证时输入的matlab的矩阵的位数限制导致的误差较大为 1e-2 可以不管
  EXPECT_TRUE(aris::dynamic::s_is_equal(2 * nContact, 2 * nContact,
                                        expA_1e4_matlab, eAt.data(), 1e-2));
}

TEST_F(TableSurfaceModelTestor, testContactDiffEqnSolnCoeff) {
  initTablePlaneModel();
  initPhysicsEngine();
  engine_->init(&table_model_);
  // 正确设置重力加速度
  double gravity[6]{0, 0, -9.8, 0, 0, 0};
  table_model_.environment().setGravity(gravity);
  table_model_.init();
  EXPECT_EQ(table_model_.forcePool().size(), 4);
  const sire::Size nContact = penetration_pairs.size();
  std::vector<double> A(4 * nContact * nContact), b(2 * nContact),
      x0(2 * nContact), v0(3 * nContact);
  std::vector<double> invCpi;
  std::vector<double> stiffness(nContact, 2e8);
  std::vector<double> damping(nContact, 1e3);

  std::vector<sire::Size> preservedPairsIdx;
  std::vector<sire::PartId> prtIdVector;
  std::vector<double> accelExt;
  sire::physics::contact::filterPairsAndPreprocessInfo(
      *engine_, penetration_pairs, T_vec, preservedPairsIdx, prtIdVector,
      accelExt, invCpi);
  sire::Size n{preservedPairsIdx.size()};
  const sire::Size n2{2 * n};
  double stiffScale = sire::physics::contact::cptInitialCondition(
      *engine_, manager_, penetration_pairs, T_vec, preservedPairsIdx,
      stiffness.data(), damping.data(), x0.data(), v0.data());
  sire::physics::contact::cptDAECoeff(*engine_, n, stiffness.data(),
                                      damping.data(), 1e-4, accelExt.data(),
                                      invCpi.data(), A.data(), b.data());
  EXPECT_EQ(table_model_.forcePool().size(), 4);

  double A_matlab[64] = {0.0000000000,     0.0000000000,     0.0000000000,
                         0.0000000000,     10000.0000000000, 0.0000000000,
                         0.0000000000,     0.0000000000,     0.0000000000,
                         0.0000000000,     0.0000000000,     0.0000000000,
                         0.0000000000,     10000.0000000000, 0.0000000000,
                         0.0000000000,     0.0000000000,     0.0000000000,
                         0.0000000000,     0.0000000000,     0.0000000000,
                         0.0000000000,     10000.0000000000, 0.0000000000,
                         0.0000000000,     0.0000000000,     0.0000000000,
                         0.0000000000,     0.0000000000,     0.0000000000,
                         0.0000000000,     10000.0000000000, -26341.0654682368,
                         -6190.4761904762, -6190.4761904762, 13960.1130872844,
                         -1317.0532734118, -309.5238095238,  -309.5238095238,
                         698.0056543642,   -6190.4761904762, -26341.0654682368,
                         13960.1130872844, -6190.4761904762, -309.5238095238,
                         -1317.0532734118, 698.0056543642,   -309.5238095238,
                         -6190.4761904762, 13960.1130872844, -26341.0654682368,
                         -6190.4761904762, -309.5238095238,  698.0056543642,
                         -1317.0532734118, -309.5238095238,  13960.1130872844,
                         -6190.4761904762, -6190.4761904762, -26341.0654682368,
                         698.0056543642,   -309.5238095238,  -309.5238095238,
                         -1317.0532734118};

  double b_matlab[8] = {0.000000, 0.000000, 0.000000, 0.000000, 0, 0, 0, 0};
  EXPECT_TRUE(aris::dynamic::s_is_equal(4 * nContact * nContact, A_matlab,
                                        A.data(), 1e-6));
  EXPECT_TRUE(
      aris::dynamic::s_is_equal(2 * nContact, b_matlab, b.data(), 1e-6));

  double x00[8]{0, 0, 0, 0, 1, 1, 1, 1};
  std::copy(x00, x00 + 8, x0.data());

  EXPECT_TRUE(aris::dynamic::s_is_equal(
      sire::physics::contact::findMinRootBisection(nContact, A.data(), b.data(),
                                                   x0.data(), 1e-10, 1000),
      1.9988e-04, 1e-4));
}

auto applyForce(
    aris::dynamic::Model& model,
    const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    const std::vector<std::array<double, 16>>& T_C_vec,
    const std::vector<sire::Size>& preservedPairsIdx,
    const sire::PartId* prtIdVector, sire::Size cpiWidth, const int* groundFlag,
    double* accel) -> void {
  auto init_interaction = [](aris::dynamic::Interaction& interaction,
                             aris::dynamic::Model* m) -> void {
    if (interaction.prtNameM().empty() && interaction.prtNameN().empty() &&
        interaction.makNameI().empty() && interaction.makNameJ().empty())
      return;

    auto find_part = [m](std::string_view name) -> aris::dynamic::Part* {
      auto found = std::find_if(
          m->partPool().begin(), m->partPool().end(),
          [name](const auto& part) -> bool { return part.name() == name; });
      return found == m->partPool().end() ? nullptr : &*found;
    };

    auto find_marker = [](aris::dynamic::Part* part,
                          std::string_view name) -> aris::dynamic::Marker* {
      auto found = std::find_if(
          part->markerPool().begin(), part->markerPool().end(),
          [name](const auto& marker) -> bool { return marker.name() == name; });
      return found == part->markerPool().end() ? nullptr : &*found;
    };

    auto prt_m = find_part(interaction.prtNameM());
    auto mak_i = find_marker(prt_m, interaction.makNameI());
    auto prt_n = find_part(interaction.prtNameN());
    auto mak_j = find_marker(prt_n, interaction.makNameJ());

    interaction.setMakI(&*mak_i);
    interaction.setMakJ(&*mak_j);
  };
  // 初始化一些重复使用的变量
  auto& forcePool = model.forcePool();
  auto& partPool = model.partPool();
  sire::Size testForceIdxOffset = forcePool.size();
  for (auto& fce : forcePool) fce.activate(false);

  const sire::Size n = preservedPairsIdx.size();
  // 给力并计算质量矩阵
  double testFce1[3] = {0, 0, 1};
  const double* gravityAs = model.environment().gravity();
  // add generalForce to forcePool() in Model and init
  auto& fce = forcePool.add<aris::dynamic::GeneralForce>(
      std::string("test_f" + 0 ? "b"
                               : "a" + std::to_string(0 + testForceIdxOffset)),
      &partPool.at(prtIdVector[0]).markerPool().at(0),
      &partPool.at(model.ground().id()).markerPool().at(0));
  fce.resetModel(&model);
  fce.setFce(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());
  // force id 可以先不管
  init_interaction(fce, &model);

  auto& gf = dynamic_cast<aris::dynamic::GeneralForce&>(
      forcePool.at(testForceIdxOffset));
  double fs[6];
  sire::core::screw::s_fpm2fs(testFce1, T_C_vec[0].data(), fs);
  gf.setFce(fs);
  if (model.forwardDynamics())
    std::cout << "forward dynamic failed" << std::endl;

  for (sire::Size jContact{0}, cpiColIdx{0}; jContact < n; ++jContact) {
    double ap_o[3]{0}, res[3]{0};
    const double* contactPosition =
        penetration_pairs[preservedPairsIdx[jContact]].p_WC.data();
    std::array<double, 6> as;
    for (sire::Size j2 = 0; j2 < 2; ++j2) {
      if (groundFlag[2 * jContact + j2]) continue;
      auto& prt = partPool[prtIdVector[2 * jContact + j2]];
      prt.getAs(as.data());
      aris::dynamic::s_vs(6, gravityAs, as.data());
      aris::dynamic::s_as2ap(prt.vs(), as.data(), contactPosition, ap_o);
      aris::dynamic::s_inv_pm_dot_v3(
          T_C_vec[preservedPairsIdx[jContact]].data(), ap_o, res);
      accel[cpiColIdx] = res[2];
      ++cpiColIdx;
    }
  }
  aris::dynamic::s_fill(1, 6, 0, const_cast<double*>(gf.fce()));
  forcePool.pop_back();
  for (auto& fce : forcePool) fce.activate(true);
}

TEST_F(TableGroundModelTestor, testContactDiffEqnCoeffGround) {
  initTablePlaneModel();
  initPhysicsEngine();
  engine_->init(&table_model_);
  // 正确设置重力加速度
  double gravity[6]{0, 0, -9.8, 0, 0, 0};
  table_model_.environment().setGravity(gravity);
  table_model_.init();
  EXPECT_EQ(table_model_.forcePool().size(), 4);
  const sire::Size nContact = penetration_pairs.size();
  std::vector<double> A(4 * nContact * nContact), b(2 * nContact),
      x0(2 * nContact), v0(3 * nContact);
  std::vector<double> invCpi;
  std::vector<double> accelExt;
  std::vector<double> stiffness(nContact, 2e8);
  std::vector<double> damping(nContact, 1e3);
  std::vector<sire::Size> preservedPairsIdx;
  std::vector<sire::PartId> prtIdVector;
  sire::physics::contact::filterPairsAndPreprocessInfo(
      *engine_, penetration_pairs, T_vec, preservedPairsIdx, prtIdVector,
      accelExt, invCpi);
  sire::Size n{preservedPairsIdx.size()};
  const sire::Size n2 = 2 * n;
  double stiffScale = sire::physics::contact::cptInitialCondition(
      *engine_, manager_, penetration_pairs, T_vec, preservedPairsIdx,
      stiffness.data(), damping.data(), x0.data(), v0.data());
  sire::physics::contact::cptDAECoeff(*engine_, n, stiffness.data(),
                                      damping.data(), 1e-4, accelExt.data(),
                                      invCpi.data(), A.data(), b.data());
  EXPECT_EQ(table_model_.forcePool().size(), 4);

  double A_matlab[64] = {0.0000000000,     0.0000000000,     0.0000000000,
                         0.0000000000,     10000.0000000000, 0.0000000000,
                         0.0000000000,     0.0000000000,     0.0000000000,
                         0.0000000000,     0.0000000000,     0.0000000000,
                         0.0000000000,     10000.0000000000, 0.0000000000,
                         0.0000000000,     0.0000000000,     0.0000000000,
                         0.0000000000,     0.0000000000,     0.0000000000,
                         0.0000000000,     10000.0000000000, 0.0000000000,
                         0.0000000000,     0.0000000000,     0.0000000000,
                         0.0000000000,     0.0000000000,     0.0000000000,
                         0.0000000000,     10000.0000000000, -10700.0398272112,
                         -2857.1428571429, -2857.1428571429, 4985.7541129255,
                         -535.0019913606,  -142.8571428571,  -142.8571428571,
                         249.2877056463,   -2857.1428571429, -10700.0398272112,
                         4985.7541129255,  -2857.1428571429, -142.8571428571,
                         -535.0019913606,  249.2877056463,   -142.8571428571,
                         -2857.1428571429, 4985.7541129255,  -10700.0398272112,
                         -2857.1428571429, -142.8571428571,  249.2877056463,
                         -535.0019913606,  -142.8571428571,  4985.7541129255,
                         -2857.1428571429, -2857.1428571429, -10700.0398272112,
                         249.2877056463,   -142.8571428571,  -142.8571428571,
                         -535.0019913606};
  double b_matlab[8] = {0.000000, 0.000000, 0.000000, 0.000000,
                        9.8,      9.8,      9.8,      9.8};
  EXPECT_TRUE(aris::dynamic::s_is_equal(4 * nContact * nContact, A_matlab,
                                        A.data(), 1e-6));
  EXPECT_TRUE(
      aris::dynamic::s_is_equal(2 * nContact, b_matlab, b.data(), 1e-6));
}

TEST_F(TableSurfaceModelTestor, testDepthFormulaAndAvgFce) {
  initTablePlaneModel();
  initPhysicsEngine();
  engine_->init(&table_model_);
  EXPECT_EQ(table_model_.forcePool().size(), 4);
  const sire::Size nContact = penetration_pairs.size();
  std::vector<double> stiffness(nContact, 2e8);
  std::vector<double> damping(nContact, 1e3);
  std::vector<double> invCpi(4 * nContact * nContact);
  std::vector<double> A(4 * nContact * nContact), b(2 * nContact),
      x0(2 * nContact), v0(3 * nContact);
  std::vector<sire::Size> preservedPairsIdx;
  std::vector<sire::PartId> prtIdVector;
  std::vector<double> accelExt;
  sire::physics::contact::filterPairsAndPreprocessInfo(
      *engine_, penetration_pairs, T_vec, preservedPairsIdx, prtIdVector,
      accelExt, invCpi);
  sire::Size n{preservedPairsIdx.size()};
  const sire::Size n2 = 2 * n;
  double stiffScale = sire::physics::contact::cptInitialCondition(
      *engine_, manager_, penetration_pairs, T_vec, preservedPairsIdx,
      stiffness.data(), damping.data(), x0.data(), v0.data());
  sire::physics::contact::cptDAECoeff(
      *engine_, n, stiffness.data(), damping.data(), stiffScale,
      accelExt.data(), invCpi.data(), A.data(), b.data());
  EXPECT_EQ(table_model_.forcePool().size(), 4);
  // std::vector<double> x0(2 * nContact, 0);
  for (int i{0}; i < nContact; ++i) {
    x0[i + nContact] = 1;
    x0[i] = 0;
  }
  double avg_fce_matlab15[4] = {-7.7016028645, -7.7016028645, -7.7016028645,
                                -7.7016028645};
  double avg_fce_matlab05[4] = {1671.3984737647, 1671.3984737647,
                                1671.3984737647, 1671.3984737647};
  double avg_fce_matlab06[4] = {2274.2736529136, 2274.2736529136,
                                2274.2736529136, 2274.2736529136};
  double avg_fce_matlab58[4] = {-1733.1725730484, -1733.1725730484,
                                -1733.1725730484, -1733.1725730484};
  std::vector<double> avgFce(nContact);
  sire::physics::contact::cptAvgContactFce(
      nContact, A.data(), b.data(), x0.data(), 0.0001, 0.0005, stiffness.data(),
      damping.data(), avgFce.data());
  EXPECT_TRUE(aris::dynamic::s_is_equal(nContact, avgFce.data(),
                                        avg_fce_matlab15, 1e-6));
  sire::physics::contact::cptAvgContactFce(
      nContact, A.data(), b.data(), x0.data(), 0.0000, 0.0005, stiffness.data(),
      damping.data(), avgFce.data());
  EXPECT_TRUE(aris::dynamic::s_is_equal(nContact, avgFce.data(),
                                        avg_fce_matlab05, 1e-6));
  sire::physics::contact::cptAvgContactFce(
      nContact, A.data(), b.data(), x0.data(), 0.0000, 0.0006, stiffness.data(),
      damping.data(), avgFce.data());
  EXPECT_TRUE(aris::dynamic::s_is_equal(nContact, avgFce.data(),
                                        avg_fce_matlab06, 1e-6));
  sire::physics::contact::cptAvgContactFce(
      nContact, A.data(), b.data(), x0.data(), 0.0005, 0.0008, stiffness.data(),
      damping.data(), avgFce.data());
  EXPECT_TRUE(aris::dynamic::s_is_equal(nContact, avgFce.data(),
                                        avg_fce_matlab58, 1e-6));
}

TEST_F(TableSurfaceModelTestor, testExternalForceGravity) {
  initTablePlaneModel();
  initPhysicsEngine();
  engine_->init(&table_model_);
  double cpi[]{1.869152,  0.000000,  7.000000,  0.000000,  7.000000,  0.000000,
               -4.011429, 0.000000,  0.000000,  1.278689,  0.000000,  6.000000,
               0.000000,  6.000000,  0.000000,  -2.228571, 7.000000,  0.000000,
               1.869152,  0.000000,  -4.011429, 0.000000,  7.000000,  0.000000,
               0.000000,  6.000000,  0.000000,  1.278689,  0.000000,  -2.228571,
               0.000000,  6.000000,  7.000000,  0.000000,  -4.011429, 0.000000,
               1.869152,  0.000000,  7.000000,  0.000000,  0.000000,  6.000000,
               0.000000,  -2.228571, 0.000000,  1.278689,  0.000000,  6.000000,
               -4.011429, 0.000000,  7.000000,  0.000000,  7.000000,  0.000000,
               1.869152,  0.000000,  0.000000,  -2.228571, 0.000000,  6.000000,
               0.000000,  6.000000,  0.000000,  1.278689};
  sire::Size nContact = penetration_pairs.size();
  std::vector<double> result(2 * nContact), b(2 * nContact);
  sire::physics::contact::AverageForceContactSolver solver;
  solver.init(engine_.get());
  std::vector<double> extInvCpi(4 * nContact * nContact, 0);
  solver.cptContactPrtExtForce(penetration_pairs, T_vec, cpi, 8,
                               extInvCpi.data(), result.data(), b.data());
  EXPECT_TRUE(
      sire::core::screw::s_is_zeros(1, 2 * nContact, result.data(), 1e-6));

  // Modify gravity and init model
  double gravity[6] = {0, 0, -9.8, 0, 0, 0};
  table_model_.environment().setGravity(gravity);
  table_model_.init();
  solver.cptContactPrtExtForce(penetration_pairs, T_vec, cpi, 8,
                               extInvCpi.data(), result.data(), b.data());
  double fext_table_matlab[8] = {-18.317689, -12.531148, -18.317689,
                                 -12.531148, -18.317689, -12.531148,
                                 -18.317689, -12.531148};
  EXPECT_TRUE(aris::dynamic::s_is_equal(1, 2 * nContact, fext_table_matlab,
                                        result.data(), 1e-5));
}

TEST_F(TableSurfaceModelTestor, testAddJointAfterInit) {
  // 第一个桌子的加速度矩阵对不上感觉是我们的关节使用的虽然是z轴向上的情况，但是数值上可能带来误差，
  // 尝试实现一下全锁住的约束来测试一下。
  initTablePlaneModel();
  initPhysicsEngine();
  engine_->init(&table_model_);
  std::cout << "Using Model Init" << std::endl;
  addFixedJointUsingModelInit();
  std::cout << "Using Init Interaction" << std::endl;
  addFixedJointUsingInitInteraction();
};

// 老师的正解认为，一个机器人的正解，那些限制住的自由度不会被修改，所以当两个不同
// 位置的FixedJoint加入的时候，因为没有自由度可以调整，所以杆件位置不会调整
// 如果有需要在没有自由度情况下依旧要根据最小二乘调整的需求，那么就不能使用这个方法
// 就看新加入的约束的可信度，如果认为和机器人的连接具有相同的可信度，那么就需要一个
// 全局的最小二乘，但是因为我们的接触约束不是真正的刚性约束，所以只需要一个在关节
// 空间上的最小二乘解即可，而不是真正的融合了接触位置的最小二乘解。
TEST_F(TableSurfaceModelTestor, testFKQP) {
  // 第一个桌子的加速度矩阵对不上感觉是我们的关节使用的虽然是z轴向上的情况，但是数值上可能带来误差，
  // 尝试实现一下全锁住的约束来测试一下。
  initTablePlaneModel();
  initPhysicsEngine();
  engine_->init(&table_model_);
  addFixedJointAtOtherCorner2TestFK();
}

// 测试一下，同时加入FixedJoint和PrismaticJoint在不同的激活状态下进行速度与位置的约束，
// 得用两次 allocateMemory，可以通过设置activate状态更改模型
TEST_F(SurfaceBallConstraintTester, testFKPosVel) {
  initTablePlaneModel();
  initPhysicsEngine();
  engine_->init(&model_);
  addFixedAndPrismaticJointFKPosVel();
}

TEST_F(SurfaceBallConstraintTester, testSingularCpi) {
  double cpi[4] = {6, 6, 6, 6};
  double kdMatrix[] = {-2e8, 0, -1000, 0, 0, -2e8, 0, -1000};
  double fext[] = {9.8, 9.8};
  std::vector<double> invCpi(4);
  using VType = sire::physics::contact::LhsVariableType;
  std::vector<VType> variableType{VType::OneDelta, VType::OneDelta};
  sire::physics::contact::cptInvCpi(2, 2, 1000, variableType.data(), cpi,
                                    kdMatrix, fext, invCpi.data());
  double fextTrue[] = {0, 9.8};
  double cpiTrue[] = {1, -1, 6, 6};
  double invCpiTrue[] = {0.5, 0.083333333, -0.5, 0.0833333333};
  double kdMatrixTrue[] = {0, 0, -200000, 200000, 0, -2e8, 0, -1000};
  EXPECT_TRUE(aris::dynamic::s_is_equal(2, fext, fextTrue, 1e-6));
  EXPECT_TRUE(aris::dynamic::s_is_equal(4, cpi, cpiTrue, 1e-3));
  EXPECT_TRUE(aris::dynamic::s_is_equal(4, kdMatrix, kdMatrixTrue, 1e-6));
  EXPECT_TRUE(aris::dynamic::s_is_equal(4, invCpi.data(), invCpiTrue, 1e-6));

  double A[] = {0, 0,         1,      0,     0, 0,         0,     1,
                0, -20000000, -80000, 79900, 0, -40000000, 40000, -40200};
  double b[] = {0, 0, 98000, 196000};
  std::vector<double> u(16), tau(4);
  std::vector<aris::Size> p(4);
  aris::Size rank;
  std::vector<double> x(4, 0);
  aris::dynamic::s_householder_utp(4, 4, A, u.data(), tau.data(), p.data(),
                                   rank);
  aris::dynamic::s_householder_utp_sov(4, 4, 1, rank, u.data(), tau.data(),
                                       p.data(), b, x.data());
  // aris::dynamic::dsp(1, 4, x.data());

  double A1[] = {0, 1, -4.66667e7, -166.6667};
  double b1[] = {0, 9.81};
  double x0[] = {0.145071, 4.63032};
  std::cout << sire::physics::contact::findMinRootBisection(1, A1, b1, x0,
                                                            1e-10, 1000)
            << std::endl;

  double A2[] = {0,
                 0,
                 0,
                 1,
                 0,
                 0,
                 0,
                 0,
                 0,
                 0,
                 1,
                 0,
                 0,
                 0,
                 0,
                 0,
                 0,
                 1,
                 5.18371e+10,
                 -5.20867e+10,
                 1.74055e+09,
                 1.29593e+06,
                 -1.30217e+06,
                 43513.7,
                 -5.20867e+10,
                 5.24294e+10,
                 -2.0478e+09,
                 -1.30217e+06,
                 1.31074e+06,
                 -51195,
                 1.74055e+09,
                 -2.0478e+09,
                 6.63074e+08,
                 43513.7,
                 -51195,
                 16576.9};
  double b2[] = {0, 0, 0, -64.4455, 55.5192, 35.808};
  double x2[] = {1.25176e-05, 3.98723e-07, 1.71304e-05,
                 0.208605,    0.223518,    -0.586628};
  std::cout << floor(std::log10(5.18371e+10)) / 3 << std::endl;
  std::cout << sire::physics::contact::findMinRootBisection(3, A2, b2, x2,
                                                            1e-10, 1000)
            << std::endl;

  double pm1[4][4] = {{-0.93279817297397949, 0.34190109496275456,
                       -0.11397898823764327, 0.17200357909272626},
                      {0.034180587799043365, -0.23090537993752255,
                       -0.97237564394302722, 0.092802268472739868},
                      {-0.35877465896313776, -0.91092609292915316,
                       0.20370173614062048, 0.17006813632165846},
                      {0.0000000000000000, 0.0000000000000000,
                       0.0000000000000000, 1.0000000000000000}};
  double vs1[6] = {-1.7206322213052863, -0.36094840408769835,
                   1.2104823963209037,  -8.6479603610514211,
                   5.7185843656418323,  -6.3471434195933734};
  double as1[6] = {13.514310707764245, 32.761516894772015, -36.210360941517116,
                   384.55564205828915, 9.1142464015863975, 164.21333007218732};

  double pm2[4][4] = {{-0.85665777934389364, 0.50313640231045953,
                       -0.11397898823764435, -0.18588761137084839},
                      {-0.0076453636225875176, -0.23329628261388499,
                       -0.97237564394303044, 0.098109313308031035},
                      {-0.51582845743974970, -0.83212174901791136,
                       0.20370173614062226, -0.0048524869589190714},
                      {0.0000000000000000, 0.0000000000000000,
                       0.0000000000000000, 1.0000000000000000}};
  double vs2[6] = {-0.62507186600704367, -0.45968186279873730,
                   1.4474964558104080,   -9.8484413731187654,
                   -3.2001984972580009,  -4.2877102809656487};
  double as2[6] = {29.559416659007027, 29.573619762580602,  -25.550286162575727,
                   334.54714555937488, -9.6199902617835225, 257.45947213736497};

  double pm3[4][4] = {{0.94370701759755271, -0.31052512809345101,
                       -0.11397898823764457, -0.16847776372011658},
                      {-0.041880995866660206, 0.22963359782835913,
                       -0.97237564394303411, 0.088204041410181497},
                      {0.32812047653620879, 0.92241127246524535,
                       0.20370173614062256, -0.042394083139627267},
                      {0.0000000000000000, 0.0000000000000000,
                       0.0000000000000000, 1.0000000000000000}};
  double vs3[6] = {-0.97497984605664900, 0.36181249273000265,
                   3.1197996779703878,   -10.223085139153113,
                   -8.3272251755709874,  -1.9313671303585085};
  double as3[6] = {30.399508148396507, 52.130662257118914,  -16.326014350065570,
                   300.12446649768037, -26.599097124164700, 315.50866580057198};
}
