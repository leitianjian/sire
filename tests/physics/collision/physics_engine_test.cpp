#include "sire/physics/physics_engine.hpp"

#include <gtest/gtest.h>
#include <hpp/fcl/data_types.h>

#include <aris/dynamic/math_matrix.hpp>

#include "sire/core/constants.hpp"
#include "sire/core/geometry/geometry_base.hpp"
#include "sire/core/sire_assert.hpp"
#include "sire/physics/collision/penetration_as_point_pair_callback.hpp"
#include "sire/physics/common/penetration_as_point_pair.hpp"
#include "sire/physics/geometry/sphere_collision_geometry.hpp"

using namespace sire::geometry;
using namespace sire::physics;
using namespace hpp::fcl;
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
    const PenetrationAsPointPair& penetration =
        penetration_results[0];

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
    Vec3f p_WCo{radius_, 0, 0};
    // Contact point on the colliding_sphere1.
    Vec3f p_WCc{colliding_x_ - radius_, 0, 0};
    expected.p_WCa = origin_is_A ? p_WCo : p_WCc;
    expected.p_WCb = origin_is_A ? p_WCc : p_WCo;
    Vec3f norm_into_B = Vec3f::UnitX();
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

  unique_ptr<PhysicsEngine> engine_{std::make_unique<PhysicsEngine>()};
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
    Vec3f p_WCo1{radius_, 0, 0};
    // Contact point on the colliding_sphere1.
    Vec3f p_WCc1{colliding_x1_ - radius_, 0, 0};
    expected1.p_WCa = p_WCo1;
    expected1.p_WCb = p_WCc1;
    Vec3f norm_into_B = Vec3f::UnitX();
    expected1.nhat_AB_W = norm_into_B;

    // Setting expected penetration result of geometry 0 and 2
    PenetrationAsPointPair expected2;
    expected2.id_A = origin_sphere;
    expected2.id_B = colliding_sphere2;
    expected2.depth = 2 * radius_ - colliding_y1_;
    // Contact point on the origin_sphere.
    Vec3f p_WCo2{0, radius_, 0};
    // Contact point on the colliding_sphere2.
    Vec3f p_WCc2{0, colliding_x1_ - radius_, 0};
    expected2.p_WCa = p_WCo2;
    expected2.p_WCb = p_WCc2;
    Vec3f norm_into_C = Vec3f::UnitY();
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
    Vec3f p_WCo1{radius_, 0, 0};
    // Contact point on the colliding_sphere1.
    Vec3f p_WCc1{colliding_x1_ - radius_, 0, 0};
    expected1.p_WCa = p_WCo1;
    expected1.p_WCb = p_WCc1;
    Vec3f norm_into_B = Vec3f::UnitX();
    expected1.nhat_AB_W = norm_into_B;

    // Setting expected penetration result of geometry 0 and 2
    PenetrationAsPointPair expected2;
    expected2.id_A = origin_sphere;
    expected2.id_B = colliding_sphere2;
    expected2.depth = 0.375;
    // Contact point on the origin_sphere.
    Vec3f p_WCo2{0.3, 0.4, 0};
    // Contact point on the colliding_sphere2.
    Vec3f p_WCc2{0.075, 0.1, 0};
    expected2.p_WCa = p_WCo2;
    expected2.p_WCb = p_WCc2;
    Vec3f norm_into_C1{0.6, 0.8, 0};
    expected2.nhat_AB_W = norm_into_C1;

    // Setting expected penetration result of geometry 1 and 2
    PenetrationAsPointPair expected3;
    expected3.id_A = colliding_sphere1;
    expected3.id_B = colliding_sphere2;
    expected3.depth = 0.375;
    // Contact point on the colliding_sphere1.
    Vec3f p_WCo3{0.45, 0.4, 0};
    // Contact point on the colliding_sphere2.
    Vec3f p_WCc3{0.675, 0.1, 0};
    expected3.p_WCa = p_WCo3;
    expected3.p_WCb = p_WCc3;
    Vec3f norm_into_C2{-0.6, 0.8, 0};
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

TEST_F(SimplePenetrationTest, PenetrationDynamicAndAnchored) {
  // Setup collision detection engine
  collision_engine_ = new collision::CollisionDetection();
  engine_->resetCollisionDetection(collision_engine_);
  engine_->setCollisionDetectionFlag(true);
  engine_->addSphereGeometry(radius_, 0, default_pm, false);
  engine_->addSphereGeometry(radius_, 1, default_pm, true);
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
  engine_->addSphereGeometry(radius_, 0, default_pm, false);
  engine_->addSphereGeometry(radius_, 1, default_pm, true);
  engine_->addSphereGeometry(radius_, 2, default_pm, true);
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