#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <set>

#include <gtest/gtest.h>

#include <coal/collision_object.h>
#include <coal/collision.h>
#include <coal/contact_patch.h>
#include <coal/hfield.h>
#include <coal/shape/geometric_shapes.h>

#include "sire/physics/collision/height_field_contact_generator.hpp"
#include "sire/physics/collision/height_field_contact_callback.hpp"

namespace {

auto flatTerrain(int grid_size) -> coal::CollisionObject {
  coal::MatrixXs heights = coal::MatrixXs::Zero(grid_size, grid_size);
  return coal::CollisionObject(
      std::make_shared<coal::HeightField<coal::AABB>>(1.0, 1.0, heights,
                                                     -1.0));
}

auto boxAt(const coal::Vec3s& center) -> coal::CollisionObject {
  return coal::CollisionObject(std::make_shared<coal::Box>(0.6, 0.4, 0.2),
                               coal::Transform3s(center));
}

TEST(HeightFieldWitnessTest, SphereWitnessStaysOnSphereAcrossCellDiagonal) {
  auto terrain = flatTerrain(2);
  constexpr coal::Scalar radius = 0.25;
  const coal::Vec3s center(0.1, -0.1, 0.2);
  coal::CollisionObject sphere(std::make_shared<coal::Sphere>(radius),
                               coal::Transform3s(center));

  const auto contacts =
      sire::physics::collision::generateHeightFieldContacts(terrain, sphere, 8);
  ASSERT_EQ(contacts.size(), 1u);
  const auto& contact = contacts.front();
  EXPECT_NEAR(contact.nearest_points[0].z(), 0.0, 1e-12);
  EXPECT_NEAR((contact.nearest_points[1] - center).norm(), radius, 1e-12);
  EXPECT_NEAR(contact.penetration_depth, -0.05, 1e-12);
  EXPECT_NEAR((contact.nearest_points[1] - contact.nearest_points[0])
                  .dot(contact.normal),
              contact.penetration_depth, 1e-12);
}

TEST(HeightFieldWitnessTest, FlatBoxKeepsFourSpreadSupportVertices) {
  for (int grid_size : {2, 5}) {
    SCOPED_TRACE(grid_size);
    auto terrain = flatTerrain(grid_size);
    auto box = boxAt(coal::Vec3s(0.0, 0.0, 0.05));
    const auto contacts = sire::physics::collision::generateHeightFieldContacts(
        terrain, box, 8);

    ASSERT_EQ(contacts.size(), 4u);
    for (int x_sign : {-1, 1}) {
      for (int y_sign : {-1, 1}) {
        const coal::Vec3s expected(0.3 * x_sign, 0.2 * y_sign, -0.05);
        bool found = false;
        for (const auto& contact : contacts) {
          if ((contact.nearest_points[1] - expected).norm() < 1e-9) {
            found = true;
            EXPECT_NEAR(contact.nearest_points[0].z(), 0.0, 1e-12);
            EXPECT_NEAR(contact.penetration_depth, -0.05, 1e-12);
            EXPECT_NEAR(contact.normal.z(), 1.0, 1e-12);
          }
        }
        EXPECT_TRUE(found) << "missing box support vertex "
                           << expected.transpose();
      }
    }
  }
}

TEST(HeightFieldWitnessTest, UnmodifiedCoalExposesOnlyLeafWitnesses) {
  auto terrain = flatTerrain(5);
  auto box = boxAt(coal::Vec3s(0, 0, 0.05));
  coal::CollisionRequest collision_request;
  collision_request.num_max_contacts = 128;
  coal::CollisionResult collision_result;
  coal::collide(&terrain, &box, collision_request, collision_result);
  ASSERT_GT(collision_result.numContacts(), 0u);
  coal::ContactPatchRequest patch_request(128);
  coal::ContactPatchResult patch_result(patch_request);
  coal::computeContactPatch(&terrain, &box, collision_result, patch_request,
                            patch_result);
  ASSERT_GT(patch_result.numContactPatches(), 0u);
  for (std::size_t i = 0; i < patch_result.numContactPatches(); ++i) {
    const auto& patch = patch_result.getContactPatch(i);
    EXPECT_EQ(patch.size(), 1u);
  }
}

TEST(HeightFieldWitnessTest, CoalLeafPatchQueryReusesLeafContacts) {
  auto terrain = flatTerrain(5);
  auto box = boxAt(coal::Vec3s(0, 0, 0.05));
  coal::CollisionRequest request;
  request.num_max_contacts = 512;
  request.enable_height_field_contact_patches = true;
  coal::CollisionResult result;
  coal::collide(&terrain, &box, request, result);

  ASSERT_GE(result.numContacts(), 4u);
  coal::Scalar min_x = std::numeric_limits<coal::Scalar>::infinity();
  coal::Scalar max_x = -min_x;
  coal::Scalar min_y = min_x;
  coal::Scalar max_y = -min_x;
  for (const auto& contact : result.getContacts()) {
    const auto& shape_witness = contact.nearest_points[1];
    EXPECT_NEAR(contact.nearest_points[0].z(), 0.0, 1e-6);
    EXPECT_NEAR(shape_witness.z(), -0.05, 1e-6);
    EXPECT_NEAR((shape_witness - contact.nearest_points[0]).dot(contact.normal),
                contact.penetration_depth, 1e-7);
    min_x = std::min(min_x, shape_witness.x());
    max_x = std::max(max_x, shape_witness.x());
    min_y = std::min(min_y, shape_witness.y());
    max_y = std::max(max_y, shape_witness.y());
  }
  EXPECT_GE(max_x - min_x, 0.4);
  EXPECT_GE(max_y - min_y, 0.2);
}

TEST(HeightFieldWitnessTest, CoalLeafPatchQueryDoesNotMultiplyFlatSphere) {
  auto terrain = flatTerrain(9);
  coal::CollisionObject sphere(std::make_shared<coal::Sphere>(0.2),
                               coal::Transform3s(coal::Vec3s(0.03, -0.04, 0.19)));
  coal::CollisionRequest request;
  request.num_max_contacts = 512;
  request.enable_height_field_contact_patches = true;
  coal::CollisionResult result;
  coal::collide(&terrain, &sphere, request, result);

  ASSERT_EQ(result.numContacts(), 1u);
  EXPECT_NEAR(result.getContact(0).nearest_points[0].z(), 0.0, 1e-6);
  EXPECT_NEAR((result.getContact(0).nearest_points[1] -
               coal::Vec3s(0.03, -0.04, 0.19)).norm(),
              0.2, 1e-5);
}

TEST(HeightFieldWitnessTest, CoalLeafPatchKeepsCapsuleWitnessOnSurface) {
  auto terrain = flatTerrain(17);
  coal::CollisionObject capsule(
      std::make_shared<coal::Capsule>(0.08, 0.16),
      coal::Transform3s(coal::Vec3s(0.02, -0.03, 0.155)));
  coal::CollisionRequest request;
  request.num_max_contacts = 512;
  request.enable_height_field_contact_patches = true;
  coal::CollisionResult result;
  coal::collide(&terrain, &capsule, request, result);

  ASSERT_GT(result.numContacts(), 0u);
  for (const auto& contact : result.getContacts()) {
    const auto local =
        capsule.getTransform().inverseTransform(contact.nearest_points[1]);
    const double axial = std::max(0.0, std::abs(local.z()) - 0.08);
    const double radius = std::hypot(local.head<2>().norm(), axial);
    EXPECT_NEAR(contact.nearest_points[0].z(), 0.0, 1e-6);
    EXPECT_NEAR(radius, 0.08, 1e-5);
    EXPECT_NEAR((contact.nearest_points[1] -
                 contact.nearest_points[0]).dot(contact.normal),
                contact.penetration_depth, 1e-6);
  }
}

TEST(HeightFieldWitnessTest, CoalLeafPatchKeepsHorizontalCapsuleSupports) {
  auto terrain = flatTerrain(17);
  const coal::Matrix3s rotation =
      Eigen::AngleAxisd(0.5 * std::acos(-1.0), coal::Vec3s::UnitY())
          .toRotationMatrix();
  coal::CollisionObject capsule(
      std::make_shared<coal::Capsule>(0.08, 0.16),
      coal::Transform3s(rotation, coal::Vec3s(0.02, -0.03, 0.075)));
  coal::CollisionRequest request;
  request.num_max_contacts = 512;
  request.enable_height_field_contact_patches = true;
  coal::CollisionResult result;
  coal::collide(&terrain, &capsule, request, result);

  ASSERT_GE(result.numContacts(), 2u);
  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -min_x;
  for (const auto& contact : result.getContacts()) {
    const auto local =
        capsule.getTransform().inverseTransform(contact.nearest_points[1]);
    const double axial = std::max(0.0, std::abs(local.z()) - 0.08);
    EXPECT_NEAR(std::hypot(local.head<2>().norm(), axial), 0.08, 1e-5);
    EXPECT_NEAR(contact.nearest_points[0].z(), 0.0, 1e-6);
    min_x = std::min(min_x, contact.nearest_points[1].x());
    max_x = std::max(max_x, contact.nearest_points[1].x());
  }
  EXPECT_GE(max_x - min_x, 0.15);
}

TEST(HeightFieldWitnessTest, CoalLeafPatchKeepsCurvedExteriorSideContact) {
  auto terrain = flatTerrain(2);
  for (bool capsule_shape : {false, true}) {
    SCOPED_TRACE(capsule_shape ? "capsule" : "sphere");
    std::shared_ptr<coal::CollisionGeometry> geometry =
        capsule_shape
            ? std::static_pointer_cast<coal::CollisionGeometry>(
                  std::make_shared<coal::Capsule>(0.2, 0.16))
            : std::static_pointer_cast<coal::CollisionGeometry>(
                  std::make_shared<coal::Sphere>(0.2));
    coal::CollisionObject shape(
        geometry, coal::Transform3s(coal::Vec3s(0.65, 0, -0.1)));
    coal::CollisionRequest request;
    request.num_max_contacts = 512;
    request.enable_height_field_contact_patches = true;
    coal::CollisionResult result;
    coal::collide(&terrain, &shape, request, result);
    ASSERT_GT(result.numContacts(), 0u);
    bool found_side = false;
    for (const auto& contact : result.getContacts())
      if (contact.normal.x() > 0.9) {
        found_side = true;
        EXPECT_NEAR(contact.nearest_points[0].x(), 0.5, 1e-6);
        EXPECT_NEAR(contact.penetration_depth, -0.05, 1e-6);
      }
    EXPECT_TRUE(found_side);
  }
}

TEST(HeightFieldWitnessTest, CoalLeafPatchKeepsCylinderFootOnSurface) {
  auto terrain = flatTerrain(17);
  coal::CollisionObject cylinder(
      std::make_shared<coal::Cylinder>(0.08, 0.16),
      coal::Transform3s(coal::Vec3s(0.02, -0.03, 0.075)));
  coal::CollisionRequest request;
  request.num_max_contacts = 512;
  request.enable_height_field_contact_patches = true;
  coal::CollisionResult result;
  coal::collide(&terrain, &cylinder, request, result);

  ASSERT_GE(result.numContacts(), 2u);
  for (const auto& contact : result.getContacts()) {
    const auto local =
        cylinder.getTransform().inverseTransform(contact.nearest_points[1]);
    EXPECT_NEAR(contact.nearest_points[0].z(), 0.0, 1e-6);
    EXPECT_NEAR(local.z(), -0.08, 1e-5);
    EXPECT_LE(local.head<2>().norm(), 0.08 + 1e-5);
    EXPECT_NEAR((contact.nearest_points[1] -
                 contact.nearest_points[0]).dot(contact.normal),
                contact.penetration_depth, 1e-6);
  }
}

TEST(HeightFieldWitnessTest, CoalLeafPatchKeepsCurvedWitnessesOnSlope) {
  coal::MatrixXs heights(17, 17);
  for (int row = 0; row < 17; ++row)
    for (int col = 0; col < 17; ++col)
      heights(row, col) = 0.1 * (-0.5 + col / 16.0);
  coal::CollisionObject terrain(
      std::make_shared<coal::HeightField<coal::AABB>>(1.0, 1.0, heights,
                                                     -1.0));
  for (bool capsule_shape : {false, true}) {
    SCOPED_TRACE(capsule_shape ? "capsule" : "cylinder");
    std::shared_ptr<coal::CollisionGeometry> geometry =
        capsule_shape
            ? std::static_pointer_cast<coal::CollisionGeometry>(
                  std::make_shared<coal::Capsule>(0.08, 0.16))
            : std::static_pointer_cast<coal::CollisionGeometry>(
                  std::make_shared<coal::Cylinder>(0.08, 0.16));
    coal::CollisionObject shape(
        geometry, coal::Transform3s(coal::Vec3s(
                      0.03, -0.04, capsule_shape ? 0.155 : 0.075)));
    coal::CollisionRequest request;
    request.num_max_contacts = 512;
    request.enable_height_field_contact_patches = true;
    coal::CollisionResult result;
    coal::collide(&terrain, &shape, request, result);
    ASSERT_GT(result.numContacts(), 0u);
    for (const auto& contact : result.getContacts()) {
      const auto local =
          shape.getTransform().inverseTransform(contact.nearest_points[1]);
      EXPECT_NEAR(contact.nearest_points[0].z(),
                  0.1 * contact.nearest_points[0].x(), 1e-5);
      if (capsule_shape) {
        const double axial = std::max(0.0, std::abs(local.z()) - 0.08);
        EXPECT_NEAR(std::hypot(local.head<2>().norm(), axial), 0.08, 1e-4);
      } else {
        const double radial = local.head<2>().norm();
        EXPECT_LE(radial, 0.08 + 1e-4);
        EXPECT_LE(std::abs(local.z()), 0.08 + 1e-4);
        EXPECT_LE(std::min(std::abs(radial - 0.08),
                           std::abs(std::abs(local.z()) - 0.08)),
                  1e-4);
      }
      EXPECT_NEAR((contact.nearest_points[1] -
                   contact.nearest_points[0]).dot(contact.normal),
                  contact.penetration_depth, 1e-5);
    }
  }
}

TEST(HeightFieldWitnessTest, GenericPatchGeneratorKeepsFlatBoxCorners) {
  auto terrain = flatTerrain(5);
  auto box = boxAt(coal::Vec3s(0, 0, 0.05));
  const auto contacts =
      sire::physics::collision::generateHeightFieldPatchContacts(terrain, box,
                                                                 4);
  ASSERT_EQ(contacts.size(), 4u);
  for (int x_sign : {-1, 1})
    for (int y_sign : {-1, 1}) {
      const coal::Vec3s expected(0.3 * x_sign, 0.2 * y_sign, -0.05);
      bool found = false;
      for (const auto& contact : contacts)
        found |= (contact.nearest_points[1] - expected).norm() < 1e-6;
      EXPECT_TRUE(found) << expected.transpose();
    }
}

TEST(HeightFieldWitnessTest, GenericPatchRespectsRotatedTerrainFrame) {
  auto terrain = flatTerrain(9);
  const coal::Matrix3s rotation =
      Eigen::AngleAxisd(0.2, coal::Vec3s::UnitY()).toRotationMatrix();
  const coal::Transform3s terrain_pose(rotation, coal::Vec3s(1, 2, 0.3));
  terrain.setTransform(terrain_pose);
  coal::CollisionObject box(
      std::make_shared<coal::Box>(0.6, 0.4, 0.2),
      terrain_pose * coal::Transform3s(coal::Vec3s(0, 0, 0.05)));
  const auto contacts =
      sire::physics::collision::generateHeightFieldPatchContacts(terrain, box,
                                                                 4);
  ASSERT_EQ(contacts.size(), 4u);
  for (const auto& contact : contacts) {
    const coal::Vec3s terrain_local =
        terrain_pose.inverseTransform(contact.nearest_points[0]);
    const coal::Vec3s shape_local =
        terrain_pose.inverseTransform(contact.nearest_points[1]);
    EXPECT_NEAR(terrain_local.z(), 0.0, 1e-6);
    EXPECT_NEAR(shape_local.z(), -0.05, 1e-6);
    EXPECT_NEAR(contact.normal.dot(rotation * coal::Vec3s::UnitZ()), 1.0,
                1e-6);
  }
}

TEST(HeightFieldWitnessTest, GenericPatchGeneratorHandlesCapsule) {
  auto terrain = flatTerrain(17);
  coal::CollisionObject capsule(std::make_shared<coal::Capsule>(0.08, 0.16),
                                 coal::Transform3s(coal::Vec3s(0, 0, 0.075)));
  const auto contacts =
      sire::physics::collision::generateHeightFieldPatchContacts(
          terrain, capsule, 4);
  ASSERT_FALSE(contacts.empty());
  for (const auto& contact : contacts) {
    EXPECT_LT(contact.penetration_depth, 0);
    EXPECT_NEAR(contact.nearest_points[0].z(), 0, 1e-6);
    EXPECT_NEAR((contact.nearest_points[1] - contact.nearest_points[0])
                    .dot(contact.normal),
                contact.penetration_depth, 1e-6);
  }
}

TEST(HeightFieldWitnessTest, GenericPatchGeneratorHandlesCylinderFoot) {
  auto terrain = flatTerrain(17);
  coal::CollisionObject cylinder(
      std::make_shared<coal::Cylinder>(0.08, 0.16),
      coal::Transform3s(coal::Vec3s(0, 0, 0.075)));
  const auto contacts =
      sire::physics::collision::generateHeightFieldPatchContacts(
          terrain, cylinder, 4);
  ASSERT_GE(contacts.size(), 2u);
  for (const auto& contact : contacts) {
    EXPECT_LT(contact.penetration_depth, 0);
    EXPECT_NEAR(contact.nearest_points[0].z(), 0, 1e-6);
    EXPECT_NEAR(contact.nearest_points[1].z(), -0.005, 1e-4);
  }
}

TEST(HeightFieldWitnessTest, ManifoldKeepsIdsAcrossHeightFieldCells) {
  auto terrain = flatTerrain(17);
  auto box = boxAt(coal::Vec3s(0, 0, 0.05));
  sire::physics::collision::HeightFieldContactManifold manifold;
  manifold.beginQuery();
  const auto first = manifold.select(
      terrain, box,
      sire::physics::collision::generateHeightFieldPatchContacts(terrain, box,
                                                                 8),
      8);
  manifold.endQuery();
  ASSERT_EQ(first.size(), 4u);
  box.setTransform(coal::Transform3s(coal::Vec3s(0.01, 0, 0.05)));
  manifold.beginQuery();
  const auto second = manifold.select(
      terrain, box,
      sire::physics::collision::generateHeightFieldPatchContacts(terrain, box,
                                                                 8),
      8);
  manifold.endQuery();
  ASSERT_EQ(second.size(), 4u);
  for (const auto& contact : first) {
    bool found = false;
    for (const auto& updated : second)
      found |= updated.contact_id == contact.contact_id &&
               updated.patch_id == contact.patch_id;
    EXPECT_TRUE(found);
  }
}

TEST(HeightFieldWitnessTest, ManifoldRefreshesMissingSingleAnchor) {
  auto terrain = flatTerrain(17);
  coal::CollisionObject sphere(std::make_shared<coal::Sphere>(0.05),
                               coal::Transform3s(coal::Vec3s(0, 0, 0.049)));
  auto candidates = sire::physics::collision::generateHeightFieldContacts(
      terrain, sphere, 4);
  ASSERT_EQ(candidates.size(), 1u);
  sire::physics::collision::HeightFieldContactManifold manifold;
  manifold.beginQuery();
  const auto first = manifold.select(terrain, sphere, candidates, 4);
  manifold.endQuery();
  ASSERT_EQ(first.size(), 1u);

  manifold.beginQuery();
  const auto refreshed = manifold.select(terrain, sphere, {}, 4);
  manifold.endQuery();
  ASSERT_EQ(refreshed.size(), 1u);
  EXPECT_EQ(refreshed[0].contact_id, first[0].contact_id);
  EXPECT_EQ(refreshed[0].patch_id, first[0].patch_id);

  sphere.setTransform(coal::Transform3s(coal::Vec3s(0, 0, 0.3)));
  manifold.beginQuery();
  const auto separated = manifold.select(terrain, sphere, {}, 4);
  manifold.endQuery();
  EXPECT_TRUE(separated.empty());
}

TEST(HeightFieldWitnessTest, ManifoldKeepsBoxSupportsDuringContinuousCellSweep) {
  auto terrain = flatTerrain(33);
  auto box = boxAt(coal::Vec3s(-0.12, 0, 0.05));
  sire::physics::collision::HeightFieldContactManifold manifold;
  std::set<std::uint64_t> original_contacts;
  std::uint64_t original_patch = 0;
  for (int sample = 0; sample <= 80; ++sample) {
    box.setTransform(coal::Transform3s(
        coal::Vec3s(-0.12 + 0.003 * sample, 0, 0.05)));
    manifold.beginQuery();
    const auto selected = manifold.select(
        terrain, box,
        sire::physics::collision::generateHeightFieldPatchContacts(
            terrain, box, 32),
        8);
    manifold.endQuery();
    ASSERT_EQ(selected.size(), 4u) << "sample " << sample;
    std::set<std::uint64_t> contacts;
    for (const auto& contact : selected) {
      contacts.insert(contact.contact_id);
      if (sample == 0)
        original_patch = contact.patch_id;
      else
        EXPECT_EQ(contact.patch_id, original_patch) << "sample " << sample;
    }
    if (sample == 0)
      original_contacts = contacts;
    else
      EXPECT_EQ(contacts, original_contacts) << "sample " << sample;
  }
}

TEST(HeightFieldWitnessTest, CurvedShapesStayValidAcrossCellsAndTilts) {
  auto terrain = flatTerrain(33);
  coal::CollisionRequest request;
  request.num_max_contacts = 512;
  request.enable_height_field_contact_patches = true;
  for (bool capsule_shape : {false, true}) {
    for (double angle : {0.0, 0.25, 0.5, 1.0}) {
      for (int sample = 0; sample <= 28; ++sample) {
        SCOPED_TRACE(capsule_shape ? "capsule" : "cylinder");
        SCOPED_TRACE(angle);
        SCOPED_TRACE(sample);
        std::shared_ptr<coal::CollisionGeometry> geometry =
            capsule_shape
                ? std::static_pointer_cast<coal::CollisionGeometry>(
                      std::make_shared<coal::Capsule>(0.08, 0.16))
                : std::static_pointer_cast<coal::CollisionGeometry>(
                      std::make_shared<coal::Cylinder>(0.08, 0.16));
        const coal::Matrix3s rotation =
            Eigen::AngleAxisd(angle, coal::Vec3s::UnitY()).toRotationMatrix();
        const double center_z =
            capsule_shape
                ? 0.075 + 0.08 * std::abs(std::cos(angle))
                : 0.075 + 0.08 * (std::abs(std::cos(angle)) +
                                  std::abs(std::sin(angle)) - 1.0);
        coal::CollisionObject shape(
            geometry,
            coal::Transform3s(rotation,
                              coal::Vec3s(-0.35 + 0.025 * sample, -0.03,
                                          center_z)));
        coal::CollisionResult result;
        coal::collide(&terrain, &shape, request, result);
        ASSERT_GT(result.numContacts(), 0u);
        for (const auto& contact : result.getContacts()) {
          const coal::Vec3s local =
              shape.getTransform().inverseTransform(contact.nearest_points[1]);
          EXPECT_NEAR(contact.nearest_points[0].z(), 0.0, 1e-5);
          if (capsule_shape) {
            const double axial =
                std::max(0.0, std::abs(local.z()) - 0.08);
            EXPECT_NEAR(std::hypot(local.head<2>().norm(), axial), 0.08,
                        1e-4);
          } else {
            const double radial = local.head<2>().norm();
            EXPECT_LE(radial, 0.08 + 1e-4);
            EXPECT_LE(std::abs(local.z()), 0.08 + 1e-4);
            EXPECT_LE(std::min(std::abs(radial - 0.08),
                               std::abs(std::abs(local.z()) - 0.08)),
                      1e-4);
          }
          EXPECT_NEAR((contact.nearest_points[1] -
                       contact.nearest_points[0]).dot(contact.normal),
                      contact.penetration_depth, 1e-5);
        }
      }
    }
  }
}

TEST(HeightFieldWitnessTest, CylinderPatchIdsRemainStableDuringCellSweep) {
  auto terrain = flatTerrain(33);
  coal::CollisionObject cylinder(
      std::make_shared<coal::Cylinder>(0.05, 0.1),
      coal::Transform3s(coal::Vec3s(-0.12, -0.03, 0.049)));
  coal::CollisionRequest request;
  request.num_max_contacts = 512;
  request.enable_height_field_contact_patches = true;
  sire::physics::collision::HeightFieldContactManifold manifold;
  std::uint64_t patch_id = 0;
  std::set<std::uint64_t> prior_ids;
  std::size_t total_id_changes = 0;
  for (int sample = 0; sample <= 80; ++sample) {
    cylinder.setTransform(coal::Transform3s(
        coal::Vec3s(-0.12 + 0.003 * sample, -0.03, 0.049)));
    coal::CollisionResult result;
    coal::collide(&terrain, &cylinder, request, result);
    manifold.beginQuery();
    const auto selected = manifold.select(terrain, cylinder,
                                           result.getContacts(), 8);
    manifold.endQuery();
    ASSERT_GE(selected.size(), 2u) << "sample " << sample;
    ASSERT_LE(selected.size(), 4u) << "sample " << sample;
    std::set<std::uint64_t> ids;
    coal::Vec3s support_centroid = coal::Vec3s::Zero();
    for (const auto& contact : selected) {
      ids.insert(contact.contact_id);
      support_centroid += contact.contact.nearest_points[1];
      if (sample == 0)
        patch_id = contact.patch_id;
      else
        EXPECT_EQ(contact.patch_id, patch_id) << "sample " << sample;
    }
    support_centroid /= static_cast<double>(selected.size());
    const coal::Vec3s centroid_local =
        cylinder.getTransform().inverseTransform(support_centroid);
    EXPECT_LE(centroid_local.head<2>().norm(), 0.005) << "sample " << sample;
    if (sample > 0) {
      for (const auto id : ids)
        total_id_changes += prior_ids.count(id) == 0;
    }
    prior_ids = std::move(ids);
  }
  // A curved rim can replace its support set at cell boundaries, but the
  // replacement must remain occasional rather than happen every frame.
  EXPECT_LE(total_id_changes, 32u);
}

TEST(HeightFieldWitnessTest, ManifoldSplitsOnlyAfterNormalHysteresis) {
  auto terrain = flatTerrain(9);
  auto box = boxAt(coal::Vec3s(0, 0, 0.05));
  auto candidates =
      sire::physics::collision::generateHeightFieldPatchContacts(terrain, box,
                                                                 4);
  ASSERT_EQ(candidates.size(), 4u);
  sire::physics::collision::HeightFieldContactManifold manifold;
  manifold.beginQuery();
  const auto first = manifold.select(terrain, box, candidates, 8);
  manifold.endQuery();
  ASSERT_EQ(first.size(), 4u);
  const auto original_id = first.front().patch_id;
  const auto normal_at = [](double degrees) {
    const double radians = degrees * 3.14159265358979323846 / 180.0;
    return coal::Vec3s(std::sin(radians), 0, std::cos(radians));
  };
  candidates.front().normal = normal_at(9);
  manifold.beginQuery();
  const auto retained = manifold.select(terrain, box, candidates, 8);
  manifold.endQuery();
  ASSERT_EQ(retained.size(), 4u);
  for (const auto& contact : retained)
    EXPECT_EQ(contact.patch_id, original_id);
  candidates.front().normal = normal_at(30);
  manifold.beginQuery();
  const auto split = manifold.select(terrain, box, candidates, 8);
  manifold.endQuery();
  ASSERT_EQ(split.size(), 4u);
  std::size_t original_count = 0;
  for (const auto& contact : split)
    original_count += contact.patch_id == original_id;
  EXPECT_GT(original_count, 0u);
  EXPECT_LT(original_count, 4u);
}

TEST(HeightFieldWitnessTest, ManifoldSeparatesParallelContactLevels) {
  auto terrain = flatTerrain(17);
  auto box = boxAt(coal::Vec3s(0, 0, 0.05));
  std::vector<coal::Contact> candidates;
  for (double level : {0.0, 0.03}) {
    for (double x : {-0.1, 0.1}) {
      const coal::Vec3s terrain_point(x, 0, level);
      const coal::Vec3s normal = coal::Vec3s::UnitZ();
      candidates.emplace_back(
          terrain.collisionGeometry().get(), box.collisionGeometry().get(),
          coal::Contact::NONE, coal::Contact::NONE, terrain_point,
          terrain_point - 0.001 * normal, normal, -0.001);
    }
  }
  sire::physics::collision::HeightFieldContactManifold manifold;
  manifold.beginQuery();
  const auto selected = manifold.select(terrain, box, candidates, 8);
  manifold.endQuery();
  std::set<std::uint64_t> patches;
  for (const auto& contact : selected) patches.insert(contact.patch_id);
  EXPECT_EQ(patches.size(), 2u);
}

TEST(HeightFieldWitnessTest, ManifoldKeepsSmoothNormalFanTogether) {
  auto terrain = flatTerrain(17);
  auto box = boxAt(coal::Vec3s(0, 0, 0.05));
  std::vector<coal::Contact> candidates;
  const coal::Vec3s terrain_point(0, 0, 0);
  for (double degrees : {0.0, 7.0, 14.0}) {
    const double radians = degrees * std::acos(-1.0) / 180.0;
    const coal::Vec3s normal(std::sin(radians), 0, std::cos(radians));
    candidates.emplace_back(
        terrain.collisionGeometry().get(), box.collisionGeometry().get(),
        coal::Contact::NONE, coal::Contact::NONE, terrain_point,
        terrain_point - 0.001 * normal, normal, -0.001);
  }
  sire::physics::collision::HeightFieldContactManifold manifold;
  manifold.beginQuery();
  const auto selected = manifold.select(terrain, box, candidates, 8);
  manifold.endQuery();
  std::set<std::uint64_t> patches;
  for (const auto& contact : selected) patches.insert(contact.patch_id);
  EXPECT_EQ(patches.size(), 1u);
}

TEST(HeightFieldWitnessTest, ManifoldSplitsSharpNormalEdge) {
  auto terrain = flatTerrain(17);
  auto box = boxAt(coal::Vec3s(0, 0, 0.05));
  std::vector<coal::Contact> candidates;
  const coal::Vec3s terrain_point(0, 0, 0);
  for (double degrees : {0.0, 30.0}) {
    const double radians = degrees * std::acos(-1.0) / 180.0;
    const coal::Vec3s normal(std::sin(radians), 0, std::cos(radians));
    candidates.emplace_back(
        terrain.collisionGeometry().get(), box.collisionGeometry().get(),
        coal::Contact::NONE, coal::Contact::NONE, terrain_point,
        terrain_point - 0.001 * normal, normal, -0.001);
  }
  sire::physics::collision::HeightFieldContactManifold manifold;
  manifold.beginQuery();
  const auto selected = manifold.select(terrain, box, candidates, 8);
  manifold.endQuery();
  std::set<std::uint64_t> patches;
  for (const auto& contact : selected) patches.insert(contact.patch_id);
  EXPECT_EQ(patches.size(), 2u);
}

TEST(HeightFieldWitnessTest, ManifoldPrioritizesDeepPatchCoverage) {
  auto terrain = flatTerrain(17);
  auto box = boxAt(coal::Vec3s(0, 0, 0.05));
  std::vector<coal::Contact> candidates;
  for (double level : {0.0, 0.03, 0.06}) {
    for (double x : {-0.1, 0.0, 0.1}) {
      const coal::Vec3s terrain_point(x, 0, level);
      const coal::Vec3s normal = coal::Vec3s::UnitZ();
      candidates.emplace_back(
          terrain.collisionGeometry().get(), box.collisionGeometry().get(),
          coal::Contact::NONE, coal::Contact::NONE, terrain_point,
          terrain_point - 0.001 * normal, normal, -0.001);
    }
  }
  sire::physics::collision::HeightFieldContactManifold manifold;
  manifold.beginQuery();
  const auto selected = manifold.select(terrain, box, candidates, 4);
  manifold.endQuery();
  ASSERT_EQ(selected.size(), 4u);
  std::set<std::uint64_t> patches;
  for (const auto& contact : selected) patches.insert(contact.patch_id);
  EXPECT_EQ(patches.size(), 2u);
}

TEST(HeightFieldWitnessTest, BoxLargerThanTerrainStillGetsSpreadContacts) {
  auto terrain = flatTerrain(2);
  coal::CollisionObject box(std::make_shared<coal::Box>(2.0, 2.0, 0.2),
                            coal::Transform3s(coal::Vec3s(0, 0, 0.05)));
  const auto contacts =
      sire::physics::collision::generateHeightFieldContacts(terrain, box, 8);
  ASSERT_GE(contacts.size(), 4u);
  for (const auto& contact : contacts) {
    EXPECT_NEAR(contact.nearest_points[0].z(), 0.0, 1e-12);
    EXPECT_NEAR(contact.nearest_points[1].z(), -0.05, 1e-12);
  }
}

TEST(HeightFieldWitnessTest, SeparatedBoxHasNoPrismFaceContact) {
  auto terrain = flatTerrain(3);
  auto box = boxAt(coal::Vec3s(0.0, 0.0, 0.25));
  const auto contacts =
      sire::physics::collision::generateHeightFieldContacts(terrain, box, 8);
  EXPECT_TRUE(contacts.empty());
}

TEST(HeightFieldWitnessTest, RotatedTerrainPreservesWorldWitnesses) {
  auto terrain = flatTerrain(3);
  const coal::Matrix3s rotation =
      Eigen::AngleAxisd(0.2, coal::Vec3s::UnitY()).toRotationMatrix();
  const coal::Transform3s terrain_pose(rotation, coal::Vec3s(1.0, 2.0, 0.3));
  terrain.setTransform(terrain_pose);
  const coal::Transform3s local_box_pose(coal::Vec3s(0.0, 0.0, 0.05));
  coal::CollisionObject box(std::make_shared<coal::Box>(0.6, 0.4, 0.2),
                            terrain_pose * local_box_pose);

  const auto contacts =
      sire::physics::collision::generateHeightFieldContacts(terrain, box, 8);
  ASSERT_EQ(contacts.size(), 4u);
  for (const auto& contact : contacts) {
    const coal::Vec3s local_terrain =
        terrain_pose.inverseTransform(contact.nearest_points[0]);
    const coal::Vec3s local_box =
        terrain_pose.inverseTransform(contact.nearest_points[1]);
    EXPECT_NEAR(local_terrain.z(), 0.0, 1e-12);
    EXPECT_NEAR(local_box.z(), -0.05, 1e-12);
    EXPECT_NEAR(contact.penetration_depth, -0.05, 1e-12);
    EXPECT_NEAR(contact.normal.dot(rotation * coal::Vec3s::UnitZ()), 1.0,
                1e-12);
  }
}

TEST(HeightFieldWitnessTest, SphereCanTouchExteriorSide) {
  auto terrain = flatTerrain(2);
  const std::array<coal::Vec3s, 4> directions{
      coal::Vec3s(1, 0, 0), coal::Vec3s(-1, 0, 0),
      coal::Vec3s(0, 1, 0), coal::Vec3s(0, -1, 0)};
  for (const coal::Vec3s& direction : directions) {
    const coal::Vec3s center = 0.65 * direction +
                               coal::Vec3s(0.0, 0.0, -0.1);
    coal::CollisionObject sphere(std::make_shared<coal::Sphere>(0.2),
                                 coal::Transform3s(center));
    const auto contacts = sire::physics::collision::generateHeightFieldContacts(
        terrain, sphere, 8);
    ASSERT_FALSE(contacts.empty());
    bool found_side = false;
    for (const auto& contact : contacts) {
      if (contact.normal.dot(direction) > 0.9) {
        found_side = true;
        EXPECT_NEAR(contact.nearest_points[0].dot(direction), 0.5, 1e-12);
        EXPECT_NEAR((contact.nearest_points[1] - center).norm(), 0.2, 1e-12);
        EXPECT_NEAR(contact.penetration_depth, -0.05, 1e-12);
      }
    }
    EXPECT_TRUE(found_side) << "direction " << direction.transpose();
  }
}

TEST(HeightFieldWitnessTest, RollingSphereKeepsOneFlatGroundContact) {
  auto terrain = flatTerrain(9);
  for (int sample = -7; sample <= 7; ++sample) {
    SCOPED_TRACE(sample);
    const coal::Vec3s center(0.05 * sample, 0.0, 0.19);
    coal::CollisionObject sphere(std::make_shared<coal::Sphere>(0.2),
                                 coal::Transform3s(center));
    const auto contacts =
        sire::physics::collision::generateHeightFieldContacts(terrain, sphere,
                                                               8);
    ASSERT_EQ(contacts.size(), 1u);
    EXPECT_NEAR(contacts[0].nearest_points[0].x(), center.x(), 1e-12);
    EXPECT_NEAR(contacts[0].nearest_points[0].y(), center.y(), 1e-12);
    EXPECT_NEAR(contacts[0].normal.z(), 1.0, 1e-12);
    EXPECT_NEAR(contacts[0].penetration_depth, -0.01, 1e-12);
  }
}

TEST(HeightFieldWitnessTest, RollingSphereKeepsContactOnRoughCells) {
  constexpr int grid_size = 33;
  coal::MatrixXs heights(grid_size, grid_size);
  for (int row = 0; row < grid_size; ++row) {
    for (int col = 0; col < grid_size; ++col) {
      const double x = -0.5 + static_cast<double>(col) / (grid_size - 1);
      heights(row, col) = 0.01 * std::sin(4 * std::acos(-1.0) * x);
    }
  }
  coal::CollisionObject terrain(
      std::make_shared<coal::HeightField<coal::AABB>>(1.0, 1.0, heights,
                                                     -1.0));
  constexpr double radius = 0.05;
  for (int sample = 0; sample <= 120; ++sample) {
    SCOPED_TRACE(sample);
    const double x = -0.4 + 0.8 * sample / 120.0;
    const double grid_coordinate = (x + 0.5) * (grid_size - 1);
    const int col = std::clamp(static_cast<int>(std::floor(grid_coordinate)),
                               0, grid_size - 2);
    const double alpha = grid_coordinate - col;
    const double h0 = heights(0, col), h1 = heights(0, col + 1);
    const double height = h0 + alpha * (h1 - h0);
    const double slope = (h1 - h0) * (grid_size - 1);
    const coal::Vec3s normal = coal::Vec3s(-slope, 0, 1).normalized();
    const coal::Vec3s terrain_point(x, 0, height);
    const coal::Vec3s center = terrain_point + (radius - 1e-4) * normal;
    coal::CollisionObject sphere(std::make_shared<coal::Sphere>(radius),
                                 coal::Transform3s(center));
    const auto contacts = sire::physics::collision::generateHeightFieldContacts(
        terrain, sphere, 8);
    ASSERT_GE(contacts.size(), 1u);
    ASSERT_LE(contacts.size(), 2u);
    for (const auto& contact : contacts) {
      EXPECT_NEAR((contact.nearest_points[1] - center).norm(), radius, 1e-8);
      EXPECT_NEAR((contact.nearest_points[1] -
                   contact.nearest_points[0]).dot(contact.normal),
                  contact.penetration_depth, 1e-9);
    }
  }
}

TEST(HeightFieldWitnessTest, SphereKeepsBothSharpRidgeSupports) {
  coal::MatrixXs heights = coal::MatrixXs::Zero(33, 33);
  heights(16, 15) = 0.15;
  heights(16, 17) = 0.15;
  coal::CollisionObject terrain(
      std::make_shared<coal::HeightField<coal::AABB>>(1.0, 1.0, heights,
                                                     -0.02));
  coal::CollisionObject sphere(
      std::make_shared<coal::Sphere>(0.05),
      coal::Transform3s(coal::Vec3s(0, 0, 0.18)));
  const auto contacts = sire::physics::collision::generateHeightFieldContacts(
      terrain, sphere, 4);
  ASSERT_GE(contacts.size(), 2u);
  bool negative_x = false, positive_x = false;
  for (const auto& contact : contacts) {
    negative_x |= contact.nearest_points[0].x() < -0.02;
    positive_x |= contact.nearest_points[0].x() > 0.02;
    EXPECT_NEAR((contact.nearest_points[1] -
                 coal::Vec3s(0, 0, 0.18)).norm(), 0.05, 1e-8);
    EXPECT_LT(contact.penetration_depth, 0);
  }
  EXPECT_TRUE(negative_x);
  EXPECT_TRUE(positive_x);
}

TEST(HeightFieldWitnessTest, SlopedSurfaceKeepsGeometricWitnesses) {
  coal::MatrixXs heights(2, 2);
  heights << 0.0, 0.1, 0.0, 0.1;
  coal::CollisionObject terrain(
      std::make_shared<coal::HeightField<coal::AABB>>(1.0, 1.0, heights,
                                                     -1.0));
  const coal::Vec3s center(0.0, 0.0, 0.2);
  coal::CollisionObject sphere(std::make_shared<coal::Sphere>(0.2),
                               coal::Transform3s(center));
  const auto contacts =
      sire::physics::collision::generateHeightFieldContacts(terrain, sphere, 8);
  ASSERT_EQ(contacts.size(), 1u);
  EXPECT_LT(contacts[0].normal.x(), -0.09);
  EXPECT_GT(contacts[0].normal.z(), 0.99);
  EXPECT_NEAR((contacts[0].nearest_points[1] - center).norm(), 0.2, 1e-12);
  EXPECT_NEAR((contacts[0].nearest_points[1] - contacts[0].nearest_points[0])
                  .dot(contacts[0].normal),
              contacts[0].penetration_depth, 1e-12);
}

TEST(HeightFieldWitnessTest, RoughTerrainBoxFaceDoesNotDisappear) {
  constexpr int grid_size = 65;
  coal::MatrixXs heights(grid_size, grid_size);
  const auto height = [](double x, double y) {
    return 0.03 + 0.006 * std::sin(4 * x) + 0.004 * std::cos(3 * y);
  };
  for (int row = 0; row < grid_size; ++row)
    for (int col = 0; col < grid_size; ++col)
      heights(row, col) = height(-2 + col * 4.0 / (grid_size - 1),
                                 2 - row * 4.0 / (grid_size - 1));
  coal::CollisionObject terrain(
      std::make_shared<coal::HeightField<coal::AABB>>(4, 4, heights, -0.2));
  for (int i = 0; i < 64; ++i) {
    SCOPED_TRACE(i);
    const double x = -0.48 + (i % 8) * 0.13;
    const double y = -0.42 + (i / 8) * 0.12;
    coal::CollisionObject box(
        std::make_shared<coal::Box>(0.12, 0.08, 0.06),
        coal::Transform3s(coal::Vec3s(x, y, height(x, y) + 0.026)));
    const auto contacts = sire::physics::collision::generateHeightFieldContacts(
        terrain, box, 8);
    ASSERT_FALSE(contacts.empty());
    for (const auto& contact : contacts) {
      const coal::Vec3s local =
          box.getTransform().inverseTransform(contact.nearest_points[1]);
      EXPECT_NEAR(local.z(), -0.03, 1e-8);
      EXPECT_LE(std::abs(local.x()), 0.06 + 1e-8);
      EXPECT_LE(std::abs(local.y()), 0.04 + 1e-8);
      EXPECT_LT(contact.penetration_depth, 0);
      EXPECT_NEAR((contact.nearest_points[1] - contact.nearest_points[0])
                      .dot(contact.normal),
                  contact.penetration_depth, 1e-8);
    }
    const auto generic_contacts =
        sire::physics::collision::generateHeightFieldPatchContacts(terrain,
                                                                    box, 4);
    ASSERT_FALSE(generic_contacts.empty());
    for (const auto& contact : generic_contacts) {
      const coal::Vec3s local =
          box.getTransform().inverseTransform(contact.nearest_points[1]);
      EXPECT_LE(std::abs(local.x()), 0.06 + 1e-5);
      EXPECT_LE(std::abs(local.y()), 0.04 + 1e-5);
      EXPECT_LE(std::abs(local.z()), 0.03 + 1e-5);
      EXPECT_NEAR(contact.nearest_points[0].z(),
                  height(contact.nearest_points[0].x(),
                         contact.nearest_points[0].y()),
                  0.001);
      EXPECT_LT(contact.penetration_depth, 0);
    }
  }
}

TEST(HeightFieldWitnessTest, SmoothRoughBoxUsesOneFrictionPatch) {
  constexpr int grid_size = 65;
  coal::MatrixXs heights(grid_size, grid_size);
  for (int row = 0; row < grid_size; ++row) {
    for (int col = 0; col < grid_size; ++col) {
      const double x = -0.5 + static_cast<double>(col) / (grid_size - 1);
      const double y = 0.5 - static_cast<double>(row) / (grid_size - 1);
      heights(row, col) =
          0.01 * std::sin(4 * std::acos(-1.0) * x) *
          std::cos(3 * std::acos(-1.0) * y);
    }
  }
  coal::CollisionObject terrain(
      std::make_shared<coal::HeightField<coal::AABB>>(1, 1, heights, -0.2));
  coal::CollisionObject box(
      std::make_shared<coal::Box>(0.4, 0.3, 0.1),
      coal::Transform3s(coal::Vec3s(0, 0, 0.059)));
  coal::CollisionRequest request;
  request.num_max_contacts = 512;
  request.enable_height_field_contact_patches = true;
  coal::CollisionResult result;
  coal::collide(&terrain, &box, request, result);
  ASSERT_GT(result.numContacts(), 4u);
  sire::physics::collision::HeightFieldContactManifold manifold;
  manifold.beginQuery();
  const auto selected =
      manifold.select(terrain, box, result.getContacts(), 4);
  manifold.endQuery();
  ASSERT_FALSE(selected.empty());
  std::set<std::uint64_t> patches;
  for (const auto& contact : selected) patches.insert(contact.patch_id);
  EXPECT_EQ(patches.size(), 1u);
}

TEST(HeightFieldWitnessTest, CallbackKeepsGeometryIdOrdering) {
  auto terrain = flatTerrain(5);
  auto box = boxAt(coal::Vec3s(0.0, 0.0, 0.05));
  sire::physics::collision::CollisionFilter filter;
  filter.addGeometry(1, &box);
  filter.addGeometry(2, &terrain);
  std::vector<sire::physics::common::PenetrationAsPointPair> pairs;
  sire::physics::collision::HeightFieldContactCallback callback(&filter,
                                                                  &pairs);
  callback.collide(&box, &terrain);

  ASSERT_EQ(pairs.size(), 4u);
  for (const auto& pair : pairs) {
    EXPECT_EQ(pair.id_A, 1);
    EXPECT_EQ(pair.id_B, 2);
    EXPECT_NEAR(pair.p_WCa.z(), -0.05, 1e-12);
    EXPECT_NEAR(pair.p_WCb.z(), 0.0, 1e-12);
    EXPECT_NEAR(pair.nhat_AB_W.z(), -1.0, 1e-12);
    EXPECT_NEAR(pair.depth, 0.05, 1e-12);
  }
}

TEST(HeightFieldWitnessTest, CoalPatchCallbackKeepsGeometryIdOrdering) {
  auto terrain = flatTerrain(5);
  auto box = boxAt(coal::Vec3s(0.0, 0.0, 0.05));
  sire::physics::collision::CollisionFilter filter;
  filter.addGeometry(1, &box);
  filter.addGeometry(2, &terrain);
  std::vector<sire::physics::common::PenetrationAsPointPair> pairs;
  sire::physics::collision::HeightFieldContactCallback callback(
      &filter, &pairs, nullptr, true);
  callback.collide(&box, &terrain);

  ASSERT_GE(pairs.size(), 4u);
  for (const auto& pair : pairs) {
    EXPECT_EQ(pair.id_A, 1);
    EXPECT_EQ(pair.id_B, 2);
    EXPECT_NEAR(pair.p_WCa.z(), -0.05, 1e-6);
    EXPECT_NEAR(pair.p_WCb.z(), 0.0, 1e-6);
    EXPECT_NEAR(pair.nhat_AB_W.z(), -1.0, 1e-6);
    EXPECT_NEAR(pair.depth, 0.05, 1e-6);
  }
}

TEST(HeightFieldWitnessTest, PrismQueryReturnsAtMostOneWitnessPerTriangle) {
  auto terrain = flatTerrain(9);
  auto box = boxAt(coal::Vec3s(0.03, 0.02, 0.099));
  sire::physics::collision::HeightFieldPrismQueryStats stats;
  const auto contacts = sire::physics::collision::generateHeightFieldPrismContacts(
      terrain, box, 4096, &stats);
  ASSERT_GT(contacts.size(), 4u);
  EXPECT_FALSE(stats.limit_reached);
  std::set<int> triangles;
  for (const auto& c : contacts) {
    EXPECT_TRUE(triangles.insert(c.b1).second);
    EXPECT_EQ(c.o1, terrain.collisionGeometry().get());
    EXPECT_EQ(c.o2, box.collisionGeometry().get());
    EXPECT_NEAR(c.normal.norm(), 1, 1e-6);
    EXPECT_NEAR((c.nearest_points[1] - c.nearest_points[0]).dot(c.normal),
                c.penetration_depth, 1e-6);
    EXPECT_NEAR(c.nearest_points[0].z(), 0, 1e-6);
    EXPECT_NEAR(c.nearest_points[1].z(), -0.001, 1e-6);
  }
  EXPECT_LE(contacts.size(), stats.prisms_tested);
  EXPECT_EQ(stats.cells_visited * 2,
            stats.prisms_tested + stats.prisms_height_rejected);
}

TEST(HeightFieldWitnessTest, PrismQueryCapIsOnCandidatesAndCanBeAudited) {
  auto terrain = flatTerrain(17);
  auto box = boxAt(coal::Vec3s(0, 0, 0.099));
  sire::physics::collision::HeightFieldPrismQueryStats limited, unlimited;
  const auto capped = sire::physics::collision::generateHeightFieldPrismContacts(
      terrain, box, 3, &limited);
  const auto full = sire::physics::collision::generateHeightFieldPrismContacts(
      terrain, box, 4096, &unlimited);
  EXPECT_EQ(capped.size(), 3u);
  EXPECT_TRUE(limited.limit_reached);
  EXPECT_GT(full.size(), capped.size());
  EXPECT_FALSE(unlimited.limit_reached);
  EXPECT_LT(limited.prisms_tested, unlimited.prisms_tested);
  EXPECT_TRUE(sire::physics::collision::generateHeightFieldPrismContacts(
      terrain, box, 0).empty());
}

TEST(HeightFieldWitnessTest, PrismQuerySupportsAllFourRobotPrimitives) {
  auto terrain = flatTerrain(17);
  std::vector<std::shared_ptr<coal::CollisionGeometry>> geometries{
      std::make_shared<coal::Sphere>(0.1),
      std::make_shared<coal::Box>(0.2, 0.2, 0.2),
      std::make_shared<coal::Capsule>(0.05, 0.1),
      std::make_shared<coal::Cylinder>(0.1, 0.2)};
  for (const auto& geometry : geometries) {
    SCOPED_TRACE(geometry->getNodeType());
    coal::CollisionObject shape(geometry, coal::Transform3s(coal::Vec3s(0, 0, 0.099)));
    const auto contacts = sire::physics::collision::generateHeightFieldPrismContacts(
        terrain, shape, 4096);
    ASSERT_FALSE(contacts.empty());
    for (const auto& c : contacts) {
      const auto local = shape.getTransform().inverseTransform(c.nearest_points[1]);
      EXPECT_NEAR((c.nearest_points[1] - c.nearest_points[0]).dot(c.normal),
                  c.penetration_depth, 2e-6);
      if (geometry->getNodeType() == coal::GEOM_SPHERE)
        EXPECT_NEAR(local.norm(), 0.1, 2e-6);
      if (geometry->getNodeType() == coal::GEOM_CAPSULE) {
        const coal::Vec3s axis(0, 0, std::clamp(local.z(), -0.05, 0.05));
        EXPECT_NEAR((local - axis).norm(), 0.05, 2e-6);
      }
      if (geometry->getNodeType() == coal::GEOM_CYLINDER) {
        EXPECT_LE(local.head<2>().norm(), 0.1 + 2e-6);
        EXPECT_LE(std::abs(local.z()), 0.1 + 2e-6);
        EXPECT_NEAR(std::min(std::abs(local.z() + 0.1),
                             std::abs(local.head<2>().norm() - 0.1)), 0, 2e-6);
      }
    }
  }
}

TEST(HeightFieldWitnessTest, PrismQueryRespectsTerrainRotationAndSeparation) {
  auto terrain = flatTerrain(9);
  auto box = boxAt(coal::Vec3s(0, 0, 0.099));
  const auto original = sire::physics::collision::generateHeightFieldPrismContacts(
      terrain, box, 4096);
  const coal::Transform3s transform(
      Eigen::AngleAxis<coal::Scalar>(0.35, coal::Vec3s::UnitY()).toRotationMatrix(),
      coal::Vec3s(0.2, 0.3, 0.4));
  terrain.setTransform(transform);
  box.setTransform(transform * box.getTransform());
  const auto rotated = sire::physics::collision::generateHeightFieldPrismContacts(
      terrain, box, 4096);
  ASSERT_EQ(original.size(), rotated.size());
  for (std::size_t i = 0; i < original.size(); ++i) {
    EXPECT_NEAR((transform.transform(original[i].nearest_points[0]) -
                 rotated[i].nearest_points[0]).norm(), 0, 1e-6);
    EXPECT_NEAR(original[i].penetration_depth, rotated[i].penetration_depth, 1e-6);
  }
  box.setTransform(transform * coal::Transform3s(coal::Vec3s(0, 0, 0.2)));
  sire::physics::collision::HeightFieldPrismQueryStats stats;
  EXPECT_TRUE(sire::physics::collision::generateHeightFieldPrismContacts(
      terrain, box, 4096, &stats).empty());
  EXPECT_EQ(stats.prisms_tested, 0u);
}

TEST(HeightFieldWitnessTest, PrismCallbackPreservesPersistentIdsAndBodyOrder) {
  auto terrain = flatTerrain(9);
  auto box = boxAt(coal::Vec3s(0.03, 0.02, 0.099));
  sire::physics::collision::CollisionFilter filter;
  filter.addGeometry(1, &box);
  filter.addGeometry(2, &terrain);
  sire::physics::collision::HeightFieldContactManifold manifold;
  std::vector<sire::physics::common::PenetrationAsPointPair> pairs;
  sire::physics::collision::HeightFieldContactCallback callback(
      &filter, &pairs, &manifold, true, true);
  manifold.beginQuery();
  callback.collide(&box, &terrain);
  manifold.endQuery();
  ASSERT_FALSE(pairs.empty());
  ASSERT_LE(pairs.size(), 4u);
  const auto previous = pairs;
  pairs.clear();
  manifold.beginQuery();
  callback.collide(&terrain, &box);
  manifold.endQuery();
  ASSERT_EQ(pairs.size(), previous.size());
  for (std::size_t i = 0; i < pairs.size(); ++i) {
    EXPECT_EQ(pairs[i].id_A, 1);
    EXPECT_EQ(pairs[i].id_B, 2);
    EXPECT_EQ(pairs[i].contact_id, previous[i].contact_id);
    EXPECT_EQ(pairs[i].patch_id, previous[i].patch_id);
    EXPECT_GT(pairs[i].contact_id, 0u);
    EXPECT_NEAR(pairs[i].nhat_AB_W.z(), -1, 1e-6);
    EXPECT_NEAR(pairs[i].depth, 0.001, 1e-6);
  }
}

TEST(HeightFieldWitnessTest, PrismQueryKeepsSphereContactsOnBothSharpRidges) {
  coal::MatrixXs heights = coal::MatrixXs::Zero(33, 33);
  heights(16, 15) = heights(16, 17) = 0.15;
  coal::CollisionObject terrain(std::make_shared<coal::HeightField<coal::AABB>>(
      1, 1, heights, -0.02));
  const coal::Vec3s center(0, 0, 0.18);
  coal::CollisionObject sphere(std::make_shared<coal::Sphere>(0.05),
                               coal::Transform3s(center));
  const auto candidates = sire::physics::collision::generateHeightFieldPrismContacts(
      terrain, sphere, 4096);
  ASSERT_GE(candidates.size(), 2u);
  sire::physics::collision::HeightFieldContactManifold manifold;
  manifold.beginQuery();
  const auto selected = manifold.select(terrain, sphere, candidates, 4);
  manifold.endQuery();
  bool left = false, right = false;
  for (const auto& item : selected) {
    const auto& c = item.contact;
    left |= c.nearest_points[0].x() < -0.02;
    right |= c.nearest_points[0].x() > 0.02;
    EXPECT_NEAR((c.nearest_points[1] - center).norm(), 0.05, 2e-6);
    EXPECT_NEAR((c.nearest_points[1] - c.nearest_points[0]).dot(c.normal),
                c.penetration_depth, 2e-6);
  }
  EXPECT_TRUE(left);
  EXPECT_TRUE(right);
}

}  // namespace
