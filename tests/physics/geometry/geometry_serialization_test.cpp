#include <gtest/gtest.h>

#include <aris/core/object.hpp>
#include <aris/core/serialization.hpp>

#include "sire/physics/geometry/sphere_collision_geometry.hpp"

TEST(GeometrySerializationTest, PenetrationDynamicAndAnchored) {
  sire::physics::geometry::SphereCollisionGeometry g;
  std::string s =
      "<SphereCollisionGeometry id=\"1\" part_id=\"1\" is_dynamic=\"true\" "
      "contact_prop=\"{test:1, test1:2}\" radius=\"0.030\" pm=\"{1, 0, 0, 0.3, "
      "0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}\"/>";
  aris::core::fromXmlString(g, s);
  EXPECT_EQ(g.contactProp().getPropValue("test"), 1);
  EXPECT_EQ(g.contactProp().getPropValue("test1"), 2);
}
