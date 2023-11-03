#include "sire/core/material_manager.hpp"

#include "gtest/gtest.h"

#include <aris/core/serialization.hpp>

using namespace sire::core;
GTEST_TEST(MaterialManager, DefaultConstructor) {
  MaterialManager m;
  EXPECT_EQ(m.getPropMapOrDefault({"steel", "copper"}).getPropValue("k"),
            1.4e8);
  EXPECT_EQ(m.getPropMapOrDefault({"steel", "copper"}).getPropValue("d"), 1000);
  EXPECT_EQ(m.getPropMapOrDefault({"steel", "copper"}).getPropValue("cr"), 1.0);
}

GTEST_TEST(MaterialManager, DeserializationFromString) {
  MaterialManager m;
  aris::core::fromXmlString(
      m,
      "<MaterialManager default_prop=\"{k:1.4e8,d:1500,cr:0.2}\">"
      "<MaterialPairPropPool>"
      "<MaterialPairProp first_name=\"steel\" second_name=\"copper\" "
      "prop=\"{k:1e8,d:100,cr:1}\"/>"
      "</MaterialPairPropPool>"
      "</MaterialManager>");
  EXPECT_EQ(m.getPropMapOrDefault({"steel", "iron"}).getPropValue("k"), 1.4e8);
  EXPECT_EQ(m.getPropMapOrDefault({"steel", "iron"}).getPropValue("d"), 1500);
  EXPECT_EQ(m.getPropMapOrDefault({"steel", "iron"}).getPropValue("cr"), 0.2);
  EXPECT_EQ(m.getPropMapOrDefault({"steel", "copper"}).getPropValue("k"), 1e8);
  EXPECT_EQ(m.getPropMapOrDefault({"steel", "copper"}).getPropValue("d"), 100);
  EXPECT_EQ(m.getPropMapOrDefault({"steel", "copper"}).getPropValue("cr"), 1);
}

GTEST_TEST(MaterialManager, SerializationToString) {
  MaterialManager m;
  aris::core::fromXmlString(
      m,
      "<MaterialManager default_prop=\"{k:1.4e8,d:1500,cr:0.2}\">"
      "<MaterialPairPropPool>"
      "<MaterialPairProp first_name=\"steel\" second_name=\"copper\" "
      "prop=\"{k:1e8,d:100,cr:1}\"/>"
      "</MaterialPairPropPool>"
      "</MaterialManager>");
  std::cout << aris::core::toXmlString(m) << std::endl;
}