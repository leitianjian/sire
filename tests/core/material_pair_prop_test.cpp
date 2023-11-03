#include "sire/core/material_pair_prop.hpp"

#include "gtest/gtest.h"

#include <aris/core/serialization.hpp>

using namespace sire::core;
GTEST_TEST(MaterialPairProp, DefaultConstructor) {
  MaterialPairProp p;
  SortedPair<std::string> s;
  PropMap m;
  EXPECT_EQ(p.getFirstName().size(), 0);
  EXPECT_EQ(p.getSecondName().size(), 0);
  EXPECT_EQ(p.sortedPair(), s);
  EXPECT_EQ(p.getMaterialProp(), m);
}

GTEST_TEST(MaterialPairProp, OtherConstructor) {
  MaterialPairProp p({"steel", "copper"}, PropMap("{k:1e8,d:100}"));
  EXPECT_EQ(p.getFirstName(), "copper");
  EXPECT_EQ(p.getSecondName(), "steel");
  EXPECT_EQ(p.sortedPair().first(), "copper");
  EXPECT_EQ(p.sortedPair().second(), "steel");
  EXPECT_EQ(p.getMaterialProp().getPropValue("k"), 1e8);
  EXPECT_EQ(p.getMaterialProp().getPropValue("d"), 100);
}

GTEST_TEST(MaterialPairProp, DeserializationToString) {
  MaterialPairProp p;
  aris::core::fromXmlString(p,
                            "<MaterialPairProp first_name=\"steel\" "
                            "second_name=\"copper\" prop=\"{k:1e8,d:100}\"/>");
  EXPECT_EQ(p.getFirstName(), "steel");
  EXPECT_EQ(p.getSecondName(), "copper");
  EXPECT_EQ(p.sortedPair().first(), "copper");
  EXPECT_EQ(p.sortedPair().second(), "steel");
  EXPECT_EQ(p.getMaterialProp().getPropValue("k"), 1e8);
  EXPECT_EQ(p.getMaterialProp().getPropValue("d"), 100);
}

GTEST_TEST(MaterialPairProp, GetterSetter) {
  MaterialPairProp p;
  p.setFirstName("steel");
  p.setSecondName("copper");
  EXPECT_EQ(p.getFirstName(), "steel");
  EXPECT_EQ(p.getSecondName(), "copper");
  EXPECT_EQ(p.sortedPair().first(), "copper");
  EXPECT_EQ(p.sortedPair().second(), "steel");
}