#include "sire/core/prop_map.hpp"

#include <sstream>

#include "gtest/gtest.h"

using namespace sire::core;
GTEST_TEST(PropMap, PropMapToString) {
  PropMap p;
  p.addProp("test", 1);
  p.addProp("test1", 1);
  EXPECT_EQ(p.toString(), "{test:1,test1:1}");
}

GTEST_TEST(PropMap, StringToPropMap) {
  PropMap p;
  p.fromString("{test:1,test1:1}");
  std::cout << p.toString() << std::endl;
  EXPECT_EQ(p.getPropValue("test"), 1);
  EXPECT_EQ(p.getPropValue("test1"), 1);

  p.clear();
  p.fromString("{ dafssa : 10.464623164 , asf : 1.54654e2    }");
  std::cout << p.toString() << std::endl;
  EXPECT_EQ(p.getPropValue("dafssa"), 10.464623164);
  EXPECT_EQ(p.getPropValue("asf"), 154.654);
}