#include "sire/core/sorted_pair.hpp"

#include <sstream>
#include <string>

#include "gtest/gtest.h"

#include "sire/core/geometry/geometry_base.hpp"

using namespace sire::core;
using namespace sire::geometry;
// verify default constructor
GTEST_TEST(SortedPair, DefaultConstructor) {
  SortedPair<GeometryId> p1;
  EXPECT_EQ(p1.first(), 0);
  EXPECT_EQ(p1.second(), 0);
  SortedPair<std::string> p2;
  EXPECT_EQ(p2.first(), "");
  EXPECT_EQ(p2.second(), "");
}

// verify sorting occur
GTEST_TEST(SortedPair, Sorted) {
  SortedPair<GeometryId> p1{6, 5};
  EXPECT_EQ(p1.first(), 5);
  EXPECT_EQ(p1.second(), 6);
  SortedPair<std::string> p2{"steel", "copper"};
  EXPECT_EQ(p2.first(), "copper");
  EXPECT_EQ(p2.second(), "steel");
}

// verify const parameter constructor
GTEST_TEST(SortedPair, ConstConstructor) {
  GeometryId a1 = 3;
  GeometryId b1 = 2;
  SortedPair<GeometryId> p1(a1, b1);
  EXPECT_EQ(p1.first(), 2);
  EXPECT_EQ(p1.second(), 3);
  std::string a2 = "steel";
  std::string b2 = "copper";
  SortedPair<std::string> p2(a2, b2);
  EXPECT_EQ(p2.first(), b2);
  EXPECT_EQ(p2.second(), a2);
}

// verify function set()
GTEST_TEST(SortedPair, Set) {
  SortedPair<GeometryId> p1;
  p1.set(1, 2);
  EXPECT_EQ(p1.first(), 1);
  EXPECT_EQ(p1.second(), 2);
  p1.set(5, 3);
  EXPECT_EQ(p1.first(), 3);
  EXPECT_EQ(p1.second(), 5);

  SortedPair<std::string> p2;
  p2.set<std::string>("steel", "copper");
  EXPECT_EQ(p2.first(), "copper");
  EXPECT_EQ(p2.second(), "steel");
  p2.set<std::string>("copper", "ground");
  EXPECT_EQ(p2.first(), "copper");
  EXPECT_EQ(p2.second(), "ground");
}

// check assignment operator
GTEST_TEST(SortedPair, Assignment) {
  SortedPair<GeometryId> a1;
  SortedPair<GeometryId> b1{3, 2};
  EXPECT_EQ(&(a1 = b1), &a1);
  EXPECT_EQ(a1.first(), 2);
  EXPECT_EQ(a1.second(), 3);

  SortedPair<std::string> a2;
  SortedPair<std::string> b2{"steel", "copper"};
  EXPECT_EQ(&(a2 = b2), &a2);
  EXPECT_EQ(a2.first(), "copper");
  EXPECT_EQ(a2.second(), "steel");
}

// check equality operator
GTEST_TEST(SortedPair, Equality) {
  SortedPair<GeometryId> a1(1, 2), b1(2, 1);
  EXPECT_EQ(a1, b1);

  SortedPair<std::string> a2("copper", "steel"), b2("steel", "copper");
  EXPECT_EQ(a2, b2);
}

// check comparison operators
GTEST_TEST(SortedPair, Comparison) {
  SortedPair<GeometryId> a(1, 2);
  SortedPair<GeometryId> b(2, 2);
  SortedPair<GeometryId> c(3, 2);
  EXPECT_FALSE(a < a);
  EXPECT_FALSE(a > a);
  EXPECT_TRUE(a >= a);
  EXPECT_TRUE(a <= a);
  EXPECT_TRUE(a < b);
  EXPECT_TRUE(c > b);
  EXPECT_TRUE(a <= b);
  EXPECT_TRUE(c >= b);

  SortedPair<std::string> a1("copper", "ground");
  SortedPair<std::string> b1("ground", "ground");
  SortedPair<std::string> c1("steel", "ground");
  EXPECT_FALSE(a1 < a1);
  EXPECT_FALSE(a1 > a1);
  EXPECT_TRUE(a1 >= a1);
  EXPECT_TRUE(a1 <= a1);
  EXPECT_TRUE(a1 < b1);
  EXPECT_TRUE(c1 > b1);
  EXPECT_TRUE(a1 <= b1);
  EXPECT_TRUE(c1 >= b1);
}

// check swap function
GTEST_TEST(SortedPair, Swap) {
  SortedPair<GeometryId> a(2, 1), b(4, 3);
  std::swap(a, b);
  EXPECT_EQ(a.first(), 3);
  EXPECT_EQ(a.second(), 4);
  EXPECT_EQ(b.first(), 1);
  EXPECT_EQ(b.second(), 2);

  SortedPair<std::string> a1("ground", "copper"), b1("copper", "steel");
  using std::swap;
  swap(a1, b1);
  EXPECT_EQ(a1.first(), "copper");
  EXPECT_EQ(a1.second(), "steel");
  EXPECT_EQ(b1.first(), "copper");
  EXPECT_EQ(b1.second(), "ground");
}

// check hash function
GTEST_TEST(SortedPair, Hash) {
  SortedPair<GeometryId> a(2, 1), b(1, 2);
  using std::hash;
  EXPECT_EQ(hash<SortedPair<GeometryId>>()(a),
            hash<SortedPair<GeometryId>>()(b));

  SortedPair<std::string> a1("copper", "steel"), b1("steel", "copper");
  using std::hash;
  EXPECT_EQ(hash<SortedPair<std::string>>()(a1),
            hash<SortedPair<std::string>>()(b1));
}

// test streaming support
GTEST_TEST(SortedPair, Write2Stream) {
  SortedPair<GeometryId> pair{8, 7};
  std::stringstream ss;
  ss << pair;
  EXPECT_EQ(ss.str(), "(7, 8)");

  SortedPair<std::string> pair1{"steel", "copper"};
  std::stringstream ss1;
  ss1 << pair1;
  EXPECT_EQ(ss1.str(), "(copper, steel)");
}