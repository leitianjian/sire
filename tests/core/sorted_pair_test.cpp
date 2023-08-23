#include "sire/core/sorted_pair.hpp"

#include <sstream>

#include "gtest/gtest.h"

using namespace sire::core;
// verify default constructor
GTEST_TEST(SortedPair, DefaultConstructor) {
  SortedPair p;
  EXPECT_EQ(p.first(), 0);
  EXPECT_EQ(p.second(), 0);
}

// verify sorting occur
GTEST_TEST(SortedPair, Sorted) {
  SortedPair p{6, 5};
  EXPECT_EQ(p.first(), 5);
  EXPECT_EQ(p.second(), 6);
}

// verify const parameter constructor
GTEST_TEST(SortedPair, ConstConstructor) {
  GeometryId a = 3;
  GeometryId b = 2;
  SortedPair p(a, b);
  EXPECT_EQ(p.first(), 2);
  EXPECT_EQ(p.second(), 3);
}

// verify function set()
GTEST_TEST(SortedPair, Set) {
  SortedPair p;
  p.set(1, 2);
  EXPECT_EQ(p.first(), 1);
  EXPECT_EQ(p.second(), 2);
  p.set(5, 3);
  EXPECT_EQ(p.first(), 3);
  EXPECT_EQ(p.second(), 5);
}

// check assignment operator
GTEST_TEST(SortedPair, Assignment) {
  SortedPair a;
  SortedPair b{3, 2};
  EXPECT_EQ(&(a = b), &a);
  EXPECT_EQ(a.first(), 2);
  EXPECT_EQ(a.second(), 3);
}

// check equality operator
GTEST_TEST(SortedPair, Equality) {
  SortedPair a(1, 2), b(2, 1);
  EXPECT_EQ(a, b);
}

// check comparison operators
GTEST_TEST(SortedPair, Comparison) {
  SortedPair a(1, 2);
  SortedPair b(2, 2);
  SortedPair c(3, 2);
  EXPECT_FALSE(a < a);
  EXPECT_FALSE(a > a);
  EXPECT_TRUE(a >= a);
  EXPECT_TRUE(a <= a);
  EXPECT_TRUE(a < b);
  EXPECT_TRUE(c > b);
  EXPECT_TRUE(a <= b);
  EXPECT_TRUE(c >= b);
}

// check swap function
GTEST_TEST(SortedPair, Swap) {
  SortedPair a(2, 1), b(4, 3);
  std::swap(a, b);
  EXPECT_EQ(a.first(), 3);
  EXPECT_EQ(a.second(), 4);
  EXPECT_EQ(b.first(), 1);
  EXPECT_EQ(b.second(), 2);
}

// test streaming support
GTEST_TEST(SortedPair, Write2Stream) {
  SortedPair pair{8, 7};
  std::stringstream ss;
  ss << pair;
  EXPECT_EQ(ss.str(), "(7, 8)");
}