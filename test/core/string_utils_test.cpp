#include "sire/core/string_utils.hpp"

#include <sstream>

#include "gtest/gtest.h"

using namespace sire::core;
TEST(StringUtilsTest, StringLeftTrim) {
  std::string s0{" "};
  std::string s1{" s"};
  std::string s2{" s "};
  std::string s3{" sd "};
  std::string s4{"sd"};
  auto v0 = ltrim(s0);
  auto v1 = ltrim(s1);
  auto v2 = ltrim(s2);
  auto v3 = ltrim(s3);
  auto v4 = ltrim(s4);
  EXPECT_EQ(v0, "");
  EXPECT_EQ(v1, "s");
  EXPECT_EQ(v2, "s ");
  EXPECT_EQ(v3, "sd ");
  EXPECT_EQ(v4, "sd");
}

TEST(StringUtilsTest, StringViewLeftTrim) {
  std::string_view s0{" "};
  std::string_view s1{" s"};
  std::string_view s2{" s "};
  std::string_view s3{" sd "};
  std::string_view s4{"sd"};
  s0 = ltrim(s0);
  s1 = ltrim(s1);
  s2 = ltrim(s2);
  s3 = ltrim(s3);
  s4 = ltrim(s4);
  EXPECT_EQ(s0, "");
  EXPECT_EQ(s1, "s");
  EXPECT_EQ(s2, "s ");
  EXPECT_EQ(s3, "sd ");
  EXPECT_EQ(s4, "sd");
}

TEST(StringUtilsTest, StringRightTrim) {
  std::string s0{" "};
  std::string s1{" s"};
  std::string s2{" s "};
  std::string s3{" sd "};
  std::string s4{"sd"};
  auto v0 = rtrim(s0);
  auto v1 = rtrim(s1);
  auto v2 = rtrim(s2);
  auto v3 = rtrim(s3);
  auto v4 = rtrim(s4);
  EXPECT_EQ(v0, "");
  EXPECT_EQ(v1, " s");
  EXPECT_EQ(v2, " s");
  EXPECT_EQ(v3, " sd");
  EXPECT_EQ(v4, "sd");
}

TEST(StringUtilsTest, StringViewRightTrim) {
  std::string_view s0{" "};
  std::string_view s1{" s"};
  std::string_view s2{" s "};
  std::string_view s3{" sd "};
  std::string_view s4{"sd"};
  s0 = rtrim(s0);
  s1 = rtrim(s1);
  s2 = rtrim(s2);
  s3 = rtrim(s3);
  s4 = rtrim(s4);
  EXPECT_EQ(s0, "");
  EXPECT_EQ(s1, " s");
  EXPECT_EQ(s2, " s");
  EXPECT_EQ(s3, " sd");
  EXPECT_EQ(s4, "sd");
}