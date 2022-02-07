/**
 * @file toml_test.cpp
 * @author dukun (dukun1@xiaomi.com)
 * @brief Test toml package functions and performance,
 *        Under googletest.
 *        Should not be compiled in OTA or DailyBuild version.
 * @version 0.1
 * @date 2022-02-05
 *
 * @copyright Copyright (c) 2022. MiRobotlab Rop Team.
 *
 */
#include <dirent.h>
#include <stdio.h>
#include <iostream>
#include <algorithm>
#include "gtest/gtest.h"
#include "cyberdog_common/cyberdog_toml.hpp"
using namespace cyberdog::common;
// using namespace toml;

TEST(hello, hello__Test)
{
  std::cout << "hello, gtest!" << std::endl;
  EXPECT_EQ('a', 97);
}

TEST(file, parser)
{
  toml::value value;
  auto result = CyberdogToml::ParseFile("nofile", value);
  ASSERT_FALSE(result);

  result = CyberdogToml::ParseFile(std::string(BenchmarkPath) + "/benchmark.toml", value);
  ASSERT_TRUE(result);
}

TEST(reader, table)
{
  toml::value value;
  if (!CyberdogToml::ParseFile(std::string(BenchmarkPath) + "/benchmark.toml", value)) {
    return;
  }

  toml::value a;
  auto result = CyberdogToml::Get(value, "a", a);
  ASSERT_TRUE(result);
  ASSERT_TRUE(a.is_table());

  int aa;
  result = CyberdogToml::Get(a, "a", aa);
  ASSERT_TRUE(result);
  ASSERT_EQ(aa, 1);

  std::string ab;
  result = CyberdogToml::Get(a, "b", ab);
  ASSERT_TRUE(result);
  ASSERT_EQ(ab, std::string("cyberdog"));

  bool ac;
  result = CyberdogToml::Get(a, "c", ac);
  ASSERT_TRUE(result);
  ASSERT_EQ(ac, true);

  float ad;
  result = CyberdogToml::Get(a, "d", ad);
  ASSERT_TRUE(result);
  ASSERT_EQ(ad, static_cast<float>(0.1));
}

TEST(reader, array)
{
  toml::value value;
  if (!CyberdogToml::ParseFile(std::string(BenchmarkPath) + "/benchmark.toml", value)) {
    return;
  }

  toml::value b;
  auto result = CyberdogToml::Get(value, "b", b);
  ASSERT_TRUE(result);
  ASSERT_TRUE(b.is_array());
  ASSERT_TRUE(b.as_array().at(0).is_table());

  toml::value b0;
  result = CyberdogToml::Get(b, 0, b0);
  ASSERT_TRUE(result);
  ASSERT_TRUE(b0.is_table());
  result = CyberdogToml::Get(b, 10, b0);
  ASSERT_FALSE(result);
}

TEST(writer, modify)
{
  toml::value value;
  if (!CyberdogToml::ParseFile(std::string(BenchmarkPath) + "/benchmark.toml", value)) {
    return;
  }

  toml::value b;
  auto result = CyberdogToml::Get(value, "b", b);
  ASSERT_TRUE(result);
  result = CyberdogToml::Set(b, "a", 0);
  ASSERT_FALSE(result);

  toml::value b1;
  result = CyberdogToml::Get(b, 1, b1);
  ASSERT_TRUE(result);
  result = CyberdogToml::Set(b1, "c", true);
  ASSERT_TRUE(result);
  bool b1c;
  result = CyberdogToml::Get(b1, "c", b1c);
  ASSERT_TRUE(b1c);
}

TEST(writer, add)
{
  toml::value a;
  std::cout << a.is_uninitialized() << std::endl;
  std::cout << a.is_table() << std::endl;
  auto result = CyberdogToml::Set(a, "b", std::string("xiaomi"));
  ASSERT_TRUE(result);

  std::string ab;
  result = CyberdogToml::Get(a, "b", ab);
  ASSERT_TRUE(result);
  ASSERT_EQ(ab, std::string("xiaomi"));

  toml::value b;
  result = CyberdogToml::Set(b, 0, std::string("xiaomi"));
  ASSERT_FALSE(result);

  result = CyberdogToml::Set(b, std::string("xiaomi"));
  ASSERT_TRUE(result);
  ASSERT_TRUE(b.is_array());

  std::string b0;
  result = CyberdogToml::Get(b, 0, b0);
  ASSERT_TRUE(result);
  ASSERT_EQ(b0, std::string("xiaomi"));
}

int main(int argc, char ** argv)
{
  // std::cout << BenchmarkPath << std::endl;
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
