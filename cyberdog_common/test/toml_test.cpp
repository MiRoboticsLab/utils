/**
 * @file toml_test.cpp
 * @author dukun (dukun1@xiaomi.com)
 * @brief Test toml package functions and performance,
 *        Under googletest
 *        Should not be compiled in OTA or DailyBuild version
 * @version 0.1
 * @date 2022-02-05
 * 
 * @copyright Copyright (c) 2022.
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

TEST(cyberdogtoml, parser)
{
  toml::value value;
  auto result = CyberdogToml::ParseFile("nofile", value);
  ASSERT_FALSE(result);

  result = CyberdogToml::ParseFile(std::string(BenchmarkPath) + "/benchmark.toml", value);
  ASSERT_TRUE(result);
}

TEST(cyberdogtoml, reader)
{
  toml::value value;
  if(! CyberdogToml::ParseFile(std::string(BenchmarkPath) + "/benchmark.toml", value))
    return;

  toml::value a;
  if(CyberdogToml::Get(value, "a", a)) {
    int aa;
    if(CyberdogToml::Get(a, "a", aa)) {
      ASSERT_EQ(aa, 1);
    }
  }
}

int main(int argc, char ** argv)
{
  // std::cout << BenchmarkPath << std::endl;
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}