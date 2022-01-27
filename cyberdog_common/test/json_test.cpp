/**
 * @file json_test.cpp
 * @author dukun (dukun1@xiaomi.com)
 * @brief Test json package functions and performance,
 *        Under googletest
 *        Should not be compiled in OTA or DailyBuild version
 * @version 0.1
 * @date 2022-01-26
 *
 * @copyright Copyright (c) 2022.
 *
 */
#include <iostream>
#include <algorithm>
#include "gtest/gtest.h"
#include "cyberdog_common/cyberdog_json.hpp"
using namespace cyberdog::common;
using namespace rapidjson;

TEST(hello, hello__Test)
{
  std::cout << "hello, gtest!" << std::endl;
  EXPECT_EQ('A', 65);
}

TEST(rapidjson, basic_using)
{
  const char json[] = "{\"hello\": \"Cyberdog\", \"version\": \"Carpo\"}";
  Document d;
  d.Parse<0>(json);
  if (d.HasParseError()) {
    return;
  }

  EXPECT_TRUE(d.IsObject());
  EXPECT_TRUE(d["hello"].IsString());
  EXPECT_TRUE(d.HasMember("version"));
  std::cout << "version: " << d["version"].GetString() << std::endl;

  Value::MemberIterator none = d.FindMember("None");
  EXPECT_EQ(none, d.MemberEnd());
  (void)none;
  Value::MemberIterator hello = d.FindMember("hello");
  EXPECT_NE(hello, d.MemberEnd());
  EXPECT_TRUE(hello->value.IsString());
  EXPECT_EQ(strcmp("Cyberdog", hello->value.GetString()), 0);
  (void)hello;
}

TEST(rapidjson, copy)
{
  std::cout << "hello copy work" << std::endl;
  const char json[] = "{\"hello\": \"Cyberdog\", \"version\": \"Carpo\"}";
  Document d;
  d.Parse<0>(json);
  if (d.HasParseError()) {
    return;
  }

  EXPECT_TRUE(d.IsObject());

  Document other;
  other.CopyFrom(d, other.GetAllocator());
  EXPECT_TRUE(other.IsObject());
  EXPECT_FALSE(d.IsNull());

  Document another;
  another.Swap(d);
  EXPECT_TRUE(d.IsNull());
}

TEST(cyberdogjson, reader)
{
  std::cout << "hello cyberdogjson reader" << std::endl;
  const char json[] = "{\"hello\": \"Cyberdog\", \"version\": \"Carpo\"}";
  Document d;
  if (!CyberdogJson::String2Document(std::string(json), d)) {
    return;
  }

  std::string hello;
  if (!CyberdogJson::Get(d, "hello", hello)) {
    return;
  }
  EXPECT_EQ(strcmp(hello.c_str(), "Cyberdog"), 0);
  // std::cout << "hello, " << hello << std::endl;
  EXPECT_FALSE(CyberdogJson::Get(d, "world", hello));
}

TEST(cyberdogjson, writer)
{
  std::cout << "hello cyberdogjson writer" << std::endl;
  const char json[] = "{\"hello\": \"Cyberdog\", \"version\": \"Carpo\"}";
  Document d;
  if (!CyberdogJson::String2Document(std::string(json), d)) {
    return;
  }
  CyberdogJson::Add(d, "project", "L91");
  EXPECT_TRUE(d.HasMember("project"));
  Value v(kObjectType);
  v.AddMember("a", Value().SetInt(1), d.GetAllocator());
  v.AddMember("b", Value().SetInt(2), d.GetAllocator());
  CyberdogJson::Add(d, "array", v);
  EXPECT_TRUE(d.HasMember("array"));
  EXPECT_TRUE(d["array"].IsObject());
}

TEST(cyberdogjson, serialize)
{
  std::cout << "hello cyberdogjson serialize" << std::endl;
  const char json[] = "{\"hello\":\"Cyberdog\",\"version\":\"Carpo\"}";
  Document d;
  if (!CyberdogJson::String2Document(std::string(json), d)) {
    return;
  }
  std::string after;
  if (!CyberdogJson::Document2String(d, after)) {
    return;
  }
  EXPECT_EQ(std::string(json), after);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
