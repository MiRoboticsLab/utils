// Copyright (c) 2021 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/**
 * @file json_test.cpp
 * @author dukun (dukun1@xiaomi.com)
 * @brief Test json package functions and performance,
 *        Under googletest
 *        Should not be compiled in OTA or DailyBuild version
 * @version 0.1
 * @date 2022-01-26
 *
 */
#include <string>
#include <iostream>
#include <algorithm>
#include "gtest/gtest.h"
#include "cyberdog_common/cyberdog_json.hpp"
// using namespace cyberdog::common;

TEST(hello, hello__Test)
{
  std::cout << "hello, gtest!" << std::endl;
  EXPECT_EQ('A', 65);
}

TEST(rapidjson, basic_using)
{
  const char json[] = "{\"hello\": \"Cyberdog\", \"version\": \"Carpo\"}";
  json::Document d;
  d.Parse<0>(json);
  if (d.HasParseError()) {
    return;
  }

  EXPECT_TRUE(d.IsObject());
  EXPECT_TRUE(d["hello"].IsString());
  EXPECT_TRUE(d.HasMember("version"));
  std::cout << "version: " << d["version"].GetString() << std::endl;

  json::Value::MemberIterator none = d.FindMember("None");
  EXPECT_EQ(none, d.MemberEnd());
  (void)none;
  json::Value::MemberIterator hello = d.FindMember("hello");
  EXPECT_NE(hello, d.MemberEnd());
  EXPECT_TRUE(hello->value.IsString());
  EXPECT_EQ(strcmp("Cyberdog", hello->value.GetString()), 0);
  (void)hello;
}

TEST(rapidjson, copy)
{
  std::cout << "hello copy work" << std::endl;
  const char json[] = "{\"hello\": \"Cyberdog\", \"version\": \"Carpo\"}";
  json::Document d;
  d.Parse<0>(json);
  if (d.HasParseError()) {
    return;
  }

  EXPECT_TRUE(d.IsObject());

  json::Document other;
  other.CopyFrom(d, other.GetAllocator());
  EXPECT_TRUE(other.IsObject());
  EXPECT_FALSE(d.IsNull());

  json::Document another;
  another.Swap(d);
  EXPECT_TRUE(d.IsNull());
}

TEST(cyberdogjson, reader)
{
  std::cout << "hello cyberdogjson reader" << std::endl;
  const char json[] = "{\"hello\": \"Cyberdog\", \"version\": \"Carpo\"}";
  json::Document d;
  auto result = CyberdogJson::String2Document(std::string(json), d);
  EXPECT_TRUE(result);

  std::string hello;
  result = CyberdogJson::Get(d, "hello", hello);
  EXPECT_TRUE(result);
  EXPECT_EQ(strcmp(hello.c_str(), "Cyberdog"), 0);
  EXPECT_FALSE(CyberdogJson::Get(d, "world", hello));
  json::Document dd(json::kArrayType);

  int no;
  result = CyberdogJson::Get(dd, "no", no);
  EXPECT_FALSE(result);
}

TEST(cyberdogjson, writer)
{
  std::cout << "hello cyberdogjson writer" << std::endl;
  json::Document d(json::kArrayType);
  json::Value v1(json::kStringType);
  v1.SetString(std::string("hello").c_str(), d.GetAllocator());
  auto result = CyberdogJson::Add(d, v1);
  EXPECT_TRUE(result);
  EXPECT_TRUE(d.IsArray());

  const char json[] = "{\"hello\": \"Cyberdog\", \"version\": \"Carpo\"}";
  result = CyberdogJson::String2Document(std::string(json), d);
  EXPECT_TRUE(result);
  EXPECT_TRUE(d.IsObject());

  result = CyberdogJson::Add(d, "project", "L91");
  EXPECT_TRUE(result);
  EXPECT_TRUE(d.HasMember("project"));

  json::Value v2(json::kObjectType);
  v2.AddMember("a", json::Value().SetInt(1), d.GetAllocator());
  v2.AddMember("b", json::Value().SetInt(2), d.GetAllocator());
  CyberdogJson::Add(d, "array", v2);
  EXPECT_TRUE(d.HasMember("array"));
  EXPECT_TRUE(d["array"].IsObject());
}

TEST(cyberdogjson, serialize)
{
  std::cout << "hello cyberdogjson serialize" << std::endl;
  const char json[] = "{\"hello\":\"Cyberdog\",\"version\":\"Carpo\"}";
  json::Document d;
  if (!CyberdogJson::String2Document(std::string(json), d)) {
    return;
  }
  std::string after;
  if (!CyberdogJson::Document2String(d, after)) {
    return;
  }
  EXPECT_EQ(std::string(json), after);
}

TEST(cyberdogjson, file)
{
  std::string file_name = std::string(BenchmarkPath) + std::string("/benchmark.json");
  json::Document d;
  auto result = CyberdogJson::ReadJsonFromFile(file_name, d);
  EXPECT_TRUE(result);
  EXPECT_TRUE(d.IsObject());

  std::string error_file("error_file");
  result = CyberdogJson::ReadJsonFromFile(error_file, d);
  EXPECT_FALSE(result);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
