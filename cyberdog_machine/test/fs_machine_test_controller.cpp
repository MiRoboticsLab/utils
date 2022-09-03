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
#include <string>
#include <thread>
#include <chrono>
#include <map>
#include "gtest/gtest.h"
#include "cyberdog_machine/cyberdog_fs_machine.hpp"

TEST(basic, build_print)
{
  auto Uninitialized_T = static_cast<uint8_t>(cyberdog::machine::StateKey::kUninitialized);
  INFO("FS Machine running");
  rclcpp::Node::SharedPtr node_ptr = rclcpp::Node::make_shared("FSController");
  if(node_ptr == nullptr) {
    INFO("node ptr create failed!");
  }
  std::shared_ptr<cyberdog::machine::MachineController> machine_ptr = std::make_shared<cyberdog::machine::MachineController>();
  if(! machine_ptr->MachineControllerInit(std::string(BenchmarkPath) + "/fs_machine_test_config.toml", node_ptr)) {
    ERROR("Test FS Machine Init failed!");
  }
  std::map<std::string, uint8_t> state_map;

  machine_ptr->GetStateMap(state_map);
  EXPECT_EQ(Uninitialized_T, state_map["test1"]);

  auto result = machine_ptr->WaitActuatorsAccess();
  EXPECT_TRUE(result);
  machine_ptr->GetStateMap(state_map);
  EXPECT_EQ(Uninitialized_T, state_map["test1"]);

  result = machine_ptr->SetState(24);
  EXPECT_EQ(Uninitialized_T, state_map["test1"]);
  EXPECT_FALSE(result);

  result = machine_ptr->SetState(0);
  EXPECT_TRUE(result);
  machine_ptr->GetStateMap(state_map);
  EXPECT_EQ(0, state_map["test1"]);

  result = machine_ptr->SetState(23);
  machine_ptr->GetStateMap(state_map);
  EXPECT_EQ(23, state_map["test1"]);
  EXPECT_TRUE(result);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  LOGGER_MAIN_INSTANCE("FSController");
  return RUN_ALL_TESTS();
}
