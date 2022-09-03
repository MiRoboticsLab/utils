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
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "cyberdog_machine/cyberdog_fs_machine.hpp"
#include "fs_machine_test_actuator.hpp"


void test()
{
  INFO("test");
  std::vector<int32_t> vec_int{1, 2, 3, 4, 5};
  for(auto i = 0; i < 5; i++) {
    INFO("vec value: %d", vec_int[i]);
  }
  auto result = std::all_of(vec_int.begin(), vec_int.end(), [](int & value){
    INFO("all of===value: %d", value);
    if(value == 2)
      return false;
    return true;
  });
  INFO("all of result: %d", result);

  result = std::none_of(vec_int.begin(), vec_int.end(), [](int & value){
    INFO("none of===value: %d", value);
    if(value == 4)
      return true;
    return false;
  });
  INFO("none of result: %d", result);

  result = std::any_of(vec_int.begin(), vec_int.end(), [](int & value){
    INFO("none of===value: %d", value);
    if(value == 3)
      return true;
    return false;
  });
  INFO("any of result: %d", result);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  LOGGER_MAIN_INSTANCE("ActuatorTest1");
  INFO("Running");
  test();
  std::shared_ptr<ActuactorDemo> ptr = std::make_shared<ActuactorDemo>("test1");
  ptr->Spin();
  return 0;
}