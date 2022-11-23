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


#include "cyberdog_common/cyberdog_model.hpp"

int main()
{
  std::string path_dir = "/SDCARD/algo/gesture_action";
  cyberdog::common::cyberdog_model test(path_dir);
  test.Mk_Folder(path_dir);
  bool exist = test.File_Exist("/SDCARD/algo/gesture_action/nx_mobilenetv2_tsm.trt");
  float_t version;
  bool read_version = test.Find_Model_Version(path_dir,version);
  std::string md5 = "83b20cbf9f41f5c9eadf944d0cb902ad";
  bool md4_comp = test.Md5_Compare("/SDCARD/algo/gesture_action/nx_mobilenetv2_tsm.trt"
    ,md5);
  std::cout<<"version: "<<version<<std::endl;
  std::cout<<"md4_comp: "<<md4_comp<<std::endl;
  return 0;
}