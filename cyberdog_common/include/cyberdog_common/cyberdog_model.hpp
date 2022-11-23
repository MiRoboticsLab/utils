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
#ifndef CYBERDOG_COMMON__CYBERDOG_MODEL_HPP_
#define CYBERDOG_COMMON__CYBERDOG_MODEL_HPP_

#include <string>
#include <vector>
#include <iostream>
#include <unistd.h>
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"
namespace cyberdog
{
namespace common
{
enum ShellEnum
{
  shell = 1993,     // 异常结束, 执行shell命令错误
  command,          // 异常结束, 待执行命令错误
  command_popen,    // 异常结束, 无法执行命令
  command_error,    // 异常结束, 执行命令失败
};
class cyberdog_model final
{
public:
  cyberdog_model(std::string model_path)
  {
    std::cout<<"hello"<<std::endl;
    std::cout<<"model_path"<<std::endl;
  }

  ~cyberdog_model()
  {

  }

  bool Md5_Compare(std::string filepath,std::string md5_string)
  {
      int shell_code;
      std::string shell_message;
      bool result = this->Shell("md5sum "+filepath,shell_code,shell_message);
      INFO("shell_code %d",shell_code);
      INFO("shell_message %s",shell_message.c_str());
      INFO("shell_result %d",static_cast<int>(result));
      bool md5same = shell_message.compare(md5_string);
      if (md5same == true){
        INFO("model md5 is same");
        return true;
      } else {
        INFO("model md5 is different");
      }      
  }
  bool Is_Newest_Version(float_t &version,std::vector<float_t>& version_array)
  {
     
  }
  bool Is_Specified_Version(float_t &spec_version,float_t &nx_version)
  {
     if (abs(spec_version - nx_version) < this->EPS){
      INFO("specified model version is same as nx model version");
      return true;
     } else {
        INFO("specified model version is differente from model version");
        return false;
     }
  }
  bool Find_Model_Version(std::string &path,float_t &version)
  {
    INFO("read model version in toml file");
    toml::value value;
    auto local_config_dir = path + std::string(
      "/version.toml");
    if (access(local_config_dir.c_str(), F_OK) != 0) {
      ERROR("%s do not exist!", local_config_dir.c_str());
      return false;
    }
    if (!cyberdog::common::CyberdogToml::ParseFile(
        std::string(local_config_dir), value))
    {
      ERROR("fail to read data from toml");
    }

    if (!cyberdog::common::CyberdogToml::Get(value, "version", version)) {
      ERROR("fail to read key queue_size from toml");
    }
    return true;
  }

  bool File_Exist(std::string filepath)
  {
    if (access(filepath.c_str(), F_OK) != 0) {
      INFO("%s do not exist!", filepath.c_str());
      return false;
    } else {
      INFO("%s exist!", filepath.c_str());
      return true;
    }  
  }

  bool Mk_Folder(std::string path){
    if (access(path.c_str(), F_OK) != 0) {
      INFO("%s do not exist!", path.c_str());
      INFO("mkdir %s",path.c_str());
      int shell_code;
      std::string shell_message;
      bool result = this->Shell("mkdir -p "+path,shell_code,shell_message);
      INFO("shell_code %d",shell_code);
      INFO("shell_message %s",shell_message.c_str());
      INFO("shell_result %d",static_cast<int>(result));
      if (result != true){
        INFO("mkdir failed");
        return false;
      } else {
        INFO("mkdir successfully");
        return true;
      }
    } else {
      INFO("%s exist!", path.c_str());
      return true;
    }
  }


bool Shell(const std::string & _command, int & _code, std::string & _message)
{
  try {
    DEBUG("Shell: %s", _command.c_str());
    _message = "";
    if (!_command.empty()) {
      std::string _code_cmd = _command + "; echo $?";  //  | xargs echo
      std::string _code_str = "";
      FILE * fstream_ptr = nullptr;
      fstream_ptr = popen(_code_cmd.c_str(), "r");
      if (fstream_ptr != nullptr) {
        char buffer[LINE_MAX_SIZE];
        while (fgets(buffer, LINE_MAX_SIZE, fstream_ptr) != nullptr) {
          _code_str = buffer;
          memset(buffer, '\0', sizeof(buffer));
          _message += _code_str;
        }
        pclose(fstream_ptr);
        fstream_ptr = nullptr;
        _code = std::atoi(_code_str.c_str());
        if (_code == static_cast<int>(0)) {
          return true;
        } else {
          _code = static_cast<int>(ShellEnum::command_error);
          _code_cmd = _command + " 2> /dev/stdout";
          fstream_ptr = popen(_code_cmd.c_str(), "r");
          if (fstream_ptr != nullptr) {
            _code_str = "";
            while (fgets(buffer, LINE_MAX_SIZE, fstream_ptr) != nullptr) {
              _code_str += std::string(buffer);
              memset(buffer, '\0', sizeof(buffer));
            }
            pclose(fstream_ptr);
            fstream_ptr = nullptr;
          }
          _message = "Shell command is error.\n - command : " + _command +
            "\n - code : " + _message + " - error : " + _code_str;
        }
      } else {
        _code = static_cast<int>(ShellEnum::command_popen);
        _message = "Canot shell command popen.\n - command : " + _command +
          "\n - error : " + strerror(errno);
      }
    } else {
      _code = static_cast<int>(ShellEnum::command);
      _message = "Shell command is empty.\n - command : " + _command;
    }
  } catch (const std::exception & e) {
    _code = static_cast<int>(ShellEnum::shell);
    _message = "Shell command is error.\n - command : " + _command +
      "\n - error : " + e.what();
  }
  ERROR("Shell: %s, %s", _command.c_str(), _message.c_str());
  return false;
}






public:
  static const unsigned int LINE_MAX_SIZE {16384};                      // 行最大值:2kb
  const double EPS = 0.0000001;


};  // class cyberdog_model
}  // namespace cyberdog
}  // namespace common


#endif  // CYBERDOG_COMMON__CYBERDOG_MODEL_HPP_