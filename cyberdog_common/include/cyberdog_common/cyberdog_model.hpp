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
#include "cyberdog_common/cyberdog_fds.hpp"
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

enum class ModelKeyCode : int32_t
{
  kOK = 0,
  kFdsClientCreateFailed = 5821,
  kFdsModelNotExist = 5822,
  kShellCmdFailed = 5823,
  kTomlParseFailed = 5824,
  kTomlWriteFailed = 5825,
  kTomlGetFailed = 5826,
  kTomlFileNotExist =5827,
  kModelVersionNotInFds = 5828,
  KNoNeedToDownload = 5829,
  kMd5NotSame = 5830,
  kDownloadFailed = 5831,
};

class cyberdog_model final
{
public:
  cyberdog_model(std::string name, bool is_specified = false,
    std::string version = "0.0",
    std::string prefix = "/SDCARD/",
    std::string category = "vision",
    std::string account = "AKIFGZADWTQQVTXBZJ",
    std::string secret = "vb9nDcV/SmkZjWHl4cBObIPdu/FXM9Uh96l0PETt",
    std::string endpoint = "cnbj2m.fds.api.xiaomi.com"):
    module_name(name),prefix_dir(prefix),func_name(category),
    specified_version(version),is_specified_version(is_specified),
    fds_(account,secret,endpoint)
  {
    INFO("cyberdog_model Constructor");
    this->model_dir = this->prefix_dir + this->func_name + "/" + this->module_name + "/";
    INFO("is_specified_version: %d",static_cast<int>(this->is_specified_version));
    INFO("specified_version: %s",this->specified_version.c_str());
    INFO("module_name: %s",this->module_name.c_str());
    INFO("model_dir: %s",this->model_dir.c_str());
  }

  ~cyberdog_model()
  {

  }

  int32_t UpdateModels()
  {
   INFO("update models fuc begin");
   if(!init()){
    ERROR("init failed");
    return this->result_code;
   }
   if(!Find_Local_Model_Version()){
    ERROR("find local model version failed");
    return this->result_code;
   }
   if(!Prepare_Download()){
    ERROR("prepare download failed");
    return this->result_code;    
   }

   if(!DownloadModels()){
    ERROR("download failed or do not need download");
    return this->result_code; 
   }
   if(!Md5_Check()){
    ERROR("md5 check failed");
    return this->result_code;    
   }
   if(!Post_Process()){
    ERROR("download post process failed");
    return this->result_code;
   }
   INFO("update models fuc over");
   return this->result_code;
  }
  // 判断fds对象是否创建成功，找出models list。
  bool init()
  {
    INFO("init fuc");
    if (!this->fds_.DoesClientExist()){
      INFO("client creation failed,please checkout account secret and endpoint");
      this->result_code = static_cast<int32_t>(ModelKeyCode::kFdsClientCreateFailed);
      return false;
    }
    this->fds_version_list = this->fds_.ListObjects(
      this->bucket_name, this->func_name + "/" + this->module_name +"/");
    if (this->fds_version_list.empty()){
      INFO("can not get model version from fds,please checkout network is fine");
      this->result_code = static_cast<int32_t>(ModelKeyCode::kFdsModelNotExist);
      return false;
    }
    INFO("%s has %d versions in fsd",this->module_name.c_str(), this->fds_version_list.size());
    return true;
  }
  
  bool Find_Local_Model_Version()
  {
    INFO("read model version in toml file");
    toml::value value;
    auto local_config_dir = this->model_dir + std::string(
      "version.toml");
    if (access(local_config_dir.c_str(), F_OK) != 0) {
      INFO("%s do not exist!", local_config_dir.c_str());
      INFO("create version toml file");
      int shell_code;
      std::string shell_message;
      bool result = this->Shell("touch "+local_config_dir,shell_code,shell_message);
      if (result == true){
        INFO("create version toml file successfully, set version as 1.0");
      } else {
        this->result_code = static_cast<int32_t>(ModelKeyCode::kShellCmdFailed);
        ERROR("create version toml file failed");
        return false;
      }
      toml::value value;
      if (!CyberdogToml::ParseFile(local_config_dir, value)) {
        this->result_code = static_cast<int32_t>(ModelKeyCode::kTomlParseFailed);
        ERROR("parse toml file failed");
        return false;
      } 
      toml::value temp;
      auto tomlresult = CyberdogToml::Set(temp, "version", std::string("1.0"));
      if(!CyberdogToml::WriteFile(local_config_dir, temp)) {
        this->result_code = static_cast<int32_t>(ModelKeyCode::kTomlWriteFailed);
        ERROR("write toml file failed");
        return false;
      }     
    }
    if (!cyberdog::common::CyberdogToml::ParseFile(
        std::string(local_config_dir), value)){
      this->result_code = static_cast<int32_t>(ModelKeyCode::kTomlParseFailed);  
      ERROR("fail to parse data from toml");
      return false;
    }

    if (!cyberdog::common::CyberdogToml::Get(value, "version", this->nx_version)) {
      this->result_code = static_cast<int32_t>(ModelKeyCode::kTomlGetFailed);
      ERROR("fail to read key queue_size from toml");
    }
    INFO("the model version in toml file is %s ",this->nx_version.c_str());
    return true;
  }

  
  
  
  
  // 下载制定和路径的模型
  bool DownloadModels()
  {
    INFO("DownloadModels fuc");
    if (this->need_download == false){
      INFO("do not need download");
      this->result_code = static_cast<int32_t>(ModelKeyCode::KNoNeedToDownload);
      return false;
    }
    std::string ftp_model_path = this->func_name + "/" + this->module_name
      + "/" + this->download_version+"/";
    INFO("download models from %s in fds",(this->bucket_name+"/"+ftp_model_path).c_str());
    std::vector<std::string> fds_models_list;
    this->temp_download_path = this->prefix_dir + this->func_name + "/" +
      this->module_name +"/" + this->download_version+"/";
    if (!Mk_Folder(this->temp_download_path)){
      ERROR("mk folder error");
      this->result_code = static_cast<int32_t>(ModelKeyCode::kShellCmdFailed);
      return false;
    }
    fds_models_list = this->fds_.ListObjects(
      this->bucket_name, ftp_model_path);
    int begin_time = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    for (auto & obj : fds_models_list) {
      INFO("downloading  %s",obj.c_str());
      bool download_over = this->fds_.GetObject(this->bucket_name, ftp_model_path,
        obj, this->temp_download_path);
      if (!download_over){
        INFO("download %s failed",obj.c_str());
        this->result_code = static_cast<int32_t>(ModelKeyCode::kDownloadFailed);
        return false; 
      }
      std::string md5 = this->fds_.GetObjectMD5(this->bucket_name, ftp_model_path, obj);
      this->md5_map.insert(std::make_pair(obj,md5));
      INFO("%s  md5 is %s",obj.c_str(),md5.c_str());
    }
    int stop_time = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    INFO("downloaded models need %d ms",stop_time -begin_time);
    return true;
  }
  
  bool Post_Process()
  {
    INFO("Post_Process func");
    INFO("mv models to original path");
    INFO("temp_download_path %s",this->temp_download_path.c_str());
    INFO("model_dir %s",this->model_dir.c_str());
    int shell_code;
    std::string shell_message;
    bool result = this->Shell("cp -rf "+this->temp_download_path+"*"+ " "+this->model_dir,shell_code,shell_message);
    if (result == false){
      INFO("mv models to original path failed");
      this->result_code = static_cast<int32_t>(ModelKeyCode::kShellCmdFailed);
      return false;
    }
    INFO("delete temp models");
    result = this->Shell("rm -rf "+this->temp_download_path,shell_code,shell_message);
    if (result == false){
      INFO("delete temp models failed");
      this->result_code = static_cast<int32_t>(ModelKeyCode::kShellCmdFailed);
      return false;
    }
    INFO("change model version in toml");
    auto local_config_dir = this->model_dir + std::string(
      "/version.toml");
    if (access(local_config_dir.c_str(), F_OK) != 0) {
      INFO("%s do not exist!", local_config_dir.c_str());
      this->result_code = static_cast<int32_t>(ModelKeyCode::kTomlFileNotExist);
      return false;
    }
    toml::value value;
    if (!CyberdogToml::ParseFile(local_config_dir, value)) {
      INFO("Parse toml failed");
      this->result_code = static_cast<int32_t>(ModelKeyCode::kTomlParseFailed);
      return false;
    } else {
      toml::value temp;
      auto result = CyberdogToml::Set(temp, "version", this->download_version);
      result = CyberdogToml::WriteFile(local_config_dir, temp);
      if (result == true){
        INFO("change model version successfully");
      } else {
        INFO("change model version failed");
        this->result_code = static_cast<int32_t>(ModelKeyCode::kTomlWriteFailed);
        return false;
      }
    }
    return true;
  }

  bool Md5_Check()
  {
    INFO("Md5_Check func");
    for (auto & obj : this->md5_map){
      bool md5_same = Md5_Compare(this->temp_download_path+obj.first.c_str()
        ,obj.second.c_str());
      if (!md5_same){
        INFO("local md5 is not same as fds,download is incomplete !!!");
        this->result_code = static_cast<int32_t>(ModelKeyCode::kMd5NotSame);
        return false;
      }
    }
    INFO("Md5 Check is ok!!!");
    return true;
  }

  bool Md5_Compare(std::string filepath,std::string md5_string)
  {
      int shell_code;
      std::string shell_message;
      bool result = this->Shell("md5sum "+filepath,shell_code,shell_message);
      if (shell_message.find(md5_string) != std::string::npos){
        INFO("model md5 is same");
        return true;
      } else {
        INFO("model md5 is different");
      }      
  }
  bool Is_Newest_Version()
  {
    for (auto & obj : this->fds_version_list){
      INFO("obj: %s",obj.c_str());
      if(std::stof(obj) > std::stof(this->ftp_newest_version)){
        this->ftp_newest_version = obj.substr(0, obj.length() - 1);
      }
    }
    INFO("ftp_newest_version  is %s",this->ftp_newest_version.c_str());
    INFO("nx_version  is %s",this->nx_version.c_str());
    if (!this->ftp_newest_version.compare(this->nx_version)){
      return true;
    } else {
    return false;
    }
  }
  bool Is_Specified_Version()
  {
    INFO("user specified version is %s",this->specified_version.c_str());
    INFO("nx_version is %s",this->nx_version.c_str());
    if (!this->specified_version.compare(this->nx_version)){
      INFO("specified model version is same as nx model version");
      return true;
     } else {
        INFO("specified model version is differente from nx model version");
        return false;
     }
  }
  // 判断是否需要下载和需要下载的模型版本
  bool Prepare_Download()
  {
    if (this->is_specified_version){
      if(this->Is_Specified_Version()){
        this->need_download = false;
        INFO("do not need download");
      } else {
        this->need_download = true;
        INFO("need download from fds");
        if (std::find(this->fds_version_list.begin(),
            this->fds_version_list.end(), this->specified_version+"/")
            != this->fds_version_list.end()){
          this->download_version = this->specified_version;
          INFO("download_version is %s",this->download_version.c_str());
        } else {
          ERROR("download_version is not in fds model version list");
          this->result_code = static_cast<int32_t>(ModelKeyCode::kModelVersionNotInFds);
          return false;
        }
      }
    } else{
      if (Is_Newest_Version()){
        INFO("nx model version is the newest version");
        this->need_download = false;
      } else {
        INFO("nx model version is not the newest version,"
          "need donwload the newest models from fds ");
        this->need_download = true;
        this->download_version = this->ftp_newest_version;
      }
    }
    return true;
  }



 
  bool File_Exist(std::string model_name)
  {
    std::string filepath = this->model_dir + model_name;
    if (access(filepath.c_str(), F_OK) != 0) {
      INFO("%s do not exist!", filepath.c_str());
      return false;
    } else {
      INFO("%s exist!", filepath.c_str());
      return true;
    }  
  }

  bool Mk_Folder(std::string& path)
  {

    if (access(path.c_str(), F_OK) != 0) {
      INFO("%s do not exist!", path.c_str());
      INFO("mkdir %s",path.c_str());
      int shell_code;
      std::string shell_message;
      bool result = this->Shell("mkdir -p "+path,shell_code,shell_message);
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
  std::string model_dir;
  std::string prefix_dir;
  std::string func_name;
  std::string module_name;
  cyberdog::common::CyberdogFDS fds_;
  std::string nx_version;
  std::string specified_version;
  bool is_specified_version;
  std::vector<std::string> fds_version_list;
  std::string bucket_name = "platform-module";
  bool need_download = false;
  std::string download_version = "0.0";
  std::string ftp_newest_version = "0.0";
  std::string temp_download_path;
  std::map<std::string,std::string> md5_map;
  int32_t result_code = 0;
  LOGGER_MINOR_INSTANCE("cyberdog_model");

};  // class cyberdog_model
}  // namespace cyberdog
}  // namespace common


#endif  // CYBERDOG_COMMON__CYBERDOG_MODEL_HPP_