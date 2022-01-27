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
#ifndef CYBERDOG_COMMON__CYBERDOG_JSON_HPP_
#define CYBERDOG_COMMON__CYBERDOG_JSON_HPP_
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h> 
#include <unistd.h>
#include <stdint.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <mutex>
#include "rapidjson/rapidjson.h"
#include "rapidjson/document.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"

namespace cyberdog
{
// using namespace std;
// using namespace rapidjson;
namespace common
{
  using namespace rapidjson;
class CyberdogJson
{
public:
  CyberdogJson() {}
  ~CyberdogJson() {}

public:
  /* construct json data */
  static void Add(Document & doc, const std::string keyName, const int value)
  {
    if (doc.HasMember(keyName.c_str())) {
      doc[keyName.c_str()].SetInt(value);
    } else {
      Document::AllocatorType & allocator = doc.GetAllocator();
      Value key(keyName.c_str(), allocator);
      Value val(value);
      doc.AddMember(key, val, allocator);
    }
  }

  static void Add(Document & doc, const std::string keyName, const uint64_t value)
  {
    if (doc.HasMember(keyName.c_str())) {
      doc[keyName.c_str()].SetUint64(value);
    } else {
      Document::AllocatorType & allocator = doc.GetAllocator();
      Value key(keyName.c_str(), allocator);
      Value val(value);
      doc.AddMember(key, val, allocator);
    }
  }

  static void Add(Document & doc, const std::string keyName, const double value)
  {
    if (doc.HasMember(keyName.c_str())) {
      doc[keyName.c_str()].SetDouble(value);
    } else {
      Document::AllocatorType & allocator = doc.GetAllocator();
      Value key(keyName.c_str(), allocator);
      Value val(value);
      doc.AddMember(key, val, allocator);
    }
  }

  static void Add(Document & doc, const std::string keyName, const float value)
  {
    if (doc.HasMember(keyName.c_str())) {
      doc[keyName.c_str()].SetFloat(value);
    } else {
      Document::AllocatorType & allocator = doc.GetAllocator();
      Value key(keyName.c_str(), allocator);
      Value val(value);
      doc.AddMember(key, val, allocator);
    }
  }

  static void Add(
    Document & doc, const std::string keyName,
    const std::string value)
  {
    Document::AllocatorType & allocator = doc.GetAllocator();
    if (doc.HasMember(keyName.c_str())) {
      doc[keyName.c_str()].SetString(value.c_str(), value.length(), allocator);
    } else {
      Value key(keyName.c_str(), allocator);
      Value val;
      val.SetString(value.c_str(), value.length(), allocator);
      doc.AddMember(key, val, allocator);
    }
  }

  static void Add(Document & doc, const std::string keyName, const char * value)
  {
    Document::AllocatorType & allocator = doc.GetAllocator();
    if (doc.HasMember(keyName.c_str())) {
      doc[keyName.c_str()].SetString(value, strlen(value), allocator);
    } else {
      Value key(keyName.c_str(), allocator);
      Value val;
      val.SetString(value, strlen(value), allocator);
      doc.AddMember(key, val, allocator);
    }
  }

  static void Add(Document & doc, const std::string keyName, const bool value)
  {
    if (doc.HasMember(keyName.c_str())) {
      doc[keyName.c_str()].SetBool(value);
    } else {
      Document::AllocatorType & allocator = doc.GetAllocator();
      Value key(keyName.c_str(), allocator);
      Value val(value);
      doc.AddMember(key, val, allocator);
    }
  }

  static void Add(
    Document & doc, const std::string keyName,
    Document & value)
  {
    Document::AllocatorType & allocator = doc.GetAllocator();
    if (doc.HasMember(keyName.c_str())) {
      doc[keyName.c_str()].CopyFrom(value, allocator);
    } else {
      Document val;
      val.CopyFrom(value, allocator);
      Value key(keyName.c_str(), allocator);
      doc.AddMember(key, val, allocator);
    }
  }

  static void Add(
    Document & doc, const std::string keyName,
    Value & value)
  {
    Document::AllocatorType & allocator = doc.GetAllocator();
    if (doc.HasMember(keyName.c_str())) {
      doc[keyName.c_str()].CopyFrom(value, allocator);
    } else {
      Value val;
      val.CopyFrom(value, allocator);
      Value key(keyName.c_str(), allocator);
      doc.AddMember(key, val, allocator);
    }
  }

  static bool Add(Document & doc, Document & value)
  {
    bool ret = true;
    Document::AllocatorType & allocator = doc.GetAllocator();
    if (doc.IsArray()) {
      static Document val(kObjectType);
      val.CopyFrom(value, allocator);
      doc.PushBack(val, allocator);
    } else {
      ret = false;
      printf("[CyberdogJson][%s] doc is not array, can't add value to it!", __func__);
    }
    return ret;
  }

  static bool Add(Document & doc, Value & value)
  {
    bool ret = true;
    Document::AllocatorType & allocator = doc.GetAllocator();
    if (doc.IsArray()) {
      static Value val(kObjectType);
      val.CopyFrom(value, allocator);
      doc.PushBack(val, allocator);
    } else {
      ret = false;
      printf("[CyberdogJson][%s] doc is not array, can't add value to it!", __func__);
    }
    return ret;
  }

public:
  /* convert json data */
  static bool String2Document(const std::string & str, Document & doc)
  {
    bool ret = true;
    if (str.empty()) {
      printf("[CyberdogJson]%s: str is empty, can't parse to doc!", __func__);
      ret = false;
    } else {
      doc.Parse<0>(str.c_str());
      if (doc.HasParseError()) {
        printf("[CyberdogJson]%s: doc.HasParseError!", __func__);
        ret = false;
      }
    }
    return ret;
  }

  static bool Document2String(const Document & doc, std::string & str)
  {
    if (!doc.IsObject() && !doc.IsArray()) {
      return false;
    }

    StringBuffer s;
    Writer<StringBuffer> writer(s);
    doc.Accept(writer);

    str = std::string(s.GetString());
    return true;
  }

  static void Value2String(Value & val, std::string & valStr)
  {
    StringBuffer buffer;
    Writer<StringBuffer> writer(buffer);
    val.Accept(writer);
    valStr = buffer.GetString();
  }

  static bool Value2Document(Value & val, Document & doc)
  {
    std::string valStr;
    StringBuffer buffer;
    Writer<StringBuffer> writer(buffer);
    val.Accept(writer);
    valStr = buffer.GetString();
    doc.Parse<0>(valStr.c_str());
    if (doc.HasParseError()) {
      printf("[CyberdogJson] %s:HasParseError!", __func__);
      return false;
    } else {
      return true;
    }
  }

  static bool ReadJsonFromFile(const std::string jsonFileName, Document * doc)
  {
    std::string jsonStr;
    doc->SetObject();
    if (ReadFile(jsonFileName, jsonStr)) {
      doc->Parse<0>(jsonStr.c_str());
      if (doc->HasParseError()) {
        printf("[CyberdogJson] %s:HasParseError!", __func__);
        return false;
      } else {
        return true;
      }
    } else {
      return false;
    }
  }

  static bool WriteJsonToFile(const std::string jsonFileName, Document * doc)
  {
    bool ret = false;
    int fd = open(jsonFileName.c_str(), O_WRONLY | O_CREAT | O_SYNC | O_TRUNC, 0660);
    if (fd >= 0) {
      StringBuffer buffer;
      PrettyWriter<StringBuffer> writer(buffer);
      writer.SetFormatOptions(kFormatSingleLineArray);
      doc->Accept(writer);
      std::string msg = buffer.GetString();
      int msg_size = msg.size();
      int len = 0;
      int n;
      while (1) {
        int wr_len = msg_size - len;
        n = write(fd, msg.c_str() + len, wr_len);
        if (n < 0) {
          printf("[CyberdogJson] %s: write file failed!", __func__);
          break;
        } else if (n == wr_len) {
          ret = true;
          printf("[CyberdogJson] %s: msg_size: %d, write %d bytes!", __func__, msg_size, n);
          break;
        }
        len += n;
      }
      close(fd);
    }
    return ret;
  }

private:
  static bool ReadFile(const std::string jsonFileName, std::string & jsonStr)
  {
    bool ret = false;
    std::ifstream in_file;
    in_file.open(jsonFileName.c_str(), std::ios::in);
    if (in_file.is_open()) {
      std::ostringstream sin;
      sin << in_file.rdbuf();
      jsonStr = sin.str();
      in_file.close();
      ret = true;
    }
    return ret;
  }

public:
  /* parse json data */
  static bool Get(
    const Document & doc, const char * key,
    std::string & value)
  {
    if (!doc.HasMember(key) || !doc[key].IsString()) {
      value = "parse error";
      return false;
    }

    value = doc[key].GetString();
    return true;
  }
  static bool Get(const Document & doc, const char * key, int & value)
  {
    if (!doc.HasMember(key) || !doc[key].IsInt()) {
      return false;
    }

    if (doc[key].IsInt()) {
      value = doc[key].GetInt();
    }
    return true;
  }
  static bool Get(const Document & doc, const char * key, uint64_t & value)
  {
    if (!doc.HasMember(key) || !doc[key].IsUint64()) {
      return false;
    }

    value = doc[key].GetUint64();
    return true;
  }
  static bool Get(
    Document & doc, const char * key,
    Value & value)
  {
    if (!doc.HasMember(key)) {
      return false;
    }

    value.CopyFrom(doc[key], doc.GetAllocator());
    return true;
  }
  static bool Get(const Document & doc, const char * key, float & value)
  {
    if (!doc.HasMember(key) || !(doc[key].IsFloat() || doc[key].IsInt())) {
      return false;
    }

    if (doc[key].IsFloat()) {
      value = doc[key].GetFloat();
    } else if (doc[key].IsInt()) {
      value = doc[key].GetInt();
    }

    return true;
  }
  static bool Get(const Document & doc, const char * key, bool & value)
  {
    if (!doc.HasMember(key) || !doc[key].IsBool()) {
      return false;
    }

    value = doc[key].GetBool();
    return true;
  }
  static bool Get(const Value & val, const char * key, std::string & value)
  {
    if (!val.HasMember(key) || !val[key].IsString()) {
      value = "parse error";
      return false;
    }

    value = val[key].GetString();
    return true;
  }
  static bool Get(const Value & val, const char * key, int & value)
  {
    if (!val.HasMember(key) || !val[key].IsInt()) {
      return false;
    }

    value = val[key].GetInt();
    return true;
  }
  static bool Get(const Value & val, const char * key, bool & value)
  {
    if (!val.HasMember(key) || !val[key].IsBool()) {
      return false;
    }

    value = val[key].GetBool();
    return true;
  }

};  // class CyberdogJson
}  // namespace common
}  // namespace cyberdog
#endif  // CYBERDOG_COMMON__CYBERDOG_JSON_HPP_
