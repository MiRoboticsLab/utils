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
#ifndef CYBERDOG_COMMON__CYBERDOG_TOML_HPP_
#define CYBERDOG_COMMON__CYBERDOG_TOML_HPP_
#include <iostream>
#include <string>
#include <vector>
#include "toml/toml.hpp"

namespace cyberdog
{
namespace common
{
// using namespace toml;
class CyberdogToml final
{
public:
  CyberdogToml() {}
  ~CyberdogToml() {}

public:
  /* Trans data between toml and file. */
  static bool ParseFile(const std::string & file_name, toml::value & v)
  {
    try {
      v = toml::parse(file_name);
      return true;
    } catch (const std::exception & e) {
      std::cerr << e.what() << '\n';
      return false;
    }
  }

  static bool WriteFile(const std::string & file_name, const toml::value & v)
  {
    try {
      std::ofstream ofs(file_name, std::ofstream::out);
      ofs << v;
      ofs.close();
      return true;
    } catch (const std::exception & e) {
      std::cerr << e.what() << '\n';
      return false;
    }

  }

  template<typename T>
  static bool Get(const toml::value & v, const std::string & k, T & m)
  {
    if ((v.is_uninitialized()) || (!v.is_table()) || (v.count(k) == 0)) {
      return false;
    } else {
      m = toml::get<T>(v.at(k));
      return true;
    }
  }

  template<typename T>
  static bool Get(const toml::value & v, size_t k, T & m)
  {
    if ((v.is_uninitialized()) || (!v.is_array()) || (!(v.size() > k))) {
      return false;
    } else {
      m = toml::get<T>(v.at(k));
      return true;
    }
  }

  template<typename T>
  static bool Set(toml::value & v, const std::string & k, const T & m)
  {
    if ((!v.is_uninitialized()) && (!v.is_table())) {
      return false;
    } else {
      v[k] = m;
      return true;
    }
  }

  template<typename T>
  static bool Set(toml::value & v, const T & m)
  {
    if (v.is_uninitialized()) {
      std::vector<int> t;
      v = t;
      v.as_array().push_back(m);
    } else if (!v.is_array()) {
      return false;
    }
    return true;
  }

  template<typename T>
  static bool Set(toml::value & v, size_t k, const T & m)
  {
    if (v.is_uninitialized() || (!v.is_array()) || (!(v.size() > k))) {
      return false;
    } else {
      v[k] = m;
      return true;
    }
  }
};  // class CyberdotToml
}  // namespace common
}  // namespace cyberdog

#endif  // CYBERDOG_COMMON__CYBERDOG_TOML_HPP_
