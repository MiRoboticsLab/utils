//
// Created by quan on 2022/3/23.
//

#include "cyberdog_parameter/cyberdog_parameter.hpp"

#include <string>
#include <vector>

namespace cyberdog
{
namespace parameter
{
void TestCase1()
{
  std::string so_filename = "test_toml.so";
  ParameterParser parser = ParameterParser(so_filename);

  // toml format
  // a = 10
  // b = "cyberdog"
  // c = true
  // d = 0.1
  // aa = ["foo", "bar", "baz"]
  // bb = [1.1, 0.5, 880.123]
  // cc = [1, 5, 8]
  int a = parser.GetInt("a");
  std::string b = parser.GetString("b");
  bool c = parser.GetBool("c");
  bool d = parser.GetDouble("d");
  std::vector<std::string> aa = parser.GetArrayValuesAsStrings("aa");
  std::cout << "a = " << a << std::endl
            << "b = " << b << std::endl
            << "c = " << c << std::endl
            << "d = " << d << std::endl;
  for (auto str : aa) {
    std::cout << str << " ";
  }
  std::cout << std::endl;
}

void TestCase2()
{
    std::string so_filename = "test_toml.so";
    ParameterParser parser = ParameterParser(so_filename);

    // toml format
    // [A]
    // e = 2
    // f = "cyberdog_A"
    // g = true
    // h = 0.2
    int e = parser.GetInt("A.e");
    std::string f = parser.GetString("A.f");
    bool g = parser.GetBool("A.g");
    bool h = parser.GetDouble("A.h");

    std::cout << "e = " << e << std::endl
              << "f = " << f << std::endl
              << "g = " << g << std::endl
              << "h = " << h << std::endl;
}

void TestCase3()
{
    std::string filename = "test_toml.toml";
    ParameterParser parser = ParameterParser(filename);

    // toml format
    // [A.c]
    // e = 22222
    // f = "cyberdog_c"
    // g = false
    // h = 0.2222

    int e = parser.GetInt("A.c.e");
    std::string f = parser.GetString("A.c.f");
    bool g = parser.GetBool("A.c.g");
    bool h = parser.GetDouble("A.c.h");

    std::cout << "e = " << e << std::endl
              << "f = " << f << std::endl
              << "g = " << g << std::endl
              << "h = " << h << std::endl;
}

} // namespace parameter
} // namespace cyberdog

int main(int argc, char **argv)
{
    cyberdog::parameter::TestCase1();
    cyberdog::parameter::TestCase2();
    cyberdog::parameter::TestCase3();
    return 0;
}