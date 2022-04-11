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
  double d = parser.GetDouble("d");
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
    double h = parser.GetDouble("A.h");

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
    double h = parser.GetDouble("A.c.h");

    std::cout << "e = " << e << std::endl
              << "f = " << f << std::endl
              << "g = " << g << std::endl
              << "h = " << h << std::endl;
}

void TestCase4()
{
   std::string filename = "test_toml.toml";
    ParameterParser parser = ParameterParser(filename);
    // [[C]]
    // a = 5555
    // b = "cyberdogCCCC"
    // c = true
    // d = 555.6

    // [[C]]
    // a = 666
    // b = "robotlabCCCCC"
    // c = true
    // d = 666.203

    // b.a 0 ==> b[0].a
    // C[0]
    int C_0_a = parser.GetArrayTableOfInteger("C.a", 0);
    std::string C_0_b = parser.GetArrayTableOfString("C.b", 0);
    bool C_0_c = parser.GetArrayTableOfBool("C.c", 0);
    double C_0_d = parser.GetArrayTableOfDouble("C.d", 0);

    std::cout << "C_0_a = " << C_0_a << std::endl
              << "C_0_b = " << C_0_b << std::endl
              << "C_0_c = " << C_0_c << std::endl
              << "C_0_d = " << C_0_d << std::endl;

    // C[1]
    int C_1_a = parser.GetArrayTableOfInteger("C.a", 1);
    std::string C_1_b = parser.GetArrayTableOfString("C.b", 1);
    bool C_1_c = parser.GetArrayTableOfBool("C.c", 1);
    double C_1_d = parser.GetArrayTableOfDouble("C.d", 1);

    std::cout << "C_1_a = " << C_1_a << std::endl
              << "C_1_b = " << C_1_b << std::endl
              << "C_1_c = " << C_1_c << std::endl
              << "C_1_d = " << C_1_d << std::endl;

}

} // namespace parameter
} // namespace cyberdog

int main(int argc, char **argv)
{
    // cyberdog::parameter::TestCase1();
    // cyberdog::parameter::TestCase2();
    // cyberdog::parameter::TestCase3();
    cyberdog::parameter::TestCase4();
    return 0;
}