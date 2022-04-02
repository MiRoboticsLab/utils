#include <stdio.h>
#include <dlfcn.h>
#include <iostream>
#include <string>
#include "cyberdog_common/cyberdog_toml.hpp"


int main(int argc, char ** argv)
{
  (void)argc;
  (void)argv;

  void * handler = dlopen("home/dukun/workspace/cyberdog_ws/utils/cyberdog_common/include/cyberdog_common/libconfig_test.so", RTLD_LAZY);
  char (*get_config_test)() = (char (*)())dlsym(handler, "get_config_test");
  std::cout << get_config_test() << std::endl;
  dlclose(handler);
  return 0;
}