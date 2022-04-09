#include <iostream>
#include "gtest/gtest.h"
#include "cyberdog_machine/cyberdog_heartbeats.hpp"


TEST(hello, hello__Test)
{
  std::cout << "hello, gtest!" << std::endl;
  EXPECT_EQ('A', 65);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
