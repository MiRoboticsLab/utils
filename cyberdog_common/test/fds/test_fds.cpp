#include <string>
#include <vector>
#include <iostream>

#include "cyberdog_common/cyberdog_fds.hpp"

double persentage;

void callback(double per)
{
  persentage = per;
  std::cout << "progress: " << persentage * 100 << "%" << std::endl;
}

int main(int argc, char** argv)
{
  if (argc < 3)
  {
    std::cout << "account and secret" << std::endl;
    return -1;
  }
  std::string account(argv[1]), secret(argv[2]);
  cyberdog::common::CyberdogFDS fds_test(account, secret, "cnbj2m-fds.api.xiaomi.net");
  // or cyberdog::common::CyberdogFDS fds_test; fds_test.CreateFdsClient(account, secret, "cnbj2m-fds.api.xiaomi.net");
  if (!fds_test.DoesClientExist())
  {
    std::cout << "client creation failed" << std::endl;
  }
  /*
  auto bucket_list = fds_test.ListBuckets();
  for (auto & bucket : bucket_list)
  {
    std::cout << bucket << std::endl;
  }
  */
  std::vector<std::string> object_list = fds_test.ListObjects("platform-module", "algo/test-inner/");
  std::cout << "got object list. list size is " << object_list.size() <<std::endl;
  for (auto & obj : object_list)
  {
    std::cout << obj << std::endl;
  }
  object_list = fds_test.ListObjects("platform-module", "algo/");
  std::cout << "got object list. list size is " << object_list.size() <<std::endl;
  for (auto & obj : object_list)
  {
    std::cout << obj << std::endl;
  }
  if (fds_test.GetObject("platform-module", "algo/test-inner/", "nx_mobilenetv2_tsm.trt", "/home/mi/", callback))  // or fds_test.GetObject("platform-module", "algo/test-inner/nx_mobilenetv2_tsm.trt", "/home/mi/")
  {
    std::cout << "Successfully downloaded file!" << std::endl;
    std::cout << "file md5 is " << fds_test.GetObjectMD5("platform-module", "algo/test-inner/", "nx_mobilenetv2_tsm.trt") << std::endl;
    // or fds_test.GetObjectMD5("platform-module", "algo/test-inner/nx_mobilenetv2_tsm.trt")
    std::cout << "file size is " << fds_test.GetObjectSize("platform-module", "algo/test-inner/", "nx_mobilenetv2_tsm.trt") << std::endl;
    // or fds_test.GetObjectSize("platform-module", "algo/test-inner/nx_mobilenetv2_tsm.trt")
    std::cout << "file type is " << fds_test.GetObjectType("platform-module", "algo/test-inner/", "nx_mobilenetv2_tsm.trt") << std::endl;
    // or fds_test.GetObjectType("platform-module", "algo/test-inner/nx_mobilenetv2_tsm.trt")
  }
  return 0;
}