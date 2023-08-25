#include "rviz_cloud_annotation_c_interface.h"

#include <iostream>

int main(int argc, char ** argv)
{
  if (argc < 2)
  {
    std::cout << "Usage: rviz_cloud_annotation_test cloud.pcd" << std::endl;
    std::exit(1);
  }

  std::string cloud_filename = argv[1];
  std::cout << "rviz_cloud_annotation_test: Loading cloud " << cloud_filename << std::endl;

  int result = rviz_cloud_annotation_loadcloud(cloud_filename.c_str());

  std::cout << "Result is " << result << std::endl;

  return 0;
}
