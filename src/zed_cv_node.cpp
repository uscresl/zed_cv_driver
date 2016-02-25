
#include "ros/ros.h"
#include <nodelet/loader.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "zed_cv_driver");

  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  std::string nodelet_name = ros::this_node::getName();
  nodelet.load(nodelet_name, "zed_cv_driver/ZedCvNodelet", remap, nargv);

  ros::spin();

  return 0;
}
