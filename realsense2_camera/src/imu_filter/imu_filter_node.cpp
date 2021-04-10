#include "imu_filter/imu_filter_ros.h"

int main (int argc, char **argv)
{
  ros::init (argc, argv, "ImuFilter");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ImuFilterRos imu_filter(nh, nh_private);
  ros::spin();
  return 0;
}
