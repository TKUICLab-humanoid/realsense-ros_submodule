#include "imu_filter/imu_filter_nodelet.h"
#include <pluginlib/class_list_macros.h>

void ImuFilterNodelet::onInit()
{
  NODELET_INFO("Initializing IMU Filter Nodelet");
  
  // TODO: Do we want the single threaded or multithreaded NH?
  ros::NodeHandle nh         = getMTNodeHandle();
  ros::NodeHandle nh_private = getMTPrivateNodeHandle();

  filter_.reset(new ImuFilterRos(nh, nh_private));
}

PLUGINLIB_EXPORT_CLASS(ImuFilterNodelet, nodelet::Nodelet)
//为了能够动态加载一个类, 那么, 这个类必须是一个被标注的, 并且导入到系统的一个类. 
