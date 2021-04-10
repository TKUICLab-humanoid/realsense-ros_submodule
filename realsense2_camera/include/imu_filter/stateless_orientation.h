#ifndef IMU_FILTER_MADWICK_STATELESS_ORIENTATION_H
#define IMU_FILTER_MADWICK_STATELESS_ORIENTATION_H

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

class StatelessOrientation
{
public:
  static bool computeOrientation(
    geometry_msgs::Vector3 acceleration,
    geometry_msgs::Vector3 magneticField,
    geometry_msgs::Quaternion& orientation);

  static bool computeOrientation(
    geometry_msgs::Vector3 acceleration,
    geometry_msgs::Quaternion& orientation);

};

#endif // IMU_FILTER_MADWICK_STATELESS_ORIENTATION_H
