#ifndef IMU_FILTER_MADGWICK_IMU_FILTER_NODELET_H
#define IMU_FILTER_MADGWICK_IMU_FILTER_NODELET_H

#include <nodelet/nodelet.h>
#include "imu_filter/imu_filter_ros.h"

class ImuFilterNodelet : public nodelet::Nodelet
{
  public:
    virtual void onInit();

  private:
    boost::shared_ptr<ImuFilterRos> filter_;
};

#endif // IMU_FILTER_MADGWICK_IMU_FILTER_NODELET_H
