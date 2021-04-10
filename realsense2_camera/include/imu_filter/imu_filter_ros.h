#ifndef IMU_FILTER_MADWICK_IMU_FILTER_ROS_H
#define IMU_FILTER_MADWICK_IMU_FILTER_ROS_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "tf2_ros/transform_broadcaster.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_reconfigure/server.h>

#include "imu_filter/imu_filter.h"
#include "imu_filter/ImuFilterMadgwickConfig.h"
#ifndef PI
const double PI = 3.14159265358979323846;
#endif
class ImuFilterRos
{
  typedef sensor_msgs::Imu              ImuMsg;

  typedef message_filters::sync_policies::ApproximateTime<ImuMsg, ImuMsg > SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
  typedef message_filters::Subscriber<ImuMsg> ImuSubscriber;
  typedef imu_filter_madgwick::ImuFilterMadgwickConfig   FilterConfig;
  typedef dynamic_reconfigure::Server<FilterConfig>   FilterConfigServer;

  public:

    ImuFilterRos(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~ImuFilterRos();

  private:

    // **** ROS-related

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    boost::shared_ptr<ImuSubscriber> imu_subscriber_;
    boost::shared_ptr<Synchronizer> sync_;

    ros::Publisher rpy_filtered_debug_publisher_;
    ros::Publisher imu_publisher_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    boost::shared_ptr<FilterConfigServer> config_server_;
    ros::Timer check_topics_timer_;

    // **** paramaters
    bool stateless_;
    bool publish_tf_;
    bool reverse_tf_;
    std::string fixed_frame_;
    std::string imu_frame_;
    double constant_dt_;
    bool remove_gravity_vector_;
    double orientation_variance_;

    // **** state variables
    boost::mutex mutex_;
    bool initialized_;
    ros::Time last_time_;

    // **** filter implementation
    ImuFilter filter_;

    // **** member functions
    void imuCallback(const ImuMsg::ConstPtr& imu_msg_raw);

    void publishFilteredMsg(const ImuMsg::ConstPtr& imu_msg_raw);
    void publishTransform(const ImuMsg::ConstPtr& imu_msg_raw);

    void reconfigCallback(FilterConfig& config, uint32_t level);
    void checkTopicsTimerCallback(const ros::TimerEvent&);
};

#endif // IMU_FILTER_IMU_MADWICK_FILTER_ROS_H
