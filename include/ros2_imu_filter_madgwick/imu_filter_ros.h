/*
 *  Copyright (C) 2010, CCNY Robotics Lab
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *
 *  http://robotics.ccny.cuny.edu
 *
 *  Based on implementation of Madgwick's IMU and AHRS algorithms.
 *  http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
 *
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef IMU_FILTER_MADWICK_IMU_FILTER_ROS_H
#define IMU_FILTER_MADWICK_IMU_FILTER_ROS_H

#include "rclcpp/rclcpp.hpp"

#include <chrono>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "imu_filter.h"

using namespace std::chrono_literals;

class ImuFilterRos : public rclcpp::Node
{
  typedef sensor_msgs::msg::Imu              ImuMsg;
  typedef sensor_msgs::msg::MagneticField    MagMsg;
  typedef geometry_msgs::msg::Vector3Stamped MagVectorMsg;

  typedef message_filters::sync_policies::ApproximateTime<ImuMsg, MagMsg> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
  typedef message_filters::Subscriber<ImuMsg> ImuSubscriber;
  typedef message_filters::Subscriber<MagMsg> MagSubscriber;
  typedef message_filters::Subscriber<MagVectorMsg> MagVectorSubscriber;

  //typedef imu_filter_madgwick::ImuFilterMadgwickConfig   FilterConfig;
  //typedef dynamic_reconfigure::Server<FilterConfig>   FilterConfigServer;

  public:
    ImuFilterRos(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    ): ImuFilterRos("", options){}
    ImuFilterRos(
      const std::string& name_space,
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    );
    virtual ~ImuFilterRos();

  private:

    // **** ROS-related

    std::shared_ptr<ImuSubscriber> imu_subscriber_;
    std::shared_ptr<MagSubscriber> mag_subscriber_;
    std::shared_ptr<Synchronizer> sync_;

    // Adapter to support the use_magnetic_field_msg param.
    std::shared_ptr<MagVectorSubscriber> vector_mag_subscriber_;

    rclcpp::Publisher<MagMsg>::SharedPtr mag_republisher_;

    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr rpy_filtered_debug_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr rpy_raw_debug_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    //std::shared_ptr<FilterConfigServer> config_server_;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr reconfig_sub_;
    rclcpp::TimerBase::SharedPtr check_topics_timer_;

    // **** paramaters
    WorldFrame::WorldFrame world_frame_;
    bool use_mag_;
    bool use_magnetic_field_msg_;
    bool stateless_;
    bool publish_tf_;
    bool reverse_tf_;
    std::string fixed_frame_;
    std::string imu_frame_;
    double constant_dt_;
    bool publish_debug_topics_;
    geometry_msgs::msg::Vector3 mag_bias_;
    double orientation_variance_;

    // **** state variables
    std::mutex mutex_;
    std::timed_mutex timed_mutex_;
    bool initialized_;
    rclcpp::Time last_time_;

    // **** filter implementation
    ImuFilter filter_;

    // **** member functions
    void imuMagCallback(const ImuMsg::SharedPtr imu_msg_raw,
                        const MagMsg::SharedPtr mav_msg);

    void imuCallback(const ImuMsg::SharedPtr imu_msg_raw);

    void imuMagVectorCallback(const MagVectorMsg::SharedPtr mag_vector_msg);

    void publishFilteredMsg(const ImuMsg::SharedPtr imu_msg_raw);
    void publishTransform(const ImuMsg::SharedPtr imu_msg_raw);

    void publishRawMsg(const rclcpp::Time& t,
                       float roll, float pitch, float yaw);

    void reconfigCallback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);
    void checkTopicsTimerCallback();
};

#endif // IMU_FILTER_IMU_MADWICK_FILTER_ROS_H
