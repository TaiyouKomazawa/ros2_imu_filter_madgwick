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

#include "imu_filter_ros.h"
#include "stateless_orientation.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

ImuFilterRos::ImuFilterRos(
      const std::string& name_space,
      const rclcpp::NodeOptions& options):
  Node("ImuFilter", name_space, options), tf_broadcaster_(this)
{
  initialized_ = false;
  RCLCPP_INFO (this->get_logger(), "Starting ImuFilter");

  // **** get paramters
  stateless_ = declare_parameter("stateless", false);
  use_mag_ = declare_parameter("use_mag", true);
  publish_tf_ = declare_parameter("publish_tf", true);
  reverse_tf_ = declare_parameter("reverse_tf", false);
  fixed_frame_ = declare_parameter("fixed_frame", "odom");
  constant_dt_ = declare_parameter("constant_dt", 0.0);
  publish_debug_topics_ = declare_parameter("publish_debug_topics", false);

  use_magnetic_field_msg_ = declare_parameter("use_magnetic_field_msg", false);
  // For ROS Jade, make this default to true.
  if (!has_parameter("use_magnetic_field_msg")){
      if (use_mag_){
          RCLCPP_WARN(this->get_logger(), "Deprecation Warning: The parameter use_magnetic_field_msg was not set, default is 'false'.");
          RCLCPP_WARN(this->get_logger(), "Starting with ROS Jade, use_magnetic_field_msg will default to 'true'!");
      }
  }

  std::string world_frame = declare_parameter("world_frame", "nwu");
  // Default should become false for next release
  if (!has_parameter("world_frame")){
    RCLCPP_WARN(this->get_logger(), "Deprecation Warning: The parameter world_frame was not set, default is 'nwu'.");
    RCLCPP_WARN(this->get_logger(), "Starting with ROS Lunar, world_frame will default to 'enu'!");
  }

  if (world_frame == "ned") {
    world_frame_ = WorldFrame::NED;
  } else if (world_frame == "nwu"){
    world_frame_ = WorldFrame::NWU;
  } else if (world_frame == "enu"){
    world_frame_ = WorldFrame::ENU;
  } else {
    RCLCPP_ERROR(this->get_logger(), "The parameter world_frame was set to invalid value '%s'.", world_frame.c_str());
    RCLCPP_ERROR(this->get_logger(), "Valid values are 'enu', 'ned' and 'nwu'. Setting to 'enu'.");
    world_frame_ = WorldFrame::ENU;
  }
  filter_.setWorldFrame(world_frame_);

  // check for illegal constant_dt values
  if (constant_dt_ < 0.0)
  {
    RCLCPP_FATAL(this->get_logger(), "constant_dt parameter is %f, must be >= 0.0. Setting to 0.0", constant_dt_);
    constant_dt_ = 0.0;
  }

  // if constant_dt_ is 0.0 (default), use IMU timestamp to determine dt
  // otherwise, it will be constant
  if (constant_dt_ == 0.0)
    RCLCPP_INFO(this->get_logger(), "Using dt computed from message headers");
  else
    RCLCPP_INFO(this->get_logger(), "Using constant dt of %f sec", constant_dt_);

  // **** register dynamic reconfigure
  declare_parameter("gain", 0.1);
  declare_parameter("zeta", 0.0);
  declare_parameter("mag_bias_x", 0.0);
  declare_parameter("mag_bias_y", 0.0);
  declare_parameter("mag_bias_z", 0.0);
  declare_parameter("orientation_stddev", 0.0);
  reconfig_sub_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
            "/parameter_events", 10,
            std::bind(&ImuFilterRos::reconfigCallback, this, std::placeholders::_1));

  // **** register publishers
  imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
      "imu/data", 5);

  if (publish_debug_topics_)
  {
    rpy_filtered_debug_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "imu/rpy/filtered", 5);

    rpy_raw_debug_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "imu/rpy/raw", 5);
  }

  // **** register subscribers
  // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.

  imu_subscriber_.reset(new ImuSubscriber(this, "imu/data_raw"));

  if (use_mag_)
  {
    if (use_magnetic_field_msg_)
    {
      mag_subscriber_.reset(new MagSubscriber(this, "imu/mag"));
    }
    else
    {
      mag_subscriber_.reset(new MagSubscriber(this, "imu/magnetic_field"));

      // Initialize the shim to support republishing Vector3Stamped messages from /mag as MagneticField
      // messages on the /magnetic_field topic.
      mag_republisher_ = this->create_publisher<MagMsg>("imu/magnetic_field", 5);
      vector_mag_subscriber_.reset(new MagVectorSubscriber(this, "imu/mag"));
      vector_mag_subscriber_->registerCallback(&ImuFilterRos::imuMagVectorCallback, this);
    }

    sync_.reset(new Synchronizer(SyncPolicy(5), *imu_subscriber_, *mag_subscriber_));
    /*sync_->registerCallback(std::bind(&ImuFilterRos::imuMagCallback, this, std::placeholders::_1, std::placeholders::_2));*/
    sync_->registerCallback(&ImuFilterRos::imuMagCallback, this);
  }
  else
  {
    imu_subscriber_->registerCallback(&ImuFilterRos::imuCallback, this);
  }
  check_topics_timer_ = this->create_wall_timer(
      10s, std::bind(&ImuFilterRos::checkTopicsTimerCallback, this));
}

ImuFilterRos::~ImuFilterRos()
{
  RCLCPP_INFO (this->get_logger(), "Destroying ImuFilter");

  // Explicitly stop callbacks; they could execute after we're destroyed
  check_topics_timer_->cancel();
}

void ImuFilterRos::imuCallback(const ImuMsg::SharedPtr imu_msg_raw)
{
  std::unique_lock<std::mutex> lock(mutex_);

  const geometry_msgs::msg::Vector3& ang_vel = imu_msg_raw->angular_velocity;
  const geometry_msgs::msg::Vector3& lin_acc = imu_msg_raw->linear_acceleration;

  rclcpp::Time time = imu_msg_raw->header.stamp;
  imu_frame_ = imu_msg_raw->header.frame_id;

  if (!initialized_)
  {
    check_topics_timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "First IMU message received.");
  }

  if (!initialized_ || stateless_)
  {
    geometry_msgs::msg::Quaternion init_q;
    StatelessOrientation::computeOrientation(world_frame_, lin_acc, init_q);
    filter_.setOrientation(init_q.w, init_q.x, init_q.y, init_q.z);

    // initialize time
    last_time_ = time;
    initialized_ = true;
  }

  // determine dt: either constant, or from IMU timestamp
  float dt;
  if (constant_dt_ > 0.0)
    dt = constant_dt_;
  else
    dt = (time - last_time_).seconds();

  last_time_ = time;

  if (!stateless_)
    filter_.madgwickAHRSupdateIMU(
      ang_vel.x, ang_vel.y, ang_vel.z,
      lin_acc.x, lin_acc.y, lin_acc.z,
      dt);

  publishFilteredMsg(imu_msg_raw);
  if (publish_tf_)
    publishTransform(imu_msg_raw);
}

void ImuFilterRos::imuMagCallback(
  const ImuMsg::SharedPtr imu_msg_raw,
  const MagMsg::SharedPtr mag_msg)
{
  std::unique_lock<std::mutex> lock(mutex_);

  const geometry_msgs::msg::Vector3& ang_vel = imu_msg_raw->angular_velocity;
  const geometry_msgs::msg::Vector3& lin_acc = imu_msg_raw->linear_acceleration;
  const geometry_msgs::msg::Vector3& mag_fld = mag_msg->magnetic_field;

  rclcpp::Time time = imu_msg_raw->header.stamp;
  imu_frame_ = imu_msg_raw->header.frame_id;

  /*** Compensate for hard iron ***/
  geometry_msgs::msg::Vector3 mag_compensated;
  mag_compensated.x = mag_fld.x - mag_bias_.x;
  mag_compensated.y = mag_fld.y - mag_bias_.y;
  mag_compensated.z = mag_fld.z - mag_bias_.z;

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;

  if (!initialized_)
  {
    check_topics_timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "First pair of IMU and magnetometer messages received.");
  }

  if (!initialized_ || stateless_)
  {
    // wait for mag message without NaN / inf
    if(!std::isfinite(mag_fld.x) || !std::isfinite(mag_fld.y) || !std::isfinite(mag_fld.z))
    {
      return;
    }

    geometry_msgs::msg::Quaternion init_q;
    StatelessOrientation::computeOrientation(world_frame_, lin_acc, mag_compensated, init_q);
    filter_.setOrientation(init_q.w, init_q.x, init_q.y, init_q.z);

    last_time_ = time;
    initialized_ = true;
  }

  // determine dt: either constant, or from IMU timestamp
  float dt;
  if (constant_dt_ > 0.0)
    dt = constant_dt_;
  else
    dt = (time - last_time_).seconds();

  last_time_ = time;

  if (!stateless_)
    filter_.madgwickAHRSupdate(
      ang_vel.x, ang_vel.y, ang_vel.z,
      lin_acc.x, lin_acc.y, lin_acc.z,
      mag_compensated.x, mag_compensated.y, mag_compensated.z,
      dt);

  publishFilteredMsg(imu_msg_raw);
  if (publish_tf_)
    publishTransform(imu_msg_raw);

  if(publish_debug_topics_)
  {
    geometry_msgs::msg::Quaternion orientation;
    if (StatelessOrientation::computeOrientation(world_frame_, lin_acc, mag_compensated, orientation))
    {
      tf2::Matrix3x3(tf2::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w)).getRPY(roll, pitch, yaw, 0);
      publishRawMsg(time, roll, pitch, yaw);
    }
  }
}

void ImuFilterRos::publishTransform(const ImuMsg::SharedPtr imu_msg_raw)
{
  double q0,q1,q2,q3;
  filter_.getOrientation(q0,q1,q2,q3);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = imu_msg_raw->header.stamp;
  if (reverse_tf_)
  {
    transform.header.frame_id = imu_frame_;
    transform.child_frame_id = fixed_frame_;
    transform.transform.rotation.w = q0;
    transform.transform.rotation.x = -q1;
    transform.transform.rotation.y = -q2;
    transform.transform.rotation.z = -q3;
  }
  else {
    transform.header.frame_id = fixed_frame_;
    transform.child_frame_id = imu_frame_;
    transform.transform.rotation.w = q0;
    transform.transform.rotation.x = q1;
    transform.transform.rotation.y = q2;
    transform.transform.rotation.z = q3;
  }
  tf_broadcaster_.sendTransform(transform);

}

void ImuFilterRos::publishFilteredMsg(const ImuMsg::SharedPtr imu_msg_raw)
{
  double q0,q1,q2,q3;
  filter_.getOrientation(q0,q1,q2,q3);

  // create and publish filtered IMU message
  std::shared_ptr<ImuMsg> imu_msg =
    std::make_shared<ImuMsg>(*imu_msg_raw);

  imu_msg->orientation.w = q0;
  imu_msg->orientation.x = q1;
  imu_msg->orientation.y = q2;
  imu_msg->orientation.z = q3;

  imu_msg->orientation_covariance[0] = orientation_variance_;
  imu_msg->orientation_covariance[1] = 0.0;
  imu_msg->orientation_covariance[2] = 0.0;
  imu_msg->orientation_covariance[3] = 0.0;
  imu_msg->orientation_covariance[4] = orientation_variance_;
  imu_msg->orientation_covariance[5] = 0.0;
  imu_msg->orientation_covariance[6] = 0.0;
  imu_msg->orientation_covariance[7] = 0.0;
  imu_msg->orientation_covariance[8] = orientation_variance_;

  imu_publisher_->publish(*imu_msg);

  if(publish_debug_topics_)
  {
    geometry_msgs::msg::Vector3Stamped rpy;
    tf2::Matrix3x3(tf2::Quaternion(q1,q2,q3,q0)).getRPY(rpy.vector.x, rpy.vector.y, rpy.vector.z);

    rpy.header = imu_msg_raw->header;
    rpy_filtered_debug_publisher_->publish(rpy);
  }
}

void ImuFilterRos::publishRawMsg(const rclcpp::Time& t,
  float roll, float pitch, float yaw)
{
  geometry_msgs::msg::Vector3Stamped rpy;
  rpy.vector.x = roll;
  rpy.vector.y = pitch;
  rpy.vector.z = yaw ;
  rpy.header.stamp = t;
  rpy.header.frame_id = imu_frame_;
  rpy_raw_debug_publisher_->publish(rpy);
}

void ImuFilterRos::reconfigCallback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
  double gain, zeta;
  std::unique_lock<std::mutex> lock(mutex_);

  if(event->node == this->get_fully_qualified_name())
  {
    this->get_parameter("gain", gain);
    this->get_parameter("zeta", zeta);
    if(has_parameter("gain")){
      filter_.setAlgorithmGain(gain);
      RCLCPP_INFO(this->get_logger(), "Imu filter gain set to %f", gain);
    }
    if(has_parameter("zeta")){
      filter_.setDriftBiasGain(zeta);
      RCLCPP_INFO(this->get_logger(), "Gyro drift bias set to %f", zeta);
    }
    this->get_parameter("mag_bias_x", mag_bias_.x);
    this->get_parameter("mag_bias_y", mag_bias_.y);
    this->get_parameter("mag_bias_z", mag_bias_.z);
    double orientation_stddev;
    this->get_parameter("orientation_stddev", orientation_stddev);
    orientation_variance_ = orientation_stddev * orientation_stddev;
    RCLCPP_INFO(this->get_logger(), "Magnetometer bias values: %f %f %f", mag_bias_.x, mag_bias_.y, mag_bias_.z);
  }
}

void ImuFilterRos::imuMagVectorCallback(const MagVectorMsg::SharedPtr mag_vector_msg)
{
  MagMsg mag_msg;
  mag_msg.header = mag_vector_msg->header;
  mag_msg.magnetic_field = mag_vector_msg->vector;
  // leaving mag_msg.magnetic_field_covariance set to all zeros (= "covariance unknown")
  mag_republisher_->publish(mag_msg);
}

void ImuFilterRos::checkTopicsTimerCallback()
{
  if (use_mag_)
    RCLCPP_WARN(this->get_logger(), "Still waiting for data on topics imu/data_raw and imu/mag...");
  else
    RCLCPP_WARN(this->get_logger(), "Still waiting for data on topic imu/data_raw...");
}
