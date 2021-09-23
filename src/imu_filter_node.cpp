#include <rclcpp/rclcpp.hpp>
#include "imu_filter_ros.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuFilterRos>());
    rclcpp::shutdown();
    return 0;
}