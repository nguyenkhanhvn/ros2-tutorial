#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Core>


class SimpleController : public rclcpp::Node
{
public:
    SimpleController(const std::string &name);

private:
    void velCallback(const geometry_msgs::msg::TwistStamped &msg);

private:
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub_;

    double wheel_radius_;
    double wheel_separation_;
    Eigen::Matrix2d speed_conversion_;
};