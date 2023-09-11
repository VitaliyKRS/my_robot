#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

class MinePositionHandler : public rclcpp::Node {
private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mRobotPositionSubscription;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr mMinePositionPublisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mMineSubscription;

    double mRobotX;
    double mRobotY;
    double mRobotTheta;
    double mHandLenght;
    geometry_msgs::msg::Point mOldPosition;

public:
    MinePositionHandler();

private:
    void publishMinePosition();
    void onMineDetected(const std_msgs::msg::Bool::SharedPtr msg);
    void onOdometryReceived(const nav_msgs::msg::Odometry::SharedPtr odom);
};