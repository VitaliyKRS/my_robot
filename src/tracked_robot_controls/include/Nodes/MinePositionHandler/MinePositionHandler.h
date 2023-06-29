#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "tracked_robot_msgs/msg/hand_position.hpp"


class MinePositionHandler
    : public rclcpp::Node {
private:
    rclcpp::Subscription<tracked_robot_msgs::msg::HandPosition>::SharedPtr mHandPositionSubscription;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mRobotPositionSubscription;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr mMinePositionPublisher;

    double mRobotX;
    double mRobotY;
    double mRobotTheta;
    double mAngleHand;
    double mLengthHand;

public:
    MinePositionHandler();

private:
    void publishMinePosition();
    void onHandPositionReceived(const tracked_robot_msgs::msg::HandPosition::SharedPtr handPos);
    void onOdometryReceived(const nav_msgs::msg::Odometry::SharedPtr odom);
};