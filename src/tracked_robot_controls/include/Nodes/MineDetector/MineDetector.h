#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


class MineDetector
    : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mSubscription;

public:
    MineDetector();

private:
    void onImageReceived(const sensor_msgs::msg::Image::SharedPtr image);
};