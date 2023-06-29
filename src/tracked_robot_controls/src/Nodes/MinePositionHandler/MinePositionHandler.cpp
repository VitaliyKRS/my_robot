#include "Nodes/MinePositionHandler/MinePositionHandler.h"

MinePositionHandler::MinePositionHandler()
    :Node("MinePositionHandler")
    , mRobotX{0}
    , mRobotY{0}
    , mRobotTheta{0}
    , mAngleHand{0}
    , mLengthHand{0}
{
    mHandPositionSubscription = this->create_subscription<tracked_robot_msgs::msg::HandPosition>(
        "/hand_position", 11,
        std::bind(&MinePositionHandler::onHandPositionReceived, this, std::placeholders::_1));

    mRobotPositionSubscription = this-> create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&MinePositionHandler::onOdometryReceived, this, std::placeholders::_1));

    mMinePositionPublisher = this->create_publisher<geometry_msgs::msg::Point>("/mine_position", 10);
}    



void MinePositionHandler::onHandPositionReceived(const tracked_robot_msgs::msg::HandPosition::SharedPtr handPos)
{
    mAngleHand = handPos->angle;
    mLengthHand = handPos->length;

    publishMinePosition();
}

void MinePositionHandler::onOdometryReceived(const nav_msgs::msg::Odometry::SharedPtr odom)
{
    mRobotX = odom->pose.pose.position.x;
    mRobotY = odom->pose.pose.position.y;
    mRobotTheta = 2.0 * atan2(odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
}


void MinePositionHandler::publishMinePosition()
{
    double mineX = mLengthHand * cos(mAngleHand);
    double mineY = mLengthHand * sin(mAngleHand);

    // Transform the mine's relative coordinates to the map frame using the robot's position and orientation
    double transformedMineX = mRobotX + mineX * cos(mRobotTheta) - mineY * sin(mRobotTheta);
    double transformedMineY = mRobotY + mineX * sin(mRobotTheta) + mineY * cos(mRobotTheta);
    
    RCLCPP_INFO(this->get_logger(),"Robot position - x{%f}, y{%f}", mRobotX, mRobotY);

    RCLCPP_INFO(this->get_logger(),"Mine position - x{%f}, y{%f}", transformedMineX, transformedMineY);
    
    auto pointMsg = geometry_msgs::msg::Point();
    pointMsg.x = transformedMineX;
    pointMsg.y = transformedMineY;
    pointMsg.z = 0.0;

    mMinePositionPublisher->publish(pointMsg);
}