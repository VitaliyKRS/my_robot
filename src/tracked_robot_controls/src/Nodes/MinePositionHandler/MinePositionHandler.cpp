#include "Nodes/MinePositionHandler/MinePositionHandler.h"

MinePositionHandler::MinePositionHandler()
    :Node("MinePositionHandler")
    , mRobotX{0}
    , mRobotY{0}
    , mRobotTheta{0}
    , mHandLenght{0.7}
{

    mOldPosition = geometry_msgs::msg::Point();
    mOldPosition.x = 0.0;
    mOldPosition.y = 0.0;
    mOldPosition.z = 0.0;

    mRobotPositionSubscription = this-> create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&MinePositionHandler::onOdometryReceived, this, std::placeholders::_1));

    
    mMineSubscription = this->create_subscription<std_msgs::msg::Bool>(
        "/mine_detection", 12,
        std::bind(&MinePositionHandler::onMineDetected, this, std::placeholders::_1));
    mMinePositionPublisher = this->create_publisher<geometry_msgs::msg::Point>("/mine_position", 10);
}    

void MinePositionHandler::onOdometryReceived(const nav_msgs::msg::Odometry::SharedPtr odom)
{
    mRobotX = odom->pose.pose.position.x;
    mRobotY = odom->pose.pose.position.y;
    mRobotTheta = 2.0 * atan2(odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
}


void MinePositionHandler::publishMinePosition()
{
    double mineX = mHandLenght;
    double mineY = 0.0;

    // Transform the mine's relative coordinates to the map frame using the robot's position and orientation
    double transformedMineX = mRobotX + mineX * cos(mRobotTheta) - mineY * sin(mRobotTheta);
    double transformedMineY = mRobotY + mineX * sin(mRobotTheta) + mineY * cos(mRobotTheta);

    auto pointMsg = geometry_msgs::msg::Point();
    pointMsg.x = transformedMineX;
    pointMsg.y = transformedMineY;
    pointMsg.z = 0.0;

    if(std::sqrt((pointMsg.x -  mOldPosition.x) * (pointMsg.x -  mOldPosition.x) +
                (pointMsg.y -  mOldPosition.y) * (pointMsg.y -  mOldPosition.y)) > 0.5) 
    {
                 
    RCLCPP_INFO(this->get_logger(),"Robot position - x{%f}, y{%f}", mRobotX, mRobotY);

    RCLCPP_INFO(this->get_logger(),"Mine position - x{%f}, y{%f}", transformedMineX, transformedMineY);
    
    mMinePositionPublisher->publish(pointMsg);
    }

    mOldPosition = pointMsg;
    
   
}
void MinePositionHandler::onMineDetected(const std_msgs::msg::Bool::SharedPtr msg)
{
    if(msg->data)
    publishMinePosition();
}