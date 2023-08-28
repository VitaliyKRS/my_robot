
#include "nav2_path_follower/PurePursuitPathFollower.h"

namespace nav2_path_follower
{
void PurePursuitPathFollower::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  // TODO: read from parameters
  Parent::configure(parent, name, tf, costmap_ros);

  mBackDistance = 1.2;
  mBackwardSpeed = 2.0;
  mMaxAngleDifference = 0.5;
  mMinePosTopic = "/mine_position";
  auto node = node_.lock();

  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }


  mMinePositionSubscription = node->create_subscription<geometry_msgs::msg::Point>(
   mMinePosTopic, 10, std::bind(
      &PurePursuitPathFollower::onMinePositionReceived, this,
      std::placeholders::_1));
}
void PurePursuitPathFollower::cleanup()
{
  Parent::cleanup();
}
void PurePursuitPathFollower::activate()
{
  Parent::activate();
}
void PurePursuitPathFollower::deactivate()
{
  Parent::deactivate();
}
geometry_msgs::msg::TwistStamped PurePursuitPathFollower::computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose, const geometry_msgs::msg::Twist &velocity, nav2_core::GoalChecker * goal_checker)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  if(shouldDriveBackward(pose)) {
       RCLCPP_INFO(
    logger_,
    "Drive backward"
    " nav2_path_follower::PurePursuitPathFollower");
      cmd_vel = computeBackWardVelocityCommand(pose);
  }else {
      cmd_vel = Parent::computeVelocityCommands(pose, velocity, goal_checker);
  }

  return cmd_vel;
}

void PurePursuitPathFollower::setPlan(const nav_msgs::msg::Path &path)
{
 Parent::setPlan(path);
}

void PurePursuitPathFollower::setSpeedLimit(const double &speed_limit, const bool &percentage)
{
  Parent::setSpeedLimit(speed_limit, percentage);
}

void PurePursuitPathFollower::onMinePositionReceived(const geometry_msgs::msg::Point::SharedPtr minePos)
{
   RCLCPP_INFO(
    logger_,
    "Mine received"
    " nav2_path_follower::PurePursuitPathFollower");

    mMinePosition.x = minePos->x;
    mMinePosition.y = minePos->y;
}

geometry_msgs::msg::TwistStamped PurePursuitPathFollower::computeBackWardVelocityCommand(const geometry_msgs::msg::PoseStamped &pose)
{
  geometry_msgs::msg::TwistStamped cmd_vel;

  // Set linear velocity for driving backward
  cmd_vel.twist.linear.x = -mBackwardSpeed;

  // Set angular velocity to zero
  cmd_vel.twist.angular.z = 0.0;

  // Set the header of the TwistStamped message
  cmd_vel.header = pose.header;

  return cmd_vel;
}

bool PurePursuitPathFollower::shouldDriveBackward(const geometry_msgs::msg::PoseStamped &pose)
{
   // Calculate the distance between the robot and the mine position
  double distance_to_mine = std::hypot(pose.pose.position.x - mMinePosition.x, pose.pose.position.y - mMinePosition.y);

  double angle_to_mine = atan2(mMinePosition.y - pose.pose.position.y, mMinePosition.x - pose.pose.position.x);
  double angle_difference = fabs(tf2::getYaw(pose.pose.orientation) - angle_to_mine);

  // Check if the robot is close enough to the mine position and if the mine is straight ahead
  return distance_to_mine < mBackDistance && angle_difference < mMaxAngleDifference;
}

} // namespace nav2_path_follower

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  nav2_path_follower::PurePursuitPathFollower,
  nav2_core::Controller)
