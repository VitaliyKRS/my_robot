
#include "nav2_path_follower/PurePursuitPathFollower.h"

using nav2_util::declare_parameter_if_not_declared;
using rcl_interfaces::msg::ParameterType;

namespace nav2_path_follower
{
void PurePursuitPathFollower::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  // TODO: read from parameters
  Parent::configure(parent, name, tf, costmap_ros);
  auto node = node_.lock();

  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }


  declare_parameter_if_not_declared(
    node, plugin_name_ + ".back_distance", rclcpp::ParameterValue(1.2));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".backward_speed", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angle_difference", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".mine_pos_topic", rclcpp::ParameterValue("/mine_position"));


  node->get_parameter(plugin_name_ + ".back_distance", mBackDistance);
  node->get_parameter(plugin_name_ + ".backward_speed", mBackwardSpeed);
  node->get_parameter(plugin_name_ + ".max_angle_difference", mMaxAngleDifference);
  node->get_parameter(plugin_name_ + ".mine_pos_topic", mMinePosTopic);


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
  auto node = node_.lock();
  mDynParamHandler = node->add_on_set_parameters_callback(
    std::bind(
      &PurePursuitPathFollower::dynamicParametersCallback,
      this, std::placeholders::_1));
}
void PurePursuitPathFollower::deactivate()
{
  Parent::deactivate();
  mDynParamHandler.reset();
}
geometry_msgs::msg::TwistStamped PurePursuitPathFollower::computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose, const geometry_msgs::msg::Twist &velocity, nav2_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> lock_reinit(mDynParamMutex);
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

rcl_interfaces::msg::SetParametersResult PurePursuitPathFollower::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(mDynParamMutex);

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".back_distance") {
        mBackDistance = parameter.as_double();
      } else if (name == plugin_name_ + ".backward_speed") {
        mBackwardSpeed = parameter.as_double();
      } else if (name == plugin_name_ + ".max_angle_difference") {
        mMaxAngleDifference = parameter.as_double();
      }
    }
  }
    result.successful = true;
    return result;
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
