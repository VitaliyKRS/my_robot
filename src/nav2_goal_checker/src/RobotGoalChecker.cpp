#include "nav2_goal_checker/RobotGoalChecker.h"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <limits>
#include <memory>
#include <string>
#include <vector>

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_goal_checker {

RobotGoalChecker::RobotGoalChecker()
    : mGoalTolerance(0.25)

{
}

void RobotGoalChecker::initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
    const std::string& plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /*costmap_ros*/)
{
    mPlugName = plugin_name;
    auto node = parent.lock();

    nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".goal_tolerance",
                                                 rclcpp::ParameterValue(0.1));

    node->get_parameter(plugin_name + ".goal_tolerance", mGoalTolerance);

    // Add callback for dynamic parameters
    mDynParamCallback = node->add_on_set_parameters_callback(
        std::bind(&RobotGoalChecker::dynamicParametersCallback, this, _1));
}

void RobotGoalChecker::reset() { return; }

bool RobotGoalChecker::isGoalReached(const geometry_msgs::msg::Pose& query_pose,
                                     const geometry_msgs::msg::Pose& goal_pose,
                                     const geometry_msgs::msg::Twist&)
{
    double distance_to_goal = std::hypot(query_pose.position.x - goal_pose.position.x,
                                         query_pose.position.y - goal_pose.position.y);
    return distance_to_goal <= mGoalTolerance;
}

bool RobotGoalChecker::getTolerances(geometry_msgs::msg::Pose& pose_tolerance,
                                     geometry_msgs::msg::Twist& vel_tolerance)
{
    double invalid_field = std::numeric_limits<double>::lowest();

    pose_tolerance.position.x = mGoalTolerance;
    pose_tolerance.position.y = mGoalTolerance;
    pose_tolerance.position.z = invalid_field;
    pose_tolerance.orientation = nav2_util::geometry_utils::orientationAroundZAxis(mGoalTolerance);

    vel_tolerance.linear.x = invalid_field;
    vel_tolerance.linear.y = invalid_field;
    vel_tolerance.linear.z = invalid_field;

    vel_tolerance.angular.x = invalid_field;
    vel_tolerance.angular.y = invalid_field;
    vel_tolerance.angular.z = invalid_field;

    return true;
}

rcl_interfaces::msg::SetParametersResult RobotGoalChecker::dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    for (auto& parameter : parameters) {
        const auto& type = parameter.get_type();
        const auto& name = parameter.get_name();

        if (type == ParameterType::PARAMETER_DOUBLE) {
            if (name == mPlugName + ".goal_tolerance") {
                mGoalTolerance = parameter.as_double();
            }
        }
    }
    result.successful = true;
    return result;
}

}  // namespace nav2_goal_checker

PLUGINLIB_EXPORT_CLASS(nav2_goal_checker::RobotGoalChecker, nav2_core::GoalChecker)
