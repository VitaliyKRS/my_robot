#include "nav2_robot_field_planner/FieldPlanner.h"

#include <tf2/LinearMath/Quaternion.h>

namespace field_planner {

using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Path;

FieldPlanner::FieldPlanner()
    : mNavFnPlanner{std::make_unique<nav2_navfn_planner::NavfnPlanner>()}
{
}

void FieldPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
                             std::string name,
                             std::shared_ptr<tf2_ros::Buffer> tf,
                             std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    mNode = parent.lock();
    mGlobalFrame = costmap_ros->getGlobalFrameID();
    mCostmap = costmap_ros->getCostmap();
    mName = name;

    RCLCPP_INFO(mNode->get_logger(), "Configure plugin %s of type FieldPlanner", mName.c_str());
    mNavFnPlanner->configure(parent, name, tf, costmap_ros);
}

void FieldPlanner::cleanup()
{
    RCLCPP_INFO(mNode->get_logger(), "CleaningUp plugin %s of type FieldPlanner", mName.c_str());
    mNavFnPlanner->cleanup();
}

void FieldPlanner::activate()
{
    using namespace std::placeholders;

    RCLCPP_INFO(mNode->get_logger(), "Activate plugin %s of type FieldPlanner", mName.c_str());

    mGetFieldPlanClient = mNode->create_client<nav_msgs::srv::GetPlan>("get_field_plan");
    mNavFnPlanner->activate();
}

void FieldPlanner::deactivate()
{
    RCLCPP_INFO(mNode->get_logger(), "Deactivate plugin %s of type FieldPlanner", mName.c_str());
    mNavFnPlanner->deactivate();
}

Path FieldPlanner::createPlan(const PoseStamped& start, const PoseStamped& goal)
{
    Path path;

    if (mGetFieldPlanClient->wait_for_service(std::chrono::seconds(1))) {
        auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
        request->start = start;
        request->goal = goal;
        request->tolerance = 0.5;
        auto future = mGetFieldPlanClient->async_send_request(request);

        auto result = future.get();

        if (result) {
            RCLCPP_INFO(mNode->get_logger(), "Path received");
            if (!result->plan.poses.empty()) {
                path = result->plan;
            }
            else {
                path = mNavFnPlanner->createPlan(start, goal);
            }
        }
        else {
            RCLCPP_ERROR(mNode->get_logger(), "Interrupted while waiting for response. Exiting.");
        }
    }
    else {
        path = mNavFnPlanner->createPlan(start, goal);
    }

    return path;
}
}  // namespace field_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(field_planner::FieldPlanner, nav2_core::GlobalPlanner)
