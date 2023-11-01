#pragma once

#include <list>

#include "action_msgs/msg/goal_status_array.hpp"
#include "tracked_robot_msgs/srv/field_plan.hpp"
#include "tracked_robot_msgs/srv/update_plan.hpp"
#include <nav2_core/global_planner.hpp>
#include <nav2_navfn_planner/navfn_planner.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace field_planner {

class FieldPlanner : public nav2_core::GlobalPlanner {
private:
    std::unique_ptr<nav2_navfn_planner::NavfnPlanner> mNavFnPlanner;
    rclcpp::Client<tracked_robot_msgs::srv::FieldPlan>::SharedPtr mGetFieldPlanClient;
    rclcpp::Client<tracked_robot_msgs::srv::UpdatePlan>::SharedPtr mUpdatePlanClient;
    rclcpp_lifecycle::LifecycleNode::SharedPtr mNode;
    std::string mName;
    std::string mGlobalFrame;
    nav2_costmap_2d::Costmap2D* mCostmap;
    geometry_msgs::msg::PoseStamped afterObstacle(nav_msgs::msg::Path& path);
    nav_msgs::msg::Path buildFieldPlan(nav_msgs::msg::Path& path,
                                       const geometry_msgs::msg::PoseStamped& start);
    bool in_obstacle(const geometry_msgs::msg::PoseStamped& pos);
    geometry_msgs::msg::PoseStamped mBeforeObstacle;

public:
    FieldPlanner();

public:  // nav2_core::GlobalPlanner
    void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
                   std::string name,
                   std::shared_ptr<tf2_ros::Buffer> tf,
                   std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override;

    void activate() override;

    void deactivate() override;

    nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped& start,
                                   const geometry_msgs::msg::PoseStamped& goal) override;
};

}  // namespace field_planner
