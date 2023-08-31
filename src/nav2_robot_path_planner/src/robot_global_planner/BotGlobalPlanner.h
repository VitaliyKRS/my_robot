#pragma once

#include <list>

#include "action_msgs/msg/goal_status_array.hpp"
#include <nav2_core/global_planner.hpp>
#include <nav2_navfn_planner/navfn_planner.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace robot_global_planner {

struct PointF {
    static PointF fromPose(const geometry_msgs::msg::PoseStamped& pose)
    {
        return {pose.pose.position.x, pose.pose.position.y};
    }

    PointF operator+(const PointF& p) const { return {x + p.x, y + p.y}; }

    PointF operator-(const PointF& p) const { return {x - p.x, y - p.y}; }

    PointF operator*(double value) const { return {x * value, y * value}; }

    double x = 0;
    double y = 0;
    bool isValid = true;
};

class RobotGlobalPlanner : public nav2_core::GlobalPlanner {
private:
    std::unique_ptr<nav2_navfn_planner::NavfnPlanner> mNavFnPlanner;
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr mSubscription;
    rclcpp_lifecycle::LifecycleNode::SharedPtr mNode;
    std::string mName;
    std::string mGlobalFrame;
    nav2_costmap_2d::Costmap2D* mCostmap;
    PointF mStartPoint;
    bool mStarted;

public:
    RobotGlobalPlanner();

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
                        
private:

    std::vector<PointF> buildPath(const PointF& start,
                                  const PointF& goal,
                                  const PointF& pos);

    PointF interpolatePoint(const PointF& start,
                                  const PointF& goal,
                                  const PointF& pos);

    bool isCloseToLine(const PointF& start,
                       const PointF& goal,
                       const PointF& pos);

    void onNavigationStatus(const action_msgs::msg::GoalStatusArray& msg);
    size_t getPathIncrements(const PointF& start,
                          const PointF& goal,
                          double& xIncrement,
                          double& yIncrement);
    geometry_msgs::msg::PoseStamped createPose(const PointF& point);
};

}  // namespace robot_global_planner
