#include "nav2_robot_path_planner/BotGlobalPlanner.h"

#include <tf2/LinearMath/Quaternion.h>

namespace robot_global_planner {

using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Path;

constexpr float RESOLUTION = 0.1f;
constexpr float DEVIATION_TRESHOLD = 0.15f;

RobotGlobalPlanner::RobotGlobalPlanner()
    : mNavFnPlanner{std::make_unique<nav2_navfn_planner::NavfnPlanner>()}
    , mStartPoint{false}
    , mGoalPoint{false}
{
}

void RobotGlobalPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
                                   std::string name,
                                   std::shared_ptr<tf2_ros::Buffer> tf,
                                   std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    mNode = parent.lock();
    mGlobalFrame = costmap_ros->getGlobalFrameID();
    mCostmap = costmap_ros->getCostmap();
    mName = name;

    RCLCPP_INFO(mNode->get_logger(), "Configure plugin %s of type RobotGlobalPlanner",
                mName.c_str());
    mNavFnPlanner->configure(parent, name, tf, costmap_ros);
}

void RobotGlobalPlanner::cleanup()
{
    RCLCPP_INFO(mNode->get_logger(), "CleaningUp plugin %s of type RobotGlobalPlanner",
                mName.c_str());
    mNavFnPlanner->cleanup();
}

void RobotGlobalPlanner::activate()
{
    using namespace std::placeholders;

    RCLCPP_INFO(mNode->get_logger(), "Activate plugin %s of type RobotGlobalPlanner",
                mName.c_str());
    mNavFnPlanner->activate();

    mSubscription = mNode->create_subscription<action_msgs::msg::GoalStatusArray>(
        "navigate_to_pose/_action/status", 10,
        std::bind(&RobotGlobalPlanner::onNavigationStatus, this, _1));
}

void RobotGlobalPlanner::deactivate()
{
    RCLCPP_INFO(mNode->get_logger(), "Deactivate plugin %s of type RobotGlobalPlanner",
                mName.c_str());
    mNavFnPlanner->deactivate();
}

size_t RobotGlobalPlanner::getPathIncrements(const PointF& start,
                                             const PointF& goal,
                                             double& xIncrement,
                                             double& yIncrement)
{
    size_t loopsCount = std::hypot(goal.x - start.x, goal.y - start.y) / RESOLUTION;
    if (loopsCount != 0) {
        xIncrement = (goal.x - start.x) / loopsCount;
        yIncrement = (goal.y - start.y) / loopsCount;
    }

    return loopsCount;
}

std::vector<PointF> RobotGlobalPlanner::avoidObstacle(const PointF& before, const PointF& after)
{
    std::vector<PointF> ret;
    auto avoidPath = mNavFnPlanner->createPlan(createPose(before), createPose(after));

    for (auto& pose : avoidPath.poses) {
        ret.push_back(PointF::fromPose(pose));
    }
    return ret;
}

std::vector<PointF> RobotGlobalPlanner::buildPath(const PointF& start,
                                                  const PointF& goal,
                                                  const PointF& pos)
{
    std::vector<PointF> path{pos};
    PointF prevPoint = pos;
    PointF interpolatedPos = interpolatePoint(start, goal, pos);
    double xInc, yInc;
    bool insideObstacle = false;
    auto loops = getPathIncrements(interpolatedPos, goal, xInc, yInc);
    for (size_t i = 0; i < loops; i++) {
        auto currPoint = PointF{interpolatedPos.x + xInc * i, interpolatedPos.y + yInc * i};
        uint32_t mx, my;
        if (mCostmap->worldToMap(currPoint.x, currPoint.y, mx, my)) {
            uint32_t cost = mCostmap->getCost(mx, my);
            if (cost > nav2_costmap_2d::FREE_SPACE) {
                if (!insideObstacle) {
                    insideObstacle = true;
                    mBeforeObstacle = prevPoint;
                }
            }
            else {
                if (!insideObstacle) {
                    path.push_back(currPoint);
                }
                else {
                    insideObstacle = false;
                    auto avoidPoints = avoidObstacle(mBeforeObstacle, currPoint);
                    path.insert(path.end(), avoidPoints.begin(), avoidPoints.end());
                }
            }
            prevPoint = currPoint;
        }
    }

    path.push_back(insideObstacle ? mBeforeObstacle : goal);

    return path;
}

PointF RobotGlobalPlanner::interpolatePoint(const PointF& start,
                                            const PointF& goal,
                                            const PointF& pos)
{
    double distanceStartPos = std::hypot(pos.x - start.x, pos.y - start.y);
    double totalDistance = std::hypot(goal.x - start.x, goal.y - start.y);

    double interpolationParam = distanceStartPos / totalDistance;

    double interpolatedX = start.x + interpolationParam * (goal.x - start.x);
    double interpolatedY = start.y + interpolationParam * (goal.y - start.y);

    return PointF{interpolatedX, interpolatedY};
}
PoseStamped RobotGlobalPlanner::createPose(const PointF& point)
{
    PoseStamped pose;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 0;
    pose.header.stamp = mNode->now();
    pose.header.frame_id = mGlobalFrame;

    return pose;
}

Path RobotGlobalPlanner::createPlan(const PoseStamped& start, const PoseStamped& goal)
{
    if (mGoalPoint != PointF::fromPose(goal)) {
        RCLCPP_INFO(mNode->get_logger(), "New Goal received");
        mGoalPoint = PointF::fromPose(goal);
        mStartPoint = PointF::fromPose(start);
    }

    auto positionPoint = PointF::fromPose(start);

    Path path;
    path.poses.clear();
    path.header.frame_id = mGlobalFrame;
    path.header.stamp = mNode->now();
    auto points = buildPath(mStartPoint, mGoalPoint, positionPoint);
    for (size_t i = 0; i < points.size(); i++) {
        path.poses.push_back(createPose(points[i]));
    }

    return path;
}

void RobotGlobalPlanner::onNavigationStatus(const action_msgs::msg::GoalStatusArray& msg)
{
    auto statusList = msg.status_list;
    auto statusCount = statusList.size();

    if (statusCount > 0) {
        auto status = *(--statusList.end());
        switch (status.status) {
        case rclcpp_action::GoalStatus::STATUS_ABORTED:
        case rclcpp_action::GoalStatus::STATUS_CANCELED:
        case rclcpp_action::GoalStatus::STATUS_SUCCEEDED:
            mStartPoint = PointF{0, 0, false};
            mGoalPoint = PointF{0, 0, false};
            RCLCPP_INFO(mNode->get_logger(), "Navigation Completes");
            break;
        }
    }
}
}  // namespace robot_global_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(robot_global_planner::RobotGlobalPlanner, nav2_core::GlobalPlanner)
