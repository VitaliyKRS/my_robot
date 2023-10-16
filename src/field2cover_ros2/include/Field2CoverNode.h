
#include "Fields2CoverConfig.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_util/service_client.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_localization/srv/from_ll.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <fields2cover.h>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
class Fields2CoverNode : public rclcpp::Node {
public:
    Fields2CoverNode();
    void initialize();
    void publishTopics();
    void calculate();

private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mFieldSwathPublisher;
    std::unique_ptr<nav2_util::ServiceClient<robot_localization::srv::FromLL>> mFromLLToMapClient;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr mNavigateToPoseClient;
    rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr mGetFieldPlanService;
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr mSubscription;

    std::vector<geometry_msgs::msg::PoseStamped> convertGpsPosesToMapPoses(F2CPath& path);

    void onNavigationStatus(const action_msgs::msg::GoalStatusArray& msg);
    void goal_response_callback(
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle);
    void get_plan_callback(const std::shared_ptr<rmw_request_id_t> request_header,
                           const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
                           const std::shared_ptr<nav_msgs::srv::GetPlan::Response> response);

    void sendNavGoal(const geometry_msgs::msg::PoseStamped& goal);

    size_t getPathIncrements(const geometry_msgs::msg::Point& start,
                             const geometry_msgs::msg::Point& goal,
                             double& xIncrement,
                             double& yIncrement);
    geometry_msgs::msg::PoseStamped createPose(const geometry_msgs::msg::Point& point);
    Fields2CoverConfig mConfig;
    F2CFields mFields;
    F2CCell mNoHeadlands;
    F2CRobot mRobot{0.8, 0.8};
    F2COptim mOptim;

    std::string mWorldFrame;
    std::string mFieldFile;
    geometry_msgs::msg::Point mPrevSwath;
    bool mSwathFound;
    bool mStartPosReached;
    visualization_msgs::msg::Marker mSwathMarket;
    nav_msgs::msg::Path mFieldPlan;
};
