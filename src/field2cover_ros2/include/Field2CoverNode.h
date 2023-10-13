
#include "Fields2CoverConfig.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_util/service_client.hpp"
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
    std::vector<geometry_msgs::msg::PoseStamped> convertGpsPosesToMapPoses(F2CPath& path);
    void goal_response_callback(
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle);
    Fields2CoverConfig mConfig;
    F2CFields mFields;
    F2CCell mNoHeadlands;
    F2CRobot mRobot{0.4, 0.4};
    F2COptim mOptim;

    std::string mWorldFrame;
    std::string mFieldFile;

    geometry_msgs::msg::PolygonStamped mBorderPoligon;
    geometry_msgs::msg::PolygonStamped mNoHeadlansPoligon;
    visualization_msgs::msg::Marker mSwathMarket;
};
