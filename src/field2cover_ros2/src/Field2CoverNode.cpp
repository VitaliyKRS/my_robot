#include "Field2CoverNode.h"
#include "ros/conversor.h"

Fields2CoverNode::Fields2CoverNode()
    : Node("Fields2CoverNode")
{
    declare_parameter("op_width", 0.4);
    declare_parameter("turn_radius", 0.0);
    declare_parameter("headland_width", 0.5);
    declare_parameter("swath_angle", 0.01);
    declare_parameter("automatic_angle", false);
    declare_parameter("sg_objective", 0);
    declare_parameter("route_type", 0);
    declare_parameter("turn_type", 0);

    declare_parameter("robot_width", 0.4);
    declare_parameter("robot_max_vel", 2.0);
    declare_parameter("world_frame", "map");
    declare_parameter("data_file", "");

    mConfig.mOpWidth = get_parameter("op_width").as_double();
    mConfig.mTurnRadius = get_parameter("turn_radius").as_double();
    mConfig.mHeadlandWidth = get_parameter("headland_width").as_double();
    mConfig.mSwathAngle = get_parameter("swath_angle").as_double();
    mConfig.mAutomaticAngle = get_parameter("automatic_angle").as_bool();
    mConfig.mSgQbjective = get_parameter("sg_objective").as_int();
    mConfig.mRouteType = get_parameter("route_type").as_int();
    mConfig.mTurnType = get_parameter("turn_type").as_int();

    mConfig.mRobotWidth = get_parameter("robot_width").as_double();
    mConfig.mRobotMaxVel = get_parameter("robot_max_vel").as_double();
    mWorldFrame = get_parameter("world_frame").as_string();

    mFieldFile = get_parameter("data_file").as_string();
}

void Fields2CoverNode::initialize()
{
    mFieldSwathPublisher =
        this->create_publisher<visualization_msgs::msg::Marker>("/field/swaths", 1);
    mNavigateToPoseClient = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this->get_node_base_interface(), this->get_node_graph_interface(),
        this->get_node_logging_interface(), this->get_node_waitables_interface(),
        "navigate_to_pose"  // Name of the action server, change as needed
    );

    mFromLLToMapClient =
        std::make_unique<nav2_util::ServiceClient<robot_localization::srv::FromLL>>(
            "/fromLL", shared_from_this());
    f2c::Parser::importJson(mFieldFile, mFields);

    mFields[0].setEPSGCoordSystem(4326);
    f2c::Transform::transform(mFields[0], "EPSG:28992");

    RCLCPP_INFO(this->get_logger(), "Created");
    mRobot.cruise_speed = mConfig.mRobotMaxVel;
    mRobot.setMinRadius(mConfig.mTurnRadius);
    mOptim.headland_width = 3.0 * mRobot.op_width;

    F2CCells border = mFields[0].field;
    f2c::hg::ConstHL hl_gen_;

    F2CCells no_headland = hl_gen_.generateHeadlands(border, 3.0 * mRobot.op_width);
    mNoHeadlands = no_headland.getGeometry(0);
    calculate();
}

void Fields2CoverNode::publishTopics() { mFieldSwathPublisher->publish(mSwathMarket); }

void Fields2CoverNode::calculate()
{
    F2CSwaths swaths;
    f2c::sg::BruteForce swath_gen_;
    if (mConfig.mAutomaticAngle) {
        switch (mConfig.mSgQbjective) {
        case 0: {
            f2c::obj::SwathLength obj;
            swaths = swath_gen_.generateBestSwaths(obj, mRobot.op_width, mNoHeadlands);
            break;
        }
        case 1: {
            f2c::obj::NSwath obj;
            swaths = swath_gen_.generateBestSwaths(obj, mRobot.op_width, mNoHeadlands);
            break;
        }
        case 2: {
            f2c::obj::FieldCoverage obj;
            swaths = swath_gen_.generateBestSwaths(obj, mRobot.op_width, mNoHeadlands);
            break;
        }
        }
    }
    else {
        swaths = swath_gen_.generateSwaths(mOptim.best_angle, mRobot.op_width, mNoHeadlands);
    }

    F2CSwaths route;
    switch (mConfig.mRouteType) {
    case 0: {
        f2c::rp::BoustrophedonOrder swath_sorter;
        route = swath_sorter.genSortedSwaths(swaths);
        break;
    }
    case 1: {
        f2c::rp::SnakeOrder swath_sorter;
        route = swath_sorter.genSortedSwaths(swaths);
        break;
    }
    case 2: {
        f2c::rp::SpiralOrder swath_sorter(6);
        route = swath_sorter.genSortedSwaths(swaths);
        break;
    }
    case 3: {
        f2c::rp::SpiralOrder swath_sorter(4);
        route = swath_sorter.genSortedSwaths(swaths);
        break;
    }
    }

    F2CPath path;
    f2c::pp::PathPlanning path_planner;

    switch (mConfig.mTurnType) {
    case 0: {
        f2c::pp::DubinsCurves turn;
        path = path_planner.searchBestPath(mRobot, route, turn);
        break;
    }
    case 1: {
        f2c::pp::DubinsCurvesCC turn;
        path = path_planner.searchBestPath(mRobot, route, turn);
        break;
    }
    case 2: {
        f2c::pp::ReedsSheppCurves turn;
        path = path_planner.searchBestPath(mRobot, route, turn);
        break;
    }
    case 3: {
        f2c::pp::ReedsSheppCurvesHC turn;
        path = path_planner.searchBestPath(mRobot, route, turn);
        break;
    }
    }

    // auto goal_msg = nav2_msgs::action::FollowGPSWaypoints::Goal();

    mSwathMarket.header.frame_id = mWorldFrame;
    mSwathMarket.header.stamp = now();
    mSwathMarket.action = visualization_msgs::msg::Marker::ADD;
    mSwathMarket.pose.orientation.w = 1.0;
    mSwathMarket.type = visualization_msgs::msg::Marker::LINE_STRIP;
    mSwathMarket.scale.x = 0.1;
    mSwathMarket.scale.y = 0.1;
    mSwathMarket.scale.z = 0.1;
    mSwathMarket.color.b = 1.0;
    mSwathMarket.color.a = 1.0;
    geometry_msgs::msg::Point ros_p;

    auto new_path = convertGpsPosesToMapPoses(path);
    for (auto&& s : new_path) {
        ros_p.x = s.pose.position.x;
        ros_p.y = s.pose.position.y;
        mSwathMarket.points.push_back(ros_p);
    }

    if (!this->mNavigateToPoseClient->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
    }

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose = new_path[0];
    auto send_goal_options =
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&Fields2CoverNode::goal_response_callback, this, std::placeholders::_1);

    mNavigateToPoseClient->async_send_goal(goal_msg, send_goal_options);
}

std::vector<geometry_msgs::msg::PoseStamped> Fields2CoverNode::convertGpsPosesToMapPoses(
    F2CPath& path)
{
    RCLCPP_INFO(this->get_logger(), "Converting GPS path points to %s Frame..",
                mWorldFrame.c_str());
    auto gps_path = f2c::Transform::transformPathWithFieldRef(path, mFields[0], "EPSG:4326");
    std::vector<geometry_msgs::msg::PoseStamped> poses_in_map_frame_vector;
    int waypoint_index = 0;
    RCLCPP_INFO(this->get_logger(), " %f, %f ", gps_path.states[0].point.getX(),
                gps_path.states[0].point.getY());
    for (auto&& curr_geopose : gps_path.states) {
        auto request = std::make_shared<robot_localization::srv::FromLL::Request>();
        auto response = std::make_shared<robot_localization::srv::FromLL::Response>();
        request->ll_point.latitude = curr_geopose.point.getY();
        request->ll_point.longitude = curr_geopose.point.getX();
        request->ll_point.altitude = 0.0;

        mFromLLToMapClient->wait_for_service((std::chrono::seconds(1)));
        if (!mFromLLToMapClient->invoke(request, response)) {
            RCLCPP_ERROR(
                this->get_logger(),
                "fromLL service of robot_localization could not convert %i th GPS waypoint to"
                "%s frame, going to skip this point!"
                "Make sure you have run navsat_transform_node of robot_localization",
                waypoint_index, mWorldFrame.c_str());
            continue;
        }
        else {
            geometry_msgs::msg::Quaternion q;
            q.x = 0;
            q.y = 0;
            q.z = 0;
            q.w = 1.0;
            geometry_msgs::msg::PoseStamped curr_pose_map_frame;
            curr_pose_map_frame.header.frame_id = mWorldFrame;
            curr_pose_map_frame.header.stamp = this->now();
            curr_pose_map_frame.pose.position = response->map_point;
            curr_pose_map_frame.pose.orientation = q;
            poses_in_map_frame_vector.push_back(curr_pose_map_frame);
        }
        waypoint_index++;
    }
    RCLCPP_INFO(this->get_logger(), "Converted all %i GPS path to %s frame",
                static_cast<int>(poses_in_map_frame_vector.size()), mWorldFrame.c_str());
    return poses_in_map_frame_vector;
}

void Fields2CoverNode::goal_response_callback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    }
    else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}
