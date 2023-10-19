#include "Field2CoverNode.h"
#include "ros/conversor.h"
constexpr float RESOLUTION = 0.1f;
Fields2CoverNode::Fields2CoverNode()
    : Node("Fields2CoverNode")
{
    declare_parameter("op_width", 0.6);
    declare_parameter("turn_radius", 0.2);
    declare_parameter("headland_width", 0.5);
    declare_parameter("swath_angle", 0.01);
    declare_parameter("automatic_angle", false);
    declare_parameter("sg_objective", 0);
    declare_parameter("route_type", 0);
    declare_parameter("turn_type", 0);

    declare_parameter("robot_width", 0.6);
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

    mFieldPlan.header.stamp = now();
    mFieldPlan.header.frame_id = mWorldFrame;
    mStartPosReached = false;
    mPrevSwath.x = 0;
    mPrevSwath.y = 0;
    mSwathFound = false;
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

    mGetFieldPlanService = create_service<tracked_robot_msgs::srv::FieldPlan>(
        "get_field_plan", std::bind(&Fields2CoverNode::get_plan_callback, this,
                                    std::placeholders::_1, std::placeholders::_2));

    mSubscription = create_subscription<action_msgs::msg::GoalStatusArray>(
        "navigate_to_pose/_action/status", 10,
        std::bind(&Fields2CoverNode::onNavigationStatus, this, std::placeholders::_1));
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
        mFieldPlan.poses.push_back(s);
        ros_p.x = s.pose.position.x;
        ros_p.y = s.pose.position.y;
        mSwathMarket.points.push_back(ros_p);
    }

    sendNavGoal(mFieldPlan.poses[0]);
}

std::vector<geometry_msgs::msg::PoseStamped> Fields2CoverNode::convertGpsPosesToMapPoses(
    F2CPath& path)
{
    RCLCPP_INFO(this->get_logger(), "Converting GPS path points to %s Frame..",
                mWorldFrame.c_str());
    auto gps_path = f2c::Transform::transformPathWithFieldRef(path, mFields[0], "EPSG:4326");
    std::vector<geometry_msgs::msg::PoseStamped> poses_in_map_frame_vector;
    int waypoint_index = 0;
    int turnInd = 0;
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
            auto map_point = response->map_point;
            if (curr_geopose.type == f2c::types::PathSectionType::SWATH) {
                if (!mSwathFound) {
                    mPrevSwath = map_point;
                    mSwathFound = true;
                }
                else {
                    mSwathFound = false;
                    double xInc, yInc;
                    auto loops = getPathIncrements(mPrevSwath, map_point, xInc, yInc);
                    for (size_t i = 0; i < loops; i++) {
                        geometry_msgs::msg::Point point;
                        point.x = mPrevSwath.x + xInc * i;
                        point.y = mPrevSwath.y + yInc * i;
                        poses_in_map_frame_vector.push_back(createPose(point));
                    }
                }
            }
            else {
                turnInd++;
                if (turnInd == 5) {
                    poses_in_map_frame_vector.push_back(createPose(response->map_point));
                    turnInd = 0;
                }
            }
        }
        waypoint_index++;
    }
    RCLCPP_INFO(this->get_logger(), "Converted all %i GPS path to %s frame",
                static_cast<int>(poses_in_map_frame_vector.size()), mWorldFrame.c_str());
    return poses_in_map_frame_vector;
}

void Fields2CoverNode::onNavigationStatus(const action_msgs::msg::GoalStatusArray& msg)
{
    auto statusList = msg.status_list;
    auto statusCount = statusList.size();

    if (statusCount > 0) {
        auto status = *(--statusList.end());
        switch (status.status) {
        case rclcpp_action::GoalStatus::STATUS_ABORTED:
            RCLCPP_INFO(this->get_logger(), "Goal aborted, try again...");
            if (!mStartPosReached) {
                sendNavGoal(mFieldPlan.poses.front());
            }
            else {
                sendNavGoal(mFieldPlan.poses.back());
            }
            break;
        case rclcpp_action::GoalStatus::STATUS_SUCCEEDED:
            if (!mStartPosReached) {
                mStartPosReached = true;
                sendNavGoal(mFieldPlan.poses.back());
            }
            else {
                mStartPosReached = false;
            }

            RCLCPP_INFO(this->get_logger(), "Navigation Completes");
            break;
        }
    }
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

void Fields2CoverNode::get_plan_callback(
    const std::shared_ptr<tracked_robot_msgs::srv::FieldPlan::Request> request,
    const std::shared_ptr<tracked_robot_msgs::srv::FieldPlan::Response> response)
{
    if (mStartPosReached) {
        RCLCPP_INFO(this->get_logger(), "Start pose %f - %f", request->start.pose.position.x,
                    request->start.pose.position.y);

        bool remove{false};
        int i = 0;
        const int max_poses_to_remove = 50;  // Adjust this number as needed

        for (; i <= std::min(static_cast<int>(mFieldPlan.poses.size()), max_poses_to_remove); ++i) {
            if (std::hypot(mFieldPlan.poses[i].pose.position.x - request->start.pose.position.x,
                           mFieldPlan.poses[i].pose.position.y - request->start.pose.position.y) <=
                request->tolerance) {
                RCLCPP_INFO(this->get_logger(), "Find close pose %f - %f",
                            mFieldPlan.poses[i].pose.position.x,
                            mFieldPlan.poses[i].pose.position.y);
                remove = true;
                break;
            }
        }

        if (remove) {
            RCLCPP_INFO(this->get_logger(), "Remove  %d poses from path", i);
            auto it = mFieldPlan.poses.begin() + i;
            mFieldPlan.poses.erase(mFieldPlan.poses.begin(), it);
        }

        response->plan = mFieldPlan;
    }
    else {
        response->plan = {};
    }
}

void Fields2CoverNode::sendNavGoal(const geometry_msgs::msg::PoseStamped& goal)
{
    if (!this->mNavigateToPoseClient->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
    }

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose = goal;
    auto send_goal_options =
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&Fields2CoverNode::goal_response_callback, this, std::placeholders::_1);

    mNavigateToPoseClient->async_send_goal(goal_msg, send_goal_options);
}

size_t Fields2CoverNode::getPathIncrements(const geometry_msgs::msg::Point& start,
                                           const geometry_msgs::msg::Point& goal,
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

geometry_msgs::msg::PoseStamped Fields2CoverNode::createPose(const geometry_msgs::msg::Point& point)
{
    geometry_msgs::msg::Quaternion q;
    q.x = 0;
    q.y = 0;
    q.z = 0;
    q.w = 1.0;
    geometry_msgs::msg::PoseStamped curr_pose_map_frame;
    curr_pose_map_frame.header.frame_id = mWorldFrame;
    curr_pose_map_frame.header.stamp = this->now();
    curr_pose_map_frame.pose.position = point;
    curr_pose_map_frame.pose.orientation = q;

    return curr_pose_map_frame;
}