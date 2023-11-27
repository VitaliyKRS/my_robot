
#include "tracked_robot_teleop/TeleopHandler.hpp"
#include <memory>

TeleopHandler::TeleopHandler(const rclcpp::NodeOptions& options)
    : rclcpp::Node("teleop_twist_joy_node", options)
{
    // Setup the parameters
    declare_parameter<std::string>("joy_topic", "joy");
    declare_parameter<std::string>("twist_topic", "/diffbot_base_controller/cmd_vel_unstamped");
    //declare_parameter<std::string>("twist_topic", "/cmd_vel");

    declare_axis_parameter("move_forward", move_forward_config_);
    declare_axis_parameter("move_reverse", move_reverse_config_);
    declare_axis_parameter("move_left", move_left_config_);
    declare_axis_parameter("move_right", move_right_config_);
    declare_axis_parameter("turn_left", turn_left_config_);
    declare_axis_parameter("turn_right", turn_right_config_);

    // Setup the velocity command publisher
    std::string twist_topic = get_parameter("twist_topic").as_string();
    twist_publisher_ = create_publisher<geometry_msgs::msg::Twist>(twist_topic, 10);

    // Setup the joystick message subscriber
    std::string joy_topic = get_parameter("joy_topic").as_string();
    joy_subscriber_ = create_subscription<sensor_msgs::msg::Joy>(
        joy_topic, 10, std::bind(&TeleopHandler::on_joy_message, this, std::placeholders::_1));
}

void TeleopHandler::on_joy_message(std::unique_ptr<sensor_msgs::msg::Joy> msg)
{
    geometry_msgs::msg::Twist::UniquePtr twist_message(new geometry_msgs::msg::Twist());
    auto lx = get_axis_value(msg, move_forward_config_) + get_axis_value(msg, move_reverse_config_);

//    if (lx > 0.33 || lx < -0.33) {
        twist_message->linear.x = lx;
        twist_message->angular.z =
            get_axis_value(msg, turn_left_config_) + get_axis_value(msg, turn_right_config_);
//    } else {
//        twist_message->linear.x = 0.0;
//        twist_message->angular.z = 0.0;
//    }

    //    get_axis_value(msg, move_forward_config_) + get_axis_value(msg, move_reverse_config_);
    twist_message->linear.y = 0.0;
    twist_message->linear.z = 0.0;
    twist_message->angular.x = 0.0;
    twist_message->angular.y = 0.0;
    twist_publisher_->publish(std::move(twist_message));
}

void TeleopHandler::declare_axis_parameter(const std::string& param_name, AxisConfig& config)
{
    std::string param_prefix = param_name + ".";
    if (param_name == "move_forward" || param_name == "move_reverse") {
        declare_parameter<int>(param_prefix + "axis", 1);
    }
    else {
        declare_parameter<int>(param_prefix + "axis", 0);
    }
    declare_parameter<double>(param_prefix + "scale", 0.1);
    declare_parameter<double>(param_prefix + "offset", 0.0);
    declare_parameter<double>(param_prefix + "deadzone", 0.0);

    // Load the parameter values
    config.axis = get_parameter(param_prefix + "axis").as_int();
    config.scale = get_parameter(param_prefix + "scale").as_double();
    config.offset = get_parameter(param_prefix + "offset").as_double();
    config.deadzone = get_parameter(param_prefix + "deadzone").as_double();
}

double TeleopHandler::get_axis_value(std::unique_ptr<sensor_msgs::msg::Joy>& msg,
                                     AxisConfig& config)
{
    if (msg->axes.size() >= static_cast<size_t>(config.axis)) {
        double value = (msg->axes[config.axis] - config.offset) * config.scale;
        if (value > config.deadzone) {
            value -= config.deadzone;
        }
        else if (value < -config.deadzone) {
            value += config.deadzone;
        }
        else {
            value = 0.0;
        }
        return value;
    }
    return 0.0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(TeleopHandler)
