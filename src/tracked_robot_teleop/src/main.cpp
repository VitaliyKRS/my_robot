#include <rclcpp/rclcpp.hpp>
#include "tracked_robot_teleop/JoystickHandler.hpp"
#include "tracked_robot_teleop/TeleopHandler.hpp"

int main(int argc, char* argv[])
{
    // Initialize the application
    rclcpp::init(argc, argv);

    // Setup the executor
    std::shared_ptr<rclcpp::Executor> executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    // Create the joystick node
    auto teleop   = std::make_shared<TeleopHandler>("teleop");
    auto joystick = std::make_shared<JoystickHandler>("joystick");

    // Run the node(s)
    executor->add_node(teleop);
    executor->add_node(joystick);
    executor->spin();

    // Exit
    rclcpp::shutdown();
    return 0;
}