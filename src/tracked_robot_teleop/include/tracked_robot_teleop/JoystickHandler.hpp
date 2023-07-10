#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <vector>


class JoystickHandler
    : public rclcpp::Node
{
public:
    JoystickHandler(const std::string & name);
    ~JoystickHandler();

private:
    void update();

private:
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
    rclcpp::TimerBase::SharedPtr update_timer_;
    std::vector<int> buttons_;
    std::vector<double> axes_;
    int device_handle_;
    double deadzone_;
    double scale_;
    double unscaled_deadzone_;

};

