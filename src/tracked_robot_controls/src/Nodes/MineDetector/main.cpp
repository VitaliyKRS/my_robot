#include "Nodes/MineDetector/MineDetector.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MineDetector>());
    rclcpp::shutdown();
    return 0;
}