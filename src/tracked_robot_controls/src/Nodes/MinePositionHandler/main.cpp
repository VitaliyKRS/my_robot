#include "Nodes/MinePositionHandler/MinePositionHandler.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinePositionHandler>());
    rclcpp::shutdown();
    return 0;
}