#include "Field2CoverNode.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Fields2CoverNode>();
    node->initialize();
    while (rclcpp::ok()) {
        node->publishTopics();
        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}