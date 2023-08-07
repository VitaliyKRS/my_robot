#include <nav2_costmap_2d/layer.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include <rclcpp/rclcpp.hpp>

namespace mine_costmap_layer {
class MineLayer : public nav2_costmap_2d::Layer
{
public:
    MineLayer();

    void onInitialize() override;
    void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y) override;
    void updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;
    bool isClearable() override;
    void reset() override;
private:
    void onMinePositionReceived(const geometry_msgs::msg::Point::SharedPtr minePos);
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr mMinePositionSubscription;

    std::vector<geometry_msgs::msg::Point> mMineObstacles;
};
}