#include "MineLayer/MineLayer.h"
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
    
PLUGINLIB_EXPORT_CLASS(mine_costmap_layer::MineLayer, nav2_costmap_2d::Layer)
   
using nav2_costmap_2d::LETHAL_OBSTACLE;

namespace mine_costmap_layer {

MineLayer::MineLayer() {}

void MineLayer::onInitialize()
{
    auto node = node_.lock();

    if (!node) {
    throw std::runtime_error{"Failed to lock node"};
    }

    mMinePositionSubscription = node->create_subscription<geometry_msgs::msg::Point>(
        "/mine_position", 10, std::bind(
          &MineLayer::onMinePositionReceived, this,
          std::placeholders::_1));
}

void MineLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y, double *max_x, double *max_y)
{
    for (const auto& mineObstacle : mMineObstacles)
    {
        *min_x = std::min(*min_x, mineObstacle.x - 2);
        *min_y = std::min(*min_y, mineObstacle.y - 2);
        *max_x = std::max(*max_x, mineObstacle.x + 2);
        *max_y = std::max(*max_y, mineObstacle.y + 2);
    }
}

void MineLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
{
    if (mMineObstacles.empty())
        return;

    nav2_costmap_2d::Costmap2D *costmap = layered_costmap_->getCostmap();
   int radius = 2; 

    int map_size_x = costmap->getSizeInCellsX();
    int map_size_y = costmap->getSizeInCellsY();

    for (const auto& mineObstacle : mMineObstacles)
    {
        int cell_x, cell_y;
        costmap->worldToMapNoBounds(mineObstacle.x, mineObstacle.y, cell_x, cell_y);

        for (int dx = -radius; dx <= radius; ++dx)
        {
            for (int dy = -radius; dy <= radius; ++dy)
            {
                int neighbor_x = cell_x + dx;
                int neighbor_y = cell_y + dy;

                if (neighbor_x >= 0 && neighbor_x < map_size_x &&
                    neighbor_y >= 0 && neighbor_y < map_size_y)
                {
                    if (LETHAL_OBSTACLE > costmap->getCost(neighbor_x, neighbor_y))
                    {
                        costmap->setCost(neighbor_x, neighbor_y, LETHAL_OBSTACLE);
                    }
                }
            }
        }
    }
}

bool MineLayer::isClearable()
{
    return true;
}
void MineLayer::reset()
{
}

void MineLayer::onMinePositionReceived(const geometry_msgs::msg::Point::SharedPtr minePos)
{
    geometry_msgs::msg::Point mineObstacle;
    if(std::none_of(mMineObstacles.begin(), mMineObstacles.end(), [minePos](auto& point) {
        return minePos->x == point.x && minePos->y == point.y;
    })) {
            RCLCPP_INFO(rclcpp::get_logger("MineLayer"), "Mine received %f, %f!", minePos->x, minePos->y);
    mineObstacle.x = minePos->x;
    mineObstacle.y = minePos->y;

    mMineObstacles.push_back(mineObstacle);
    }

}
}