#include "MineLayer/MineLayer.h"
#include <pluginlib/class_list_macros.hpp>
    
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
        *min_y = std::min(*min_y, mineObstacle.y - 2 );
        *max_x = std::max(*max_x, mineObstacle.x + 2);
        *max_y = std::max(*max_y, mineObstacle.y + 2);
    }
}

void MineLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
{
    if (mMineObstacles.empty())
        return;

    double mine_radius = 2.0;  // Change this value to the desired radius

     for (const auto& mineObstacle : mMineObstacles)
    {
        unsigned int cell_x, cell_y;
        if (master_grid.worldToMapEnforceBounds(mineObstacle.x, mineObstacle.y, cell_x, cell_y))
        {
            // Update cells within the mine_radius
            for (int dx = -static_cast<int>(mine_radius); dx <= static_cast<int>(mine_radius); ++dx)
            {
                for (int dy = -static_cast<int>(mine_radius); dy <= static_cast<int>(mine_radius); ++dy)
                {
                    int update_cell_x = cell_x + dx;
                    int update_cell_y = cell_y + dy;

                    if (master_grid.isCellInBounds(update_cell_x, update_cell_y))
                    {
                        master_grid.setCost(update_cell_x, update_cell_y, LETHAL_OBSTACLE);
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
    RCLCPP_INFO(rclcpp::get_logger("MineLayer"), "Mine received!");
    mineObstacle.x = minePos->x;
    mineObstacle.y = minePos->y;

    mMineObstacles.push_back(mineObstacle);
}
}