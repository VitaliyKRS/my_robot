#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include <geometry_msgs/msg/point.hpp>

namespace nav2_path_follower
{

class PurePursuitPathFollower : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
{
public:
  using Parent = nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController;
  PurePursuitPathFollower() = default;

  ~PurePursuitPathFollower() override = default;

  /**
   * @brief Configure controller state machine
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup controller state machine
   */
  void cleanup() override;

  /**
   * @brief Activate controller state machine
   */
  void activate() override;

  /**
   * @brief Deactivate controller state machine
   */
  void deactivate() override;

  /**
   * @brief Compute the best command given the current pose and velocity, with possible debug information
   *
   * Same as above computeVelocityCommands, but with debug results.
   * If the results pointer is not null, additional information about the twists
   * evaluated will be in results after the call.
   *
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param goal_checker   Ptr to the goal checker for this task in case useful in computing commands
   * @return          Best command
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  /**
   * @brief nav2_core setPlan - Sets the global plan
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief Limits the maximum linear speed of the robot.
   * @param speed_limit expressed in absolute value (in m/s)
   * or in percentage from maximum robot speed.
   * @param percentage Setting speed limit in percentage if true
   * or in absolute values in false case.
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
  void onMinePositionReceived(const geometry_msgs::msg::Point::SharedPtr minePos);
  geometry_msgs::msg::TwistStamped computeBackWardVelocityCommand(const geometry_msgs::msg::PoseStamped & pose);
  bool shouldDriveBackward(const geometry_msgs::msg::PoseStamped & pose);

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr mMinePositionSubscription;
  double mBackDistance;
  double mBackwardSpeed;
  double mMaxAngleDifference;
  std::string mMinePosTopic;
  geometry_msgs::msg::Point mMinePosition;

    // Dynamic parameters handler
  std::mutex mDynParamMutex;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr mDynParamHandler;
};

}  // namespace nav2_path_follower
