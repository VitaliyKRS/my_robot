#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "tracked_robot_msgs/msg/hand_position.hpp"

class HandPlugin : public gazebo::ModelPlugin {
  public:
    HandPlugin();
    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);

    void OnUpdate();
    void onMineDetected(const std_msgs::msg::Bool::SharedPtr msg);

  private:
    gazebo::physics::ModelPtr mModel;
    gazebo::physics::JointPtr mJoint;
    double mAngle;
    double mSpeed;
    bool mLeftDirection;
    bool mHandPaused;
    double mHandLength;
    double mCurrentAngle;
    gazebo::event::ConnectionPtr mUpdateConnection;
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mMineSubscription;
    rclcpp::Publisher<tracked_robot_msgs::msg::HandPosition>::SharedPtr mHandPositionPublisher;
    rclcpp::Node::SharedPtr mRosNode;
    std::unique_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster;
};