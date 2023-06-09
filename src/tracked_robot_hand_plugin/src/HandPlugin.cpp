#include "HandPlugin.h"

#include <rclcpp/rclcpp.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void HandPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
      RCLCPP_INFO(rclcpp::get_logger("tracked_robot_hand_plugin"), "Tracked Robot Hand Plugin loaded successfully!");
      mModel = model;

      mJoint = model->GetJoint("hand_joint");
      mAngle = 60.0 * M_PI / 180.0;
      mSpeed = 3;
      mLeftDirection = true;
      mUpdateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&HandPlugin::OnUpdate, this));
}

void HandPlugin::OnUpdate()
{
    double timeStep = 0.01; // Adjust the time step as needed

        // Check the current motion direction

        // Get the current angle of the joint
        double currentAngle = mJoint->Position(0);

        // Calculate the remaining angle to reach the target angle
        double remainingAngle = mAngle - currentAngle;

        // Check if the remaining angle is within a small tolerance
        double angleTolerance = 0.01; // Adjust the tolerance as needed
        if (std::abs(remainingAngle) < angleTolerance)
        {
          // Stop the motion when the target angle is reached
          mJoint->SetVelocity(0, 0.0);
          RCLCPP_INFO(rclcpp::get_logger("tracked_robot_hand_plugin"), "Reached target angle!");
          if(mLeftDirection) {
                mAngle = -mAngle;
                mLeftDirection = false;
          } else {
                mAngle = +mAngle;
                mLeftDirection = true;
          }

          return;
        }

        // Calculate the direction of rotation based on the remaining angle
        double rotationDirection = (remainingAngle > 0) ? 1.0 : -1.0;

        // Calculate the desired joint velocity based on the rotation speed and direction
        double jointVelocity = mSpeed * rotationDirection;

        // Set the joint velocity to achieve continuous motion
        mJoint->SetVelocity(0, jointVelocity);

        // Wait for the time step
        gazebo::common::Time::MSleep(static_cast<unsigned int>(timeStep * 1000.0));
      
}

GZ_REGISTER_MODEL_PLUGIN(HandPlugin)