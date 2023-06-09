#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>


class HandPlugin : public gazebo::ModelPlugin {
  public:
    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);

    void OnUpdate();

  private:
    gazebo::physics::ModelPtr mModel;
    gazebo::physics::JointPtr mJoint;
    double mAngle;
    double mSpeed;
    bool mLeftDirection;
    gazebo::event::ConnectionPtr mUpdateConnection;
};