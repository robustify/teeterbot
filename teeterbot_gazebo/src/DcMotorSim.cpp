#include "DcMotorSim.h"

namespace teeterbot_gazebo
{

DcMotorSim::DcMotorSim(ros::NodeHandle n,
                       const gazebo::physics::JointPtr &joint,
                       const gazebo::physics::LinkPtr &link)
{
  joint_ = joint;
  props_.inductance = 0.0025;
  props_.resistance = 0.5;
  props_.torque_constant = 0.35;
  props_.max_current = 30.0;
  current_ = 0.0;
}

void DcMotorSim::step(double ts, double voltage_in, double load_torque)
{
  current_ += ts / props_.inductance * (voltage_in - props_.resistance * current_ - props_.torque_constant * joint_->GetVelocity(0));
  if (current_ > props_.max_current){
    current_ = props_.max_current;
  }else if (current_ < -props_.max_current){
    current_ = -props_.max_current;
  }
  joint_->SetForce(0, current_ * props_.torque_constant);
}

}