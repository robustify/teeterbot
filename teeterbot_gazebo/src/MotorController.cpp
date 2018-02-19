#include "MotorController.h"

namespace teeterbot_gazebo
{

MotorController::MotorController(ros::NodeHandle n, ros::NodeHandle pn, const std::string& name) :
  srv_(ros::NodeHandle(pn, name)),
  int_val_(0.0),
  target_(0.0),
  last_error_(0.0)
{
  srv_.setCallback(boost::bind(&MotorController::reconfigCb, this, _1, _2));
}

double MotorController::update(double ts, double cmd, double feedback)
{
  // Reset integrator when target is near zero
  if (fabs(target_) < 1e-2) {
    int_val_ = 0;
  }

  // Impose ramp limit on target speed
  if ((cmd - target_) > (0.5 * ts * cfg_.ramp_limit)) {
    target_ += ts * cfg_.ramp_limit;
  } else if ((cmd - target_) < (-0.5 * ts * cfg_.ramp_limit)) {
    target_ -= ts * cfg_.ramp_limit;
  } else {
    target_ = cmd;
  }

  // Compute voltage command
  double voltage_output;
  double error = target_ - feedback;
  double derivative = (error - last_error_) / ts;
  voltage_output = cfg_.kp * error + cfg_.ki * int_val_ + cfg_.kd * derivative;

  // Saturate voltage command at 0 depending on target speed
  if (target_ > 0 && voltage_output < 0) {
    voltage_output = 0;
  } else if (target_ < 0 && voltage_output > 0) {
    voltage_output = 0;
  } else {
    int_val_ += ts * error;
  }

  return voltage_output;
}

void MotorController::reconfigCb(MotorControlConfig &config, uint32_t level)
{
  cfg_ = config;
  int_val_ = 0;
}

}