#include "MotorSpeedController.h"

namespace teeterbot_gazebo
{

MotorSpeedController::MotorSpeedController(ros::NodeHandle n, ros::NodeHandle pn) :
  srv_(pn),
  int_val_(0.0),
  speed_cmd_(0.0),
  speed_target_(0.0),
  last_error_(0.0),
  wheel_speed_(0.0)
{
  srv_.setCallback(boost::bind(&MotorSpeedController::reconfigCb, this, _1, _2));

  sub_speed_cmd_ = n.subscribe("speed_cmd", 1, &MotorSpeedController::recvCmdCb, this);
  sub_encoder_measurement_ = n.subscribe("wheel_speed", 1, &MotorSpeedController::recvEncoderCb, this);
  pub_voltage_ = n.advertise<std_msgs::Float64>("voltage_output", 1);

  control_timer_ = n.createTimer(ros::Duration(SAMPLE_TIME), &MotorSpeedController::controlTimerCb, this);
}

void MotorSpeedController::controlTimerCb(const ros::TimerEvent &event)
{
  // Stop wheel in case of command timeout
  if ((event.current_real - cmd_stamp_).toSec() > cfg_.timeout_duration && cfg_.timeout_duration > 0) {
    speed_cmd_ = 0.0;
  }

  // Reset integrator when target is near zero
  if (fabs(speed_target_) < 1e-2) {
    int_val_ = 0;
  }

  // Impose ramp limit on target speed
  if ((speed_cmd_ - speed_target_) > (0.5 * SAMPLE_TIME * cfg_.ramp_limit)) {
    speed_target_ += SAMPLE_TIME * cfg_.ramp_limit;
  } else if ((speed_cmd_ - speed_target_) < (-0.5 * SAMPLE_TIME * cfg_.ramp_limit)) {
    speed_target_ -= SAMPLE_TIME * cfg_.ramp_limit;
  } else {
    speed_target_ = speed_cmd_;
  }

  // Compute voltage command
  std_msgs::Float64 cmd_msg;
  double error = speed_target_ - wheel_speed_;
  double derivative = (error - last_error_) / SAMPLE_TIME;
  cmd_msg.data = cfg_.kp * error + cfg_.ki * int_val_ + cfg_.kd * derivative;

  // Saturate voltage command at 0 depending on target speed
  if (speed_target_ > 0 && cmd_msg.data < 0) {
    cmd_msg.data = 0;
  } else if (speed_target_ < 0 && cmd_msg.data > 0) {
    cmd_msg.data = 0;
  } else {
    int_val_ += SAMPLE_TIME * error;
  }

  pub_voltage_.publish(cmd_msg);
}

void MotorSpeedController::recvEncoderCb(const std_msgs::Float64ConstPtr &msg)
{
  wheel_speed_ = msg->data;
}

void MotorSpeedController::recvCmdCb(const std_msgs::Float64ConstPtr &msg)
{
  speed_cmd_ = msg->data;
  cmd_stamp_ = ros::Time::now();
}

void MotorSpeedController::reconfigCb(MotorSpeedControlConfig &config, uint32_t level)
{
  cfg_ = config;
  int_val_ = 0;
}

}