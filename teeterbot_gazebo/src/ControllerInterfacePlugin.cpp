#include "ControllerInterfacePlugin.h"

namespace gazebo
{

ControllerInterfacePlugin::ControllerInterfacePlugin()
{
  fallen_over_ = false;
  fallen_over_stamp_ = 0;
  left_cmd_ = 0;
  right_cmd_ = 0;

  is_nudging_ = false;
  nudge_stamp_ = -1;
  nudge_duration_ = 0;
#if GAZEBO_MAJOR_VERSION >= 9
  nudge_force_.Set(0, 0, 0);
  nudge_offset_.Set(0, 0, 0);
#else
  nudge_force_.x = 0;
  nudge_force_.y = 0;
  nudge_force_.x = 0;

  nudge_offset_.x = 0;
  nudge_offset_.y = 0;
  nudge_offset_.z = 0;
#endif
}

ControllerInterfacePlugin::~ControllerInterfacePlugin()
{
  left_motor_.reset();
  right_motor_.reset();
  left_control_.reset();
  right_control_.reset();
  n_->shutdown();
  delete n_;
}

void ControllerInterfacePlugin::Reset()
{
}

void ControllerInterfacePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Gazebo setup
  model_ = model;
  left_wheel_joint_ = model->GetJoint("left_wheel");
  right_wheel_joint_ = model->GetJoint("right_wheel");
  left_wheel_link_ = model->GetLink("left_wheel");
  right_wheel_link_ = model->GetLink("right_wheel");
  body_link_ = model->GetLink("base_link");

  bool gazebo_ready_ = true;
  if (!body_link_) {
    ROS_ERROR("Body link not found!");
    gazebo_ready_ = false;
  }
  if (!left_wheel_link_) {
    ROS_ERROR("Left wheel link not found!");
    gazebo_ready_ = false;
  }
  if (!right_wheel_link_) {
    ROS_ERROR("Right wheel link not found!");
    gazebo_ready_ = false;
  }

  if (!gazebo_ready_) {
    return;
  }

  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&ControllerInterfacePlugin::OnUpdate, this, _1));

  // Load SDF parameters
  if (sdf->HasElement("pubGroundTruth")) {
    sdf->GetElement("pubGroundTruth")->GetValue()->Get(pub_ground_truth_);
  } else {
    pub_ground_truth_ = false;
  }

  // Auto-reset parameters
  if (sdf->HasElement("autoResetOrientation")) {
    sdf->GetElement("autoResetOrientation")->GetValue()->Get(auto_reset_orientation_);
    if (auto_reset_orientation_){
      ROS_INFO("Resetting orientation automatically");
    }else{
      ROS_INFO("Not resetting orientation automatically");
    }
  } else {
    auto_reset_orientation_ = false;
  }

  if (sdf->HasElement("autoResetDelay") && auto_reset_orientation_) {
    sdf->GetElement("autoResetDelay")->GetValue()->Get(auto_reset_delay_);
    ROS_INFO("Will wait %f seconds before resetting orientation", auto_reset_delay_);
  } else {
    auto_reset_delay_ = 2.0;
  }

  // Load nudge force offset
#if GAZEBO_MAJOR_VERSION >= 9
  if (sdf->HasElement("bodyLength")) {
    sdf->GetElement("bodyLength")->GetValue()->Get(nudge_offset_.Z());
    ROS_INFO("Will apply nudge force at top of robot");
  } else {
    ROS_WARN("No body length specified! Nudge force applied 0.5 meters above base");
    nudge_offset_.Z(0.5);
  }
  nudge_offset_.X(0);
  nudge_offset_.Y(0);
#else
  if (sdf->HasElement("bodyLength")) {
    sdf->GetElement("bodyLength")->GetValue()->Get(nudge_offset_.z);
    ROS_INFO("Will apply nudge force at top of robot");
  } else {
    ROS_WARN("No body length specified! Nudge force applied 0.5 meters above base");
    nudge_offset_.z = 0.5;
  }
  nudge_offset_.x = 0;
  nudge_offset_.y = 0;
#endif

  // Load control mode
  voltage_mode_ = false;
  torque_mode_ = false;
  speed_mode_ = false;
  int num_modes_selected = 0;
  if (sdf->HasElement("voltageMode")) {
    sdf->GetElement("voltageMode")->GetValue()->Get(voltage_mode_);
    if (voltage_mode_) {
      num_modes_selected++;
    }
  }
  if (sdf->HasElement("torqueMode")) {
    sdf->GetElement("torqueMode")->GetValue()->Get(torque_mode_);
    if (torque_mode_) {
      num_modes_selected++;
    }
  }
  if (sdf->HasElement("speedMode")) {
    sdf->GetElement("speedMode")->GetValue()->Get(speed_mode_);
    if (speed_mode_) {
      num_modes_selected++;
    }
  }

  if (num_modes_selected == 0) {
    ROS_WARN("No control mode specified; defaulting to voltage mode");
    voltage_mode_ = true;
  } else if (num_modes_selected > 1) {
    ROS_ERROR("Multiple control modes specified; defaulting to voltage mode");
    voltage_mode_ = true;
    torque_mode_ = false;
    speed_mode_ = false;
  }

  // ROS setup
  n_ = new ros::NodeHandle(model->GetName());
  left_motor_.reset(new teeterbot_gazebo::DcMotorSim(ros::NodeHandle(*n_, "left_motor"), left_wheel_joint_, left_wheel_link_));
  right_motor_.reset(new teeterbot_gazebo::DcMotorSim(ros::NodeHandle(*n_, "right_motor"), right_wheel_joint_, right_wheel_link_));

  if (voltage_mode_) {
    ROS_INFO("Using voltage mode");
  }
  if (torque_mode_) {
    ROS_INFO("Using torque mode");
    left_control_.reset(new teeterbot_gazebo::MotorController(*n_, ros::NodeHandle("~"), "left_torque_control"));
    right_control_.reset(new teeterbot_gazebo::MotorController(*n_, ros::NodeHandle("~"), "right_torque_control"));
  }
  if (speed_mode_) {
    ROS_INFO("Using speed mode");
    left_control_.reset(new teeterbot_gazebo::MotorController(*n_, ros::NodeHandle("~"), "left_speed_control"));
    right_control_.reset(new teeterbot_gazebo::MotorController(*n_, ros::NodeHandle("~"), "right_speed_control"));
  }

  pub_left_encoder_ = n_->advertise<std_msgs::Float64>("left_wheel_speed", 1);
  pub_right_encoder_ = n_->advertise<std_msgs::Float64>("right_wheel_speed", 1);
  pub_left_current_ = n_->advertise<std_msgs::Float64>("left_current", 1);
  pub_right_current_ = n_->advertise<std_msgs::Float64>("right_current", 1);
  pub_fallen_over_ = n_->advertise<std_msgs::Bool>("fallen_over", 1, true);

  std::string left_cmd_topic;
  std::string right_cmd_topic;
  if (voltage_mode_) {
    left_cmd_topic = "left_motor_voltage";
    right_cmd_topic = "right_motor_voltage";
  } else if (torque_mode_) {
    left_cmd_topic = "left_torque_cmd";
    right_cmd_topic = "right_torque_cmd";
  } else {
    left_cmd_topic = "left_speed_cmd";
    right_cmd_topic = "right_speed_cmd";
  }
  sub_left_cmd_ = n_->subscribe<std_msgs::Float64>(left_cmd_topic, 1, boost::bind(&ControllerInterfacePlugin::recvMotorCmd, this, _1, 0));
  sub_right_cmd_ = n_->subscribe<std_msgs::Float64>(right_cmd_topic, 1, boost::bind(&ControllerInterfacePlugin::recvMotorCmd, this, _1, 1));

  nudge_srv_ = n_->advertiseService("nudge", &ControllerInterfacePlugin::nudgeCb, this);

  data_100Hz_timer_ = n_->createTimer(ros::Duration(0.01), &ControllerInterfacePlugin::data100Cb, this);

  left_cmd_ = 0.0;
  right_cmd_ = 0.0;

  std_msgs::Bool init_fallen_over;
  init_fallen_over.data = false;
  pub_fallen_over_.publish(init_fallen_over);
  fallen_over_ = false;
}

void ControllerInterfacePlugin::recvMotorCmd(const std_msgs::Float64ConstPtr &msg, int side)
{
  if (side == 0) { // left
    left_cmd_ = msg->data;
  } else { // right
    right_cmd_ = msg->data;
  }
}

void ControllerInterfacePlugin::data100Cb(const ros::TimerEvent &event)
{
  std_msgs::Float64 left_encoder_msg;
  std_msgs::Float64 right_encoder_msg;
  std_msgs::Float64 left_current_msg;
  std_msgs::Float64 right_current_msg;

  left_encoder_msg.data = left_wheel_joint_->GetVelocity(0);
  right_encoder_msg.data = right_wheel_joint_->GetVelocity(0);
  left_current_msg.data = left_motor_->current_;
  right_current_msg.data = right_motor_->current_;

  pub_left_encoder_.publish(left_encoder_msg);
  pub_right_encoder_.publish(right_encoder_msg);
  pub_left_current_.publish(left_current_msg);
  pub_right_current_.publish(right_current_msg);
}

void ControllerInterfacePlugin::OnUpdate(const common::UpdateInfo &info)
{
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3d pose = body_link_->WorldPose();
#else
  math::Pose pose = body_link_->GetWorldPose();
#endif

  double roll, pitch, yaw;
  getEuler(roll, pitch, yaw);

  // Detect if fallen over
  if (!fallen_over_ && fabs(pitch) > M_PI_4) {
    std_msgs::Bool fallen_over_msg;
    fallen_over_msg.data = true;
    pub_fallen_over_.publish(fallen_over_msg);
    fallen_over_stamp_ = info.simTime.Double();
  } else if (fallen_over_ && fabs(pitch) <= M_PI_4) {
    std_msgs::Bool fallen_over_msg;
    fallen_over_msg.data = false;
    pub_fallen_over_.publish(fallen_over_msg);
  }
  fallen_over_ = fabs(pitch) > M_PI_4;

  // Reset orientation if enabled and conditions are met
  if (auto_reset_orientation_ && fallen_over_ && (info.simTime.Double() - fallen_over_stamp_) > auto_reset_delay_){
#if GAZEBO_MAJOR_VERSION >= 9
    model_->SetWorldPose(ignition::math::Pose3d(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z(), 0.0, 0.0, yaw));
#else
    model_->SetWorldPose(math::Pose(pose.pos, math::Quaternion(0.0, 0.0, yaw)));
#endif
  }

  double left_voltage;
  double right_voltage;
  double left_feedback;
  double right_feedback;

  if (voltage_mode_) {
    left_voltage = left_cmd_;
    right_voltage = right_cmd_;
  } else {
    if (torque_mode_) {
      left_feedback = left_motor_->current_ * left_motor_->props_.torque_constant;
      right_feedback = right_motor_->current_ * right_motor_->props_.torque_constant;
    } else {
      left_feedback = left_wheel_joint_->GetVelocity(0);
      right_feedback = right_wheel_joint_->GetVelocity(0);
    }
    left_voltage = left_control_->update(0.001, left_cmd_, left_feedback);
    right_voltage = right_control_->update(0.001, right_cmd_, right_feedback);
  }

  // Apply voltage to motors
  left_motor_->step(0.001, left_voltage);
  right_motor_->step(0.001, right_voltage);

  // Publish footprint frame
  tf::StampedTransform footprint_link_transform;
  footprint_link_transform.child_frame_id_ = "base_link";
  footprint_link_transform.frame_id_ = "base_footprint";
  footprint_link_transform.stamp_.fromSec(info.simTime.Double());
#if GAZEBO_MAJOR_VERSION >= 9
  footprint_link_transform.setOrigin(tf::Vector3(0, 0, pose.Pos().Z()));
#else
  footprint_link_transform.setOrigin(tf::Vector3(0, 0, pose.pos.z));
#endif
  ignition::math::Quaternion<double> rollpitch;
  rollpitch.Euler(roll, pitch, 0.0);
  footprint_link_transform.setRotation(tf::Quaternion(rollpitch.X(), rollpitch.Y(), rollpitch.Z(), rollpitch.W()));
  broadcaster_.sendTransform(footprint_link_transform);

  // Apply nudge force
  if (is_nudging_) {
    if (nudge_stamp_ < 0) {
      nudge_stamp_ = info.simTime.Double();
    }

    if ((info.simTime.Double() - nudge_stamp_) > nudge_duration_) {
      is_nudging_ = false;
      nudge_stamp_ = -1;
    }

    body_link_->AddLinkForce(nudge_force_, nudge_offset_);
  } else {
#if GAZEBO_MAJOR_VERSION >= 9
    body_link_->AddLinkForce(ignition::math::Vector3d(), ignition::math::Vector3d());
#else
    math::Vector3 zero;
    body_link_->AddLinkForce(zero, zero);
#endif
  }

  // Publish ground truth transform from world to base_footprint, if enabled
  if (pub_ground_truth_) {
    tf::StampedTransform ground_truth_transform;
    ground_truth_transform.child_frame_id_ = "base_footprint";
    ground_truth_transform.frame_id_ = "world";
    ground_truth_transform.stamp_.fromSec(info.simTime.Double());
#if GAZEBO_MAJOR_VERSION >= 9
    ground_truth_transform.setOrigin(tf::Vector3(pose.Pos().X(), pose.Pos().Y(), 0.0));
#else
    ground_truth_transform.setOrigin(tf::Vector3(pose.pos.x, pose.pos.y, 0.0));
#endif
    ground_truth_transform.setRotation(tf::Quaternion(0.0, 0.0, sin(0.5 * yaw), cos(0.5 * yaw)));
    broadcaster_.sendTransform(ground_truth_transform);
  }
}

bool ControllerInterfacePlugin::nudgeCb(teeterbot_gazebo::NudgeTeeterbotRequest &req, teeterbot_gazebo::NudgeTeeterbotResponse &res)
{
  nudge_duration_ = (req.duration == 0 ? 0.1 : req.duration);
#if GAZEBO_MAJOR_VERSION >= 9
  nudge_force_.Set(req.force, 0, 0);
#else
  nudge_force_.x = req.force;
  nudge_force_.y = 0;
  nudge_force_.z = 0;
#endif
  is_nudging_ = true;
  return true;
}

void ControllerInterfacePlugin::getEuler(double &roll, double &pitch, double &yaw)
{
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3d pose = body_link_->WorldPose();
  ignition::math::Quaternion<double> ignition_orientation(pose.Rot().W(), pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z());
  roll = ignition_orientation.Roll();
  pitch = ignition_orientation.Pitch();
  yaw = ignition_orientation.Yaw();

  // Adjust RPY angles if body z axis is pointing down in global frame
  ignition::math::Matrix3d rot(pose.Rot());
  if (rot(2, 2) < 0) {
    // Modify RPY angles
    roll += M_PI;
    yaw += M_PI;
    pitch = M_PI - pitch;

    // Wrap new angles into range (-pi, pi)
    if (roll > M_PI) {
      roll -= 2 * M_PI;
    } else if (roll < -M_PI) {
      roll += 2 * M_PI;
    }
    if (pitch > M_PI) {
      pitch -= 2 * M_PI;
    } else if (pitch < -M_PI) {
      pitch += 2 * M_PI;
    }
    if (yaw > M_PI) {
      yaw -= 2 * M_PI;
    } else if (yaw < -M_PI) {
      yaw += 2 * M_PI;
    }
  }
#else
  math::Pose pose = body_link_->GetWorldPose();
  ignition::math::Quaternion<double> ignition_orientation(pose.rot.w, pose.rot.x, pose.rot.y, pose.rot.z);
  roll = ignition_orientation.Roll();
  pitch = ignition_orientation.Pitch();
  yaw = ignition_orientation.Yaw();

  // Adjust RPY angles if body z axis is pointing down in global frame
  if (pose.rot.GetAsMatrix3()[2][2] < 0) {
    // Modify RPY angles
    roll += M_PI;
    yaw += M_PI;
    pitch = M_PI - pitch;

    // Wrap new angles into range (-pi, pi)
    if (roll > M_PI) {
      roll -= 2 * M_PI;
    } else if (roll < -M_PI) {
      roll += 2 * M_PI;
    }
    if (pitch > M_PI) {
      pitch -= 2 * M_PI;
    } else if (pitch < -M_PI) {
      pitch += 2 * M_PI;
    }
    if (yaw > M_PI) {
      yaw -= 2 * M_PI;
    } else if (yaw < -M_PI) {
      yaw += 2 * M_PI;
    }
  }
#endif
}

}
