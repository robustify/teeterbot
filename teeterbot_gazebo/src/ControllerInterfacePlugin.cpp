#include "ControllerInterfacePlugin.h"

namespace gazebo
{

ControllerInterfacePlugin::ControllerInterfacePlugin()
{
}

ControllerInterfacePlugin::~ControllerInterfacePlugin()
{
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

  // ROS setup
  n_ = new ros::NodeHandle(model->GetName());
  left_motor_ = new teeterbot_gazebo::DcMotorSim(ros::NodeHandle(*n_, "left_motor"), left_wheel_joint_, left_wheel_link_);
  right_motor_ = new teeterbot_gazebo::DcMotorSim(ros::NodeHandle(*n_, "right_motor"), right_wheel_joint_, right_wheel_link_);

  pub_left_encoder_ = n_->advertise<std_msgs::Float64>("left_wheel_speed", 1);
  pub_right_encoder_ = n_->advertise<std_msgs::Float64>("right_wheel_speed", 1);
  pub_left_current_ = n_->advertise<std_msgs::Float64>("left_current", 1);
  pub_right_current_ = n_->advertise<std_msgs::Float64>("right_current", 1);
  pub_fallen_over_ = n_->advertise<std_msgs::Bool>("fallen_over", 1, true);

  sub_left_voltage_ = n_->subscribe<std_msgs::Float64>("left_motor_voltage", 1, boost::bind(&ControllerInterfacePlugin::recvMotorVoltage, this, _1, 0));
  sub_right_voltage_ = n_->subscribe<std_msgs::Float64>("right_motor_voltage", 1, boost::bind(&ControllerInterfacePlugin::recvMotorVoltage, this, _1, 1));

  data_100Hz_timer_ = n_->createTimer(ros::Duration(0.01), &ControllerInterfacePlugin::data100Cb, this);

  left_voltage_ = 0.0;
  right_voltage_ = 0.0;

  std_msgs::Bool init_fallen_over;
  init_fallen_over.data = false;
  pub_fallen_over_.publish(init_fallen_over);
  fallen_over_ = false;
}

void ControllerInterfacePlugin::recvMotorVoltage(const std_msgs::Float64ConstPtr &msg, int side)
{
  if (side == 0) { // left
    left_voltage_ = msg->data;
  } else { // right
    right_voltage_ = msg->data;
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
  math::Pose pose = body_link_->GetWorldPose();
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
  if (fallen_over_ && (info.simTime.Double() - fallen_over_stamp_) > auto_reset_delay_){
    model_->SetWorldPose(math::Pose(pose.pos, math::Quaternion(0.0, 0.0, yaw)));
  }

  // Apply voltage to motors
  left_motor_->step(0.001, left_voltage_);
  right_motor_->step(0.001, right_voltage_);

  // Publish footprint frame
  tf::StampedTransform footprint_link_transform;
  footprint_link_transform.child_frame_id_ = "base_link";
  footprint_link_transform.frame_id_ = "base_footprint";
  footprint_link_transform.stamp_.fromSec(info.simTime.Double());
  footprint_link_transform.setOrigin(tf::Vector3(0, 0, pose.pos.z));
  ignition::math::Quaternion<double> rollpitch;
  rollpitch.Euler(roll, pitch, 0.0);
  footprint_link_transform.setRotation(tf::Quaternion(rollpitch.X(), rollpitch.Y(), rollpitch.Z(), rollpitch.W()));
  broadcaster_.sendTransform(footprint_link_transform);

  // Publish ground truth transform from world to base_footprint, if enabled
  if (pub_ground_truth_) {
    tf::StampedTransform ground_truth_transform;
    ground_truth_transform.child_frame_id_ = "base_footprint";
    ground_truth_transform.frame_id_ = "world";
    ground_truth_transform.stamp_.fromSec(info.simTime.Double());
    ground_truth_transform.setOrigin(tf::Vector3(pose.pos.x, pose.pos.y, 0.0));
    ground_truth_transform.setRotation(tf::Quaternion(0.0, 0.0, sin(0.5 * yaw), cos(0.5 * yaw)));
    broadcaster_.sendTransform(ground_truth_transform);
  }
}

void ControllerInterfacePlugin::getEuler(double &roll, double &pitch, double &yaw)
{
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
}

}