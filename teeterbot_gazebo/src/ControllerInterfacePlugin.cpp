#include "ControllerInterfacePlugin.hpp"
#include <gazebo_ros/conversions/builtin_interfaces.hpp>

namespace gazebo {

  ControllerInterfacePlugin::ControllerInterfacePlugin() {
    fallen_over_ = false;
    fallen_over_stamp_ = 0;
    left_cmd_ = 0;
    right_cmd_ = 0;

    is_nudging_ = false;
    nudge_stamp_ = -1;
    nudge_duration_ = 0;
    nudge_force_.Set(0, 0, 0);
    nudge_offset_.Set(0, 0, 0);
    data_10_count_ = 0;
  }

  ControllerInterfacePlugin::~ControllerInterfacePlugin() {
    left_motor_.reset();
    right_motor_.reset();
  }

  void ControllerInterfacePlugin::Reset() {}

  void ControllerInterfacePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
    ros_node_ = gazebo_ros::Node::Get(sdf);
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(ros_node_);

    // Gazebo setup
    model_ = model;
    left_wheel_link_ = model->GetLink("left_wheel");
    right_wheel_link_ = model->GetLink("right_wheel");
    body_link_ = model->GetLink("base_link");
    left_wheel_joint_ = model->GetJoint("left_wheel_joint");
    right_wheel_joint_ = model->GetJoint("right_wheel_joint");
    left_motor_ = std::make_shared<teeterbot_gazebo::DcMotorSim>(left_wheel_joint_);
    right_motor_ = std::make_shared<teeterbot_gazebo::DcMotorSim>(right_wheel_joint_);

    bool gazebo_ready_ = true;
    if (!body_link_) {
      RCLCPP_ERROR(ros_node_->get_logger(), "Body link not found!");
      gazebo_ready_ = false;
    }
    if (!left_wheel_link_) {
      RCLCPP_ERROR(ros_node_->get_logger(), "Left wheel link not found!");
      gazebo_ready_ = false;
    }
    if (!right_wheel_link_) {
      RCLCPP_ERROR(ros_node_->get_logger(), "Right wheel link not found!");
      gazebo_ready_ = false;
    }

    if (!gazebo_ready_) {
      return;
    }

    update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&ControllerInterfacePlugin::OnUpdate, this, _1));

    // Load SDF parameters
    pub_ground_truth_ = sdf->Get<bool>("pub_ground_truth", false).first;
    if (auto_reset_orientation_ = sdf->Get<bool>("auto_reset_orientation", false).first) {
      RCLCPP_INFO(ros_node_->get_logger(), "Resetting orientation automatically");
    } else {
      RCLCPP_INFO(ros_node_->get_logger(), "Not resetting orientation automatically");
    }

    auto_reset_delay_ = sdf->Get<double>("auto_reset_delay", 2.0).first;
    if (auto_reset_orientation_) {
      RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Will wait " << auto_reset_delay_ <<" seconds before resetting orientation");
    }

    double nudge_z_offset;
    if (nudge_z_offset = sdf->Get<double>("body_length", 0.5).first) {
      RCLCPP_INFO(ros_node_->get_logger(), "Nudge force will be applied at the top of the robot");
      nudge_offset_.Z(nudge_z_offset);
    } else {
      RCLCPP_WARN(ros_node_->get_logger(), "No body length specified! Nudge force applied 0.5 meters above base");
      nudge_offset_.Z(0.5);
    }
    nudge_offset_.X(0);
    nudge_offset_.Y(0);

    voltage_mode_ = false;
    torque_mode_ = false;
    speed_mode_ = false;
    int num_modes_selected = 0;
    if (voltage_mode_ = sdf->Get<bool>("voltage_mode", false).first) {
      num_modes_selected++;
    }
    if (torque_mode_ = sdf->Get<bool>("torque_mode", false).first) {
      num_modes_selected++;
    }
    if (speed_mode_ = sdf->Get<bool>("speed_mode", false).first) {
      num_modes_selected++;
    }

    if (num_modes_selected == 0) {
      RCLCPP_WARN(ros_node_->get_logger(), "No control mode specified; defaulting to voltage mode");
      voltage_mode_ = true;
    } else if (num_modes_selected > 1) {
      RCLCPP_ERROR(ros_node_->get_logger(), "Multiple control modes specified; defaulting to voltage mode");
      voltage_mode_ = true;
      torque_mode_ = false;
      speed_mode_ = false;
    }

    if (voltage_mode_) {
      RCLCPP_INFO(ros_node_->get_logger(), "Using voltage mode");
    }
    if (torque_mode_) {
      RCLCPP_INFO(ros_node_->get_logger(), "Using torque mode");
      left_control_ = std::make_shared<teeterbot_gazebo::MotorController>(teeterbot_gazebo::TORQUE_PARAMS);
      right_control_ = std::make_shared<teeterbot_gazebo::MotorController>(teeterbot_gazebo::TORQUE_PARAMS);
    }
    if (speed_mode_) {
      RCLCPP_INFO(ros_node_->get_logger(), "Using speed mode");
      left_control_ = std::make_shared<teeterbot_gazebo::MotorController>(teeterbot_gazebo::SPEED_PARAMS);
      right_control_ = std::make_shared<teeterbot_gazebo::MotorController>(teeterbot_gazebo::SPEED_PARAMS);
    }

    pub_left_encoder = ros_node_->create_publisher<example_interfaces::msg::Float64>("left_wheel_speed", 1);
    pub_right_encoder = ros_node_->create_publisher<example_interfaces::msg::Float64>("right_wheel_speed", 1);
    pub_left_current = ros_node_->create_publisher<example_interfaces::msg::Float64>("left_current", 1);
    pub_right_current = ros_node_->create_publisher<example_interfaces::msg::Float64>("right_current", 1);

    auto latch_qos = rclcpp::QoS(1).transient_local();
    pub_fallen_over = ros_node_->create_publisher<example_interfaces::msg::Bool>("fallen_over", latch_qos);

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
    sub_left_cmd = ros_node_->create_subscription<example_interfaces::msg::Float64>(
                    left_cmd_topic, 1, std::bind(&ControllerInterfacePlugin::recv_left_motor_cmd, this, std::placeholders::_1));
    sub_right_cmd= ros_node_->create_subscription<example_interfaces::msg::Float64>(
                    right_cmd_topic, 1, std::bind(&ControllerInterfacePlugin::recv_right_motor_cmd, this, std::placeholders::_1));

    nudge_srv = ros_node_->create_service<teeterbot_gazebo::srv::NudgeTeeterbot>("nudge", std::bind(&ControllerInterfacePlugin::nudge_cb, this, std::placeholders::_1, std::placeholders::_2));
    left_cmd_ = 0.0;
    right_cmd_ = 0.0;

    example_interfaces::msg::Bool init_fallen_over;
    init_fallen_over.data = false;
    pub_fallen_over->publish(init_fallen_over);
    fallen_over_ = false;
  }

  void ControllerInterfacePlugin::recv_left_motor_cmd(const example_interfaces::msg::Float64::ConstSharedPtr msg) {
    left_cmd_ = msg->data;
  }

  void ControllerInterfacePlugin::recv_right_motor_cmd(const example_interfaces::msg::Float64::ConstSharedPtr msg) {
    right_cmd_ = msg->data;
  }

  void ControllerInterfacePlugin::data_10_cb() {
    example_interfaces::msg::Float64 left_encoder_msg;
    example_interfaces::msg::Float64 right_encoder_msg;
    example_interfaces::msg::Float64 left_current_msg;
    example_interfaces::msg::Float64 right_current_msg;

    left_encoder_msg.data = left_wheel_joint_->GetVelocity(0);
    right_encoder_msg.data = right_wheel_joint_->GetVelocity(0);
    left_current_msg.data = left_motor_->get_current();
    right_current_msg.data = right_motor_->get_current();

    pub_left_encoder->publish(left_encoder_msg);
    pub_right_encoder->publish(right_encoder_msg);
    pub_left_current->publish(left_current_msg);
    pub_right_current->publish(right_current_msg);
  }

  void ControllerInterfacePlugin::OnUpdate(const common::UpdateInfo &info) {
    double time_step = (info.simTime - last_update_time).Double();
    last_update_time = info.simTime;

    ignition::math::Pose3d pose = body_link_->WorldPose();

    double roll, pitch, yaw;
    get_euler(roll, pitch, yaw);

    if (data_10_count_++ >= 10) {
      data_10_cb();
      data_10_count_ = 0;
    }

    // Detect if fallen over
    if (!fallen_over_ && std::abs(pitch) > M_PI_4) {
      example_interfaces::msg::Bool fallen_over_msg;
      fallen_over_msg.data = true;
      pub_fallen_over->publish(fallen_over_msg);
      fallen_over_stamp_ = info.simTime.Double();
    } else if (fallen_over_ && std::abs(pitch) <= M_PI_4) {
      example_interfaces::msg::Bool fallen_over_msg;
      fallen_over_msg.data = false;
      pub_fallen_over->publish(fallen_over_msg);
    }
    fallen_over_ = std::abs(pitch) > M_PI_4;

    // Reset orientation if enabled and conditions are met
    if (auto_reset_orientation_ && fallen_over_ && (info.simTime.Double() - fallen_over_stamp_) > auto_reset_delay_){
      model_->SetWorldPose(ignition::math::Pose3d(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z(), 0.0, 0.0, yaw));
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
        left_feedback = left_motor_->get_current() * left_motor_->get_torque_constant();
        right_feedback = right_motor_->get_current() * right_motor_->get_torque_constant();
      } else {
        left_feedback = left_wheel_joint_->GetVelocity(0);
        right_feedback = right_wheel_joint_->GetVelocity(0);
      }
      left_voltage = left_control_->update(time_step, left_cmd_, left_feedback);
      right_voltage = right_control_->update(time_step, right_cmd_, right_feedback);
    }

    // Apply voltage to motors
    left_motor_->step(time_step, left_voltage);
    right_motor_->step(time_step, right_voltage);

    // Publish footprint frame
    auto current_ros_time = gazebo_ros::Convert<builtin_interfaces::msg::Time>(info.simTime);
    geometry_msgs::msg::TransformStamped footprint_link_transform;
    footprint_link_transform.child_frame_id = "base_link";
    footprint_link_transform.header.frame_id = "base_footprint";
    footprint_link_transform.header.stamp = current_ros_time;
    footprint_link_transform.transform.translation.x = 0.0;
    footprint_link_transform.transform.translation.y = 0.0;
    footprint_link_transform.transform.translation.z = pose.Pos().Z();
    ignition::math::Quaternion<double> rollpitch;
    rollpitch.Euler(roll, pitch, 0.0);
    // footprint_link_transform.setRotation(tf::Quaternion(rollpitch.X(), rollpitch.Y(), rollpitch.Z(), rollpitch.W()));
    footprint_link_transform.transform.rotation.w = rollpitch.W();
    footprint_link_transform.transform.rotation.x = rollpitch.X();
    footprint_link_transform.transform.rotation.y = rollpitch.Y();
    footprint_link_transform.transform.rotation.z = rollpitch.Z();
    tf_broadcaster->sendTransform(footprint_link_transform);

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
      body_link_->AddLinkForce(ignition::math::Vector3d(), ignition::math::Vector3d());
    }

    // Publish ground truth transform from world to base_footprint, if enabled
    if (pub_ground_truth_) {
      geometry_msgs::msg::TransformStamped ground_truth_transform;
      ground_truth_transform.child_frame_id = "base_footprint";
      ground_truth_transform.header.frame_id = "world";
      ground_truth_transform.header.stamp = current_ros_time;
      ground_truth_transform.transform.translation.x = pose.Pos().X();
      ground_truth_transform.transform.translation.y = pose.Pos().Y();
      ground_truth_transform.transform.translation.z = 0.0;
      ground_truth_transform.transform.rotation.w = cos(0.5 * yaw);
      ground_truth_transform.transform.rotation.z = sin(0.5 * yaw);
      tf_broadcaster->sendTransform(ground_truth_transform);
    }
  }

  void ControllerInterfacePlugin::nudge_cb(std::shared_ptr<teeterbot_gazebo::srv::NudgeTeeterbot::Request> req, std::shared_ptr<teeterbot_gazebo::srv::NudgeTeeterbot::Response> res) {
    nudge_duration_ = (req->duration < 1e-3 ? 0.1 : req->duration);
    nudge_force_.Set(req->force, 0, 0);
    is_nudging_ = true;
  }

  void ControllerInterfacePlugin::get_euler(double &roll, double &pitch, double &yaw) {
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
  }

}
