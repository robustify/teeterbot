#pragma once

#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/float64.hpp>
#include <example_interfaces/msg/bool.hpp>
#include <teeterbot_gazebo/srv/nudge_teeterbot.hpp>

#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include "MotorController.hpp"
#include "DcMotorSim.hpp"

namespace gazebo {

  class ControllerInterfacePlugin : public ModelPlugin {
    public:
      ControllerInterfacePlugin();
      virtual ~ControllerInterfacePlugin();

    protected:
      virtual void Load (physics::ModelPtr model, sdf::ElementPtr sdf);
      virtual void Reset();

    private:
      void OnUpdate(const common::UpdateInfo& info);

      void nudge_cb(std::shared_ptr<teeterbot_gazebo::srv::NudgeTeeterbot::Request> req, std::shared_ptr<teeterbot_gazebo::srv::NudgeTeeterbot::Response> res);
      void data_10_cb();
      void recv_left_motor_cmd(const example_interfaces::msg::Float64::ConstSharedPtr msg);
      void recv_right_motor_cmd(const example_interfaces::msg::Float64::ConstSharedPtr msg);
      void get_euler(double& roll, double& pitch, double& yaw);

      // ROS
      gazebo_ros::Node::SharedPtr ros_node_;
      rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr pub_left_encoder;
      rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr pub_right_encoder;
      rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr pub_left_current;
      rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr pub_right_current;
      rclcpp::Publisher<example_interfaces::msg::Bool>::SharedPtr pub_fallen_over;
      rclcpp::Subscription<example_interfaces::msg::Float64>::SharedPtr sub_left_cmd;
      rclcpp::Subscription<example_interfaces::msg::Float64>::SharedPtr sub_right_cmd;
      rclcpp::Service<teeterbot_gazebo::srv::NudgeTeeterbot>::SharedPtr nudge_srv;
      std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
      int data_10_count_;

      // Gazebo
      event::ConnectionPtr update_connection_;
      physics::ModelPtr model_;
      physics::JointPtr left_wheel_joint_;
      physics::JointPtr right_wheel_joint_;
      physics::LinkPtr left_wheel_link_;
      physics::LinkPtr right_wheel_link_;
      physics::LinkPtr body_link_;
      common::Time last_update_time;

      // DC motor simulation instances
      teeterbot_gazebo::DcMotorSim::SharedPtr left_motor_;
      teeterbot_gazebo::DcMotorSim::SharedPtr right_motor_;

      // Controller instances
      teeterbot_gazebo::MotorController::SharedPtr left_control_;
      teeterbot_gazebo::MotorController::SharedPtr right_control_;

      // Status properties
      bool fallen_over_;
      double fallen_over_stamp_;
      double left_cmd_;
      double right_cmd_;

      // Nudge properties
      bool is_nudging_;
      double nudge_stamp_;
      double nudge_duration_;
      ignition::math::Vector3d nudge_force_;
      ignition::math::Vector3d nudge_offset_;

      // Control mode
      bool voltage_mode_;
      bool torque_mode_;
      bool speed_mode_;

      // SDF parameters
      bool pub_ground_truth_;
      bool auto_reset_orientation_;
      double auto_reset_delay_;
  };

  GZ_REGISTER_MODEL_PLUGIN(ControllerInterfacePlugin)

}
