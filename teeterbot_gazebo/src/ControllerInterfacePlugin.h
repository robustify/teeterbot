#ifndef CONTROLLERINTERFACEPLUGIN_H
#define CONTROLLERINTERFACEPLUGIN_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <teeterbot_gazebo/NudgeTeeterbot.h>

#include "MotorController.h"
#include "DcMotorSim.h"

namespace gazebo
{

class ControllerInterfacePlugin : public ModelPlugin
{
public:
    ControllerInterfacePlugin();
    virtual ~ControllerInterfacePlugin();

protected:
    virtual void Load ( physics::ModelPtr model, sdf::ElementPtr sdf );
    virtual void Reset();

private:
    bool nudgeCb(teeterbot_gazebo::NudgeTeeterbotRequest& req, teeterbot_gazebo::NudgeTeeterbotResponse& res);
    void OnUpdate ( const common::UpdateInfo &info );
    void data100Cb ( const ros::TimerEvent &event );
    void recvMotorCmd ( const std_msgs::Float64ConstPtr &msg, int side );
    void getEuler ( double &roll, double &pitch, double &yaw );

    // ROS
    ros::NodeHandle *n_;
    ros::Publisher pub_left_encoder_;
    ros::Publisher pub_right_encoder_;
    ros::Publisher pub_left_current_;
    ros::Publisher pub_right_current_;
    ros::Publisher pub_fallen_over_;
    ros::Subscriber sub_left_cmd_;
    ros::Subscriber sub_right_cmd_;
    ros::ServiceServer nudge_srv_;
    ros::Timer data_100Hz_timer_;
    tf::TransformBroadcaster broadcaster_;

    // Gazebo
    event::ConnectionPtr update_connection_;
    physics::ModelPtr model_;
    physics::JointPtr left_wheel_joint_;
    physics::JointPtr right_wheel_joint_;
    physics::LinkPtr left_wheel_link_;
    physics::LinkPtr right_wheel_link_;
    physics::LinkPtr body_link_;

    // DC motor simulation instances
    boost::shared_ptr<teeterbot_gazebo::DcMotorSim> left_motor_;
    boost::shared_ptr<teeterbot_gazebo::DcMotorSim> right_motor_;

    // Controller instances
    boost::shared_ptr<teeterbot_gazebo::MotorController> left_control_;
    boost::shared_ptr<teeterbot_gazebo::MotorController> right_control_;

    // Status properties
    bool fallen_over_;
    double fallen_over_stamp_;
    double left_cmd_;
    double right_cmd_;

    // Nudge properties
    bool is_nudging_;
    double nudge_stamp_;
    double nudge_duration_;
#if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Vector3d nudge_force_;
    ignition::math::Vector3d nudge_offset_;
#else
    math::Vector3 nudge_force_;
    math::Vector3 nudge_offset_;
#endif

    // Control mode
    bool voltage_mode_;
    bool torque_mode_;
    bool speed_mode_;

    // SDF parameters
    bool pub_ground_truth_;
    bool auto_reset_orientation_;
    double auto_reset_delay_;
};

GZ_REGISTER_MODEL_PLUGIN ( ControllerInterfacePlugin )

}

#endif // CONTROLLERINTERFACEPLUGIN_H
