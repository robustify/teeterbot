#ifndef MOTORSPEEDCONTROLLER_H
#define MOTORSPEEDCONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <dynamic_reconfigure/server.h>
#include <teeterbot_controllers/MotorSpeedControlConfig.h>

namespace teeterbot_controllers
{

class MotorSpeedController
{
public:
    MotorSpeedController ( ros::NodeHandle n, ros::NodeHandle pn );

private:
    void reconfigCb ( MotorSpeedControlConfig &config, uint32_t level );
    void recvCmdCb ( const std_msgs::Float64ConstPtr &msg );
    void recvEncoderCb ( const std_msgs::Float64ConstPtr &msg );
    void controlTimerCb ( const ros::TimerEvent &event );

    ros::Publisher pub_voltage_;
    ros::Subscriber sub_speed_cmd_;
    ros::Subscriber sub_encoder_measurement_;
    ros::Timer control_timer_;

    dynamic_reconfigure::Server<MotorSpeedControlConfig> srv_;
    MotorSpeedControlConfig cfg_;

    static const double SAMPLE_TIME = 0.01;
    double speed_cmd_;
    double speed_target_;
    double last_error_;
    double wheel_speed_;
    ros::Time cmd_stamp_;
    double int_val_;
};

}
#endif // MOTORSPEEDCONTROLLER_H
