#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <dynamic_reconfigure/server.h>
#include <teeterbot_gazebo/MotorControlConfig.h>

namespace teeterbot_gazebo
{

class MotorController
{
public:
    MotorController ( ros::NodeHandle n, ros::NodeHandle pn, const std::string& name );
    double update (double ts, double cmd, double feedback);

private:
    void reconfigCb ( MotorControlConfig &config, uint32_t level );

    dynamic_reconfigure::Server<MotorControlConfig> srv_;
    MotorControlConfig cfg_;

    double target_;
    double last_error_;
    double int_val_;
};

}
#endif // MOTORCONTROLLER_H
