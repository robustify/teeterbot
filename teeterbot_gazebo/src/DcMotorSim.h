#ifndef DCMOTORSIM_H
#define DCMOTORSIM_H

#include <ros/ros.h>
#include <gazebo/physics/physics.hh>

namespace teeterbot_gazebo
{

typedef struct {
    double inductance;
    double resistance;
    double torque_constant;
    double max_current;
} DcMotorProps;

class DcMotorSim
{
public:
    DcMotorSim ( ros::NodeHandle n, const gazebo::physics::JointPtr &joint, const gazebo::physics::LinkPtr &link );
    void step ( double ts, double voltage_in, double load_torque=0 );
    double current_;
    DcMotorProps props_;
private:
    gazebo::physics::JointPtr joint_;
};

}

#endif // DCMOTORSIM_H
