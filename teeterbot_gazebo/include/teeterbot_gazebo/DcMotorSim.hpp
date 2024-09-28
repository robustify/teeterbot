#pragma once

namespace teeterbot_gazebo {

  class DcMotorSim {
    public:
      using SharedPtr = std::shared_ptr<DcMotorSim>;
      using ConstSharedPtr = std::shared_ptr<DcMotorSim const>;

      DcMotorSim()
      : current(0.0) {}

      double step(double ts, double voltage_in, double joint_vel, double load_torque=0) {
        current += ts / MOTOR_PROPS.inductance * (voltage_in - MOTOR_PROPS.resistance * current - MOTOR_PROPS.torque_constant * joint_vel);
        if (current > MOTOR_PROPS.max_current) {
          current = MOTOR_PROPS.max_current;
        } else if (current < -MOTOR_PROPS.max_current){
          current = -MOTOR_PROPS.max_current;
        }
        return current * MOTOR_PROPS.torque_constant; // Motor torque in Nm
      }

      const double& get_current() { return current; }
      const double& get_torque_constant() const { return MOTOR_PROPS.torque_constant; }

    private:
      double current;

      const struct {
        double inductance = 0.0025;
        double resistance = 0.5;
        double torque_constant = 0.35;
        double max_current = 30.0;
      } MOTOR_PROPS;
  };

}
