#pragma once

namespace teeterbot_gazebo {

  struct MotorControllerParams {
    double ramp_limit;
    double kp;
    double ki;
    double kd;
  };

  static const MotorControllerParams TORQUE_PARAMS{
    .ramp_limit = 1000.0,
    .kp = 10.0,
    .ki = 2.0,
    .kd = 0.0
  };

  static const MotorControllerParams SPEED_PARAMS{
    .ramp_limit = 20.0,
    .kp = 3.0,
    .ki = 2.0,
    .kd = 0.0
  };

  class MotorController {
    public:
      using SharedPtr = std::shared_ptr<MotorController>;
      using ConstSharedPtr = std::shared_ptr<MotorController const>;

      MotorController(const MotorControllerParams& params)
      : target(0.0), last_error(0.0), int_val(0.0),
        ramp_limit(params.ramp_limit), kp(params.kp), ki(params.ki), kd(params.kd)
      {}

      double update(double ts, double cmd, double feedback) {
        // Reset integrator when target is near zero
        if (std::abs(target) < 1e-2) {
          int_val = 0;
        }

        // Impose ramp limit on target speed
        if ((cmd - target) > (0.5 * ts * this->ramp_limit)) {
          target += ts * this->ramp_limit;
        } else if ((cmd - target) < (-0.5 * ts * this->ramp_limit)) {
          target -= ts * this->ramp_limit;
        } else {
          target = cmd;
        }

        // Compute voltage command
        double voltage_output;
        double error = target - feedback;
        double derivative = (error - last_error) / ts;
        voltage_output = this->kp * error + this->ki * int_val + this->kd * derivative;
        last_error = error;

        // Saturate voltage command at 0 depending on target speed
        if (target > 0 && voltage_output < 0) {
          voltage_output = 0;
        } else if (target < 0 && voltage_output > 0) {
          voltage_output = 0;
        } else {
          int_val += ts * error;
        }

        return voltage_output;
      }

      void set_ramp_limit(double ramp_limit) { this->ramp_limit = ramp_limit; }
      void set_kp(double kp) { this->kp = kp; }
      void set_ki(double ki) { this->ki = ki; }
      void set_kd(double kd) { this->kd = kd; }

    private:
      // Internal variables
      double target;
      double last_error;
      double int_val;

      // Parameters
      double ramp_limit;
      double kp;
      double ki;
      double kd;
  };

}