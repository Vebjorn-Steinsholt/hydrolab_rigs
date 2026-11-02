#pragma once

#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace rig_controller
{

struct ActuatorSetpoint
{
  double position{0.0};
  double velocity{0.0};
  double acceleration{0.0};
};

class Actuator
{
public:
  using FaultCallback = std::function<void(const std::string &)>;

  virtual ~Actuator() = default;

  virtual void set_fault_callback(FaultCallback callback) = 0;
  virtual bool start(const rclcpp::Logger & logger) = 0;
  // Stop moving and hold position, staying enabled so the next setpoint is followed.
  virtual void halt() = 0;
  virtual void stop() = 0;
  virtual bool is_live() const = 0;
  virtual bool faulted() const = 0;
  virtual std::string fault_reason() const = 0;
  virtual std::string status() const = 0;
  virtual double actual_position_m() const = 0;
  virtual ActuatorSetpoint to_actuator_setpoint(
    double position_m, double velocity_mps, double acceleration_mps2) const = 0;
  virtual bool write_setpoint(const ActuatorSetpoint & setpoint) = 0;
  // Fastest paddle speed the drive accepts, in m/s; write_setpoint() fails above it.
  virtual double max_velocity_mps() const = 0;
  // Largest acceleration the drive accepts, in m/s^2; write_setpoint() fails above it.
  virtual double max_acceleration_mps2() const = 0;
  // How often new setpoints actually reach the drive, in ms. The control loops run at this
  // period, because writing faster only overwrites setpoints before they are sent.
  virtual int update_period_ms() const = 0;
};

}  // namespace rig_controller
