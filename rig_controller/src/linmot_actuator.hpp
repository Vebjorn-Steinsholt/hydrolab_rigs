#pragma once

#include <memory>
#include <string>

#include "actuator.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "linmot/linmot_drive.h"
#include "linmot/serial_bridge.h"

namespace rig_controller
{

class LinMotActuator final : public Actuator
{
public:
  explicit LinMotActuator(rclcpp_lifecycle::LifecycleNode & node);
  void set_fault_callback(FaultCallback callback) override;
  bool start(const rclcpp::Logger & logger) override;
  void stop() override;
  void halt() override;
  bool is_live() const override;
  bool faulted() const override;
  std::string fault_reason() const override;
  std::string status() const override;
  double actual_position_m() const override;
  ActuatorSetpoint to_actuator_setpoint(
    double position_m, double velocity_mps, double acceleration_mps2) const override;
  bool write_setpoint(const ActuatorSetpoint & setpoint) override;
  double max_velocity_mps() const override;
  double max_acceleration_mps2() const override;
  int update_period_ms() const override;

private:
  std::unique_ptr<linmot::SerialBridge> bridge_;
  std::unique_ptr<linmot::LinMotDrive> drive_;  // after bridge_: destroyed first
  int period_ms_{20};
  double max_velocity_mps_{0.0};
  double max_acceleration_mps2_{0.0};
};

}  // namespace rig_controller
