#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <string>

#include "actuator.hpp"
#include "drive_scale.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "mgate/indradrive.h"
#include "mgate/mgate_driver.h"

namespace rig_controller
{

class IndraDriveActuator final : public Actuator
{
public:
  explicit IndraDriveActuator(rclcpp_lifecycle::LifecycleNode & node);
  void set_fault_callback(FaultCallback callback) override;

  bool start(const rclcpp::Logger & logger) override;
  void halt() override;
  void stop() override;
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
  std::unique_ptr<mgate::MGateDriver> mgate_driver_;
  std::unique_ptr<mgate::IndraDrive> indradrive_;
  DriveScale scale_;
  int poll_interval_ms_;
  // Drive limits in actuator units, and the speed cap used when holding position.
  double min_position_{0.0};
  double max_position_{0.0};
  double hold_velocity_rpm_{10.0};
  double max_velocity_rpm_{0.0};
  // Last target accepted by move_to(); halt() holds here instead of using Drive Halt.
  std::atomic<double> last_target_{0.0};
  std::atomic<bool> has_target_{false};
};

}  // namespace rig_controller
