#include "linmot_actuator.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace rig_controller
{

namespace
{

// The bridge firmware's own limits (POSITION_MIN/MAX, VELOCITY_MAX, ACCELERATION_MAX in
// firmware/linmot_serial_bridge). Within them, the bridge never rejects a command that
// LinMotDrive::move_to() accepted, which would fault the drive.
constexpr double kFirmwareMinPositionM = 0.005;
constexpr double kFirmwareMaxPositionM = 0.195;
constexpr double kFirmwareMaxVelocityMps = 3.0;
constexpr double kFirmwareMaxAccelerationMps2 = 20.0;

template<typename T>
void declare_parameter_if_missing(
  rclcpp_lifecycle::LifecycleNode & node, const std::string & name, const T & default_value)
{
  if (!node.has_parameter(name)) {
    node.declare_parameter<T>(name, default_value);
  }
}

}  // namespace

LinMotActuator::LinMotActuator(rclcpp_lifecycle::LifecycleNode & node)
{
  declare_parameter_if_missing(node, "serial_port", std::string("/dev/ttyACM0"));
  declare_parameter_if_missing(node, "serial_period_ms", 20);
  declare_parameter_if_missing(node, "min_position_m", 0.005);
  declare_parameter_if_missing(node, "max_position_m", 0.195);
  declare_parameter_if_missing(node, "max_velocity_mps", 2.0);
  declare_parameter_if_missing(node, "max_acceleration_mps2", 20.0);
  declare_parameter_if_missing(node, "home_if_needed", true);
  declare_parameter_if_missing(node, "step_timeout_ms", 2000);
  declare_parameter_if_missing(node, "home_timeout_ms", 60000);
  declare_parameter_if_missing(node, "ready_timeout_ms", 3000);
  declare_parameter_if_missing(node, "feedback_timeout_ms", 100);
  declare_parameter_if_missing(node, "drive_motion", std::string("linear"));

  if (node.get_parameter("drive_motion").as_string() != "linear") {
    throw std::invalid_argument("LinMotActuator is linear: drive_motion must be 'linear'");
  }

  linmot::BridgeConfig bridge_config;
  bridge_config.device = node.get_parameter("serial_port").as_string();
  bridge_config.period_ms = static_cast<int>(node.get_parameter("serial_period_ms").as_int());
  linmot::LinMotDriveConfig drive_config;
  drive_config.min_position_m = node.get_parameter("min_position_m").as_double();
  drive_config.max_position_m = node.get_parameter("max_position_m").as_double();
  drive_config.max_velocity_mps = node.get_parameter("max_velocity_mps").as_double();
  drive_config.max_acceleration_mps2 = node.get_parameter("max_acceleration_mps2").as_double();
  drive_config.home_if_needed = node.get_parameter("home_if_needed").as_bool();
  drive_config.step_timeout_ms = static_cast<int>(node.get_parameter("step_timeout_ms").as_int());
  drive_config.home_timeout_ms = static_cast<int>(node.get_parameter("home_timeout_ms").as_int());
  drive_config.ready_timeout_ms =
    static_cast<int>(node.get_parameter("ready_timeout_ms").as_int());
  drive_config.feedback_timeout_ms =
    static_cast<int>(node.get_parameter("feedback_timeout_ms").as_int());

  if (bridge_config.device.empty()) {
    throw std::invalid_argument("serial_port parameter must be specified for LinMotActuator");
  }
  if (bridge_config.period_ms <= 0 ||
    drive_config.min_position_m >= drive_config.max_position_m ||
    drive_config.max_velocity_mps <= 0.0 || drive_config.max_acceleration_mps2 <= 0.0 ||
    drive_config.step_timeout_ms <= 0 || drive_config.home_timeout_ms <= 0 ||
    drive_config.ready_timeout_ms <= 0 || drive_config.feedback_timeout_ms <= 0)
  {
    throw std::invalid_argument("invalid LinMot limits, period or timeouts");
  }
  // min_position_m / max_position_m are declared by the node with 0.0 / 1.0, so a value missing
  // from the yaml shows up here rather than as a rejected command on the rig.
  if (drive_config.min_position_m < kFirmwareMinPositionM ||
    drive_config.max_position_m > kFirmwareMaxPositionM ||
    drive_config.max_velocity_mps > kFirmwareMaxVelocityMps ||
    drive_config.max_acceleration_mps2 > kFirmwareMaxAccelerationMps2)
  {
    throw std::invalid_argument(
            "LinMot limits must be within the bridge firmware's: 0.005-0.195 m, 3 m/s, 20 m/s^2");
  }

  period_ms_ = bridge_config.period_ms;
  max_velocity_mps_ = drive_config.max_velocity_mps;
  max_acceleration_mps2_ = drive_config.max_acceleration_mps2;

  bridge_ = std::make_unique<linmot::SerialBridge>(bridge_config);
  drive_ = std::make_unique<linmot::LinMotDrive>(*bridge_, drive_config);
}

void LinMotActuator::set_fault_callback(Actuator::FaultCallback callback)
{
  drive_->set_fault_callback(std::move(callback));
}

bool LinMotActuator::start(const rclcpp::Logger & logger)
{
  if (!bridge_->start()) {
    RCLCPP_ERROR(logger, "LinMot bridge: %s", bridge_->stats().last_error.c_str());
    return false;
  }
  const bool ok = drive_->bring_up(
    [&logger](const std::string & message) {RCLCPP_INFO(logger, "LinMot: %s", message.c_str());}
  );
  if (!ok) {
    RCLCPP_ERROR(logger, "Failed to bring up LinMot drive: %s", drive_->last_error().c_str());
    bridge_->stop();
    return false;
  }
  const auto stats = bridge_->stats();
  RCLCPP_INFO(
    logger, "Drive status: %s (start-up bytes skipped: %llu)", drive_->describe().c_str(),
    static_cast<unsigned long long>(stats.dropped_bytes));
  return true;
}

void LinMotActuator::halt()
{
  drive_->halt();
}

void LinMotActuator::stop()
{
  drive_->disable();
  bridge_->stop();
}

bool LinMotActuator::write_setpoint(const ActuatorSetpoint & setpoint)
{
  // setpoint.velocity is the speed limit for this motion (peak x margin), not a target speed;
  // the VAI command needs a positive limit. Until step C passes setpoint.acceleration, use the
  // configured default.
  const double velocity = std::max(std::abs(setpoint.velocity), 1e-3);
  return drive_->move_to(setpoint.position, velocity, setpoint.acceleration);
}

int LinMotActuator::update_period_ms() const
{
  return period_ms_;
}

double LinMotActuator::max_velocity_mps() const
{
  return max_velocity_mps_;
}
double LinMotActuator::max_acceleration_mps2() const
{
  return max_acceleration_mps2_;
}
bool LinMotActuator::is_live() const
{
  return bridge_->port_ok() && bridge_->ready() && drive_->enabled() && !drive_->faulted();
}

bool LinMotActuator::faulted() const
{
  return drive_->faulted();
}

std::string LinMotActuator::fault_reason() const
{
  if (drive_->faulted()) {
    return drive_->fault_reason();
  }
  // After a failed start(): bring_up() sets last_error(); a port that did not open only the
  // bridge knows about.
  const std::string error = drive_->last_error();
  return error.empty() ? bridge_->stats().last_error : error;
}

std::string LinMotActuator::status() const
{
  return drive_->describe();
}

double LinMotActuator::actual_position_m() const
{
  return drive_->position_m();
}

ActuatorSetpoint LinMotActuator::to_actuator_setpoint(
  double position_m, double velocity_mps, double acceleration_mps2) const
{
  return {position_m, velocity_mps, acceleration_mps2};
}

}  // namespace rig_controller
