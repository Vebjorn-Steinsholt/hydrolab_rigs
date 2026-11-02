#include "indradrive_actuator.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <exception>
#include <limits>
#include <stdexcept>
#include <thread>
#include <utility>

namespace rig_controller
{

namespace
{

template<typename T>
void declare_parameter_if_missing(
  rclcpp_lifecycle::LifecycleNode & node, const std::string & name, const T & default_value)
{
  if (!node.has_parameter(name)) {
    node.declare_parameter<T>(name, default_value);
  }
}

}  // namespace

IndraDriveActuator::IndraDriveActuator(rclcpp_lifecycle::LifecycleNode & node)
: poll_interval_ms_(50)
{
  declare_parameter_if_missing(node, "driver_port", 502);
  declare_parameter_if_missing(node, "driver_unit_id", 1);
  declare_parameter_if_missing(node, "input_base_reg", 0);
  declare_parameter_if_missing(node, "input_word_count", 0);
  declare_parameter_if_missing(node, "output_base_reg", 0x0800);
  declare_parameter_if_missing(node, "output_word_count", 0);
  declare_parameter_if_missing(node, "input_uses_fc4", true);
  declare_parameter_if_missing(node, "status_base_reg", 0);
  declare_parameter_if_missing(node, "status_word_count", 0);
  declare_parameter_if_missing(node, "status_uses_fc4", true);
  declare_parameter_if_missing(node, "poll_interval_ms", 50);
  declare_parameter_if_missing(node, "timeout_ms", 1000);
  declare_parameter_if_missing(node, "reconnect_ms", 2000);
  declare_parameter_if_missing(node, "write_only_dirty", false);
  declare_parameter_if_missing(node, "profibus_address", 2);
  declare_parameter_if_missing(node, "min_position_deg", -550.0);
  declare_parameter_if_missing(node, "max_position_deg", 500.0);
  declare_parameter_if_missing(node, "min_position_m", 0.0);
  declare_parameter_if_missing(node, "max_position_m", 1.0);
  declare_parameter_if_missing(node, "max_velocity_rpm", 100.0);
  declare_parameter_if_missing(node, "hold_velocity_rpm", 10.0);
  declare_parameter_if_missing(node, "in_position_tol_deg", 0.1);
  declare_parameter_if_missing(node, "step_timeout_ms", 10000);
  declare_parameter_if_missing(node, "drive_motion", std::string("linear"));
  declare_parameter_if_missing(node, "actuator_lead_m_per_degree", 0.0);

  scale_ = DriveScale::from_parameters(node);
  poll_interval_ms_ = node.get_parameter("poll_interval_ms").as_int();

  const std::string host = node.get_parameter("driver_address").as_string();
  const int port = node.get_parameter("driver_port").as_int();
  const int unit_id = node.get_parameter("driver_unit_id").as_int();
  const int input_base = node.get_parameter("input_base_reg").as_int();
  const int input_count = node.get_parameter("input_word_count").as_int();
  const int output_base = node.get_parameter("output_base_reg").as_int();
  const int output_count = node.get_parameter("output_word_count").as_int();
  const int status_base = node.get_parameter("status_base_reg").as_int();
  const int status_count = node.get_parameter("status_word_count").as_int();

  if (host.empty()) {
    throw std::invalid_argument("driver_address must be set before configuring the drive");
  }
  if (port < 1 || port > 65535 || unit_id < 0 || unit_id > 255 ||
    input_base < 0 || input_base > 65535 || input_count <= 0 || input_count > 65535 ||
    output_base < 0 || output_base > 65535 || output_count <= 0 || output_count > 65535 ||
    status_base < 0 || status_base > 65535 || status_count < 0 || status_count > 65535)
  {
    throw std::invalid_argument("invalid Modbus register or connection configuration");
  }

  const int profibus_address = node.get_parameter("profibus_address").as_int();
  if (profibus_address < 0 || profibus_address > 125) {
    throw std::invalid_argument("profibus_address must be between 0 and 125");
  }

  mgate::GatewayConfig gateway_config;
  gateway_config.host = host;
  gateway_config.port = static_cast<std::uint16_t>(port);
  gateway_config.unit_id = static_cast<std::uint8_t>(unit_id);
  gateway_config.input_base_reg = static_cast<std::uint16_t>(input_base);
  gateway_config.input_word_count = static_cast<std::uint16_t>(input_count);
  gateway_config.output_base_reg = static_cast<std::uint16_t>(output_base);
  gateway_config.output_word_count = static_cast<std::uint16_t>(output_count);
  gateway_config.input_uses_fc4 = node.get_parameter("input_uses_fc4").as_bool();
  gateway_config.status_base_reg = static_cast<std::uint16_t>(status_base);
  gateway_config.status_word_count = static_cast<std::uint16_t>(status_count);
  gateway_config.status_uses_fc4 = node.get_parameter("status_uses_fc4").as_bool();
  gateway_config.poll_interval_ms = node.get_parameter("poll_interval_ms").as_int();
  gateway_config.timeout_ms = node.get_parameter("timeout_ms").as_int();
  gateway_config.reconnect_ms = node.get_parameter("reconnect_ms").as_int();
  gateway_config.write_only_dirty = node.get_parameter("write_only_dirty").as_bool();

  if (gateway_config.poll_interval_ms <= 0 || gateway_config.timeout_ms <= 0 ||
    gateway_config.reconnect_ms <= 0)
  {
    throw std::invalid_argument("poll_interval_ms, timeout_ms, and reconnect_ms must be positive");
  }

  mgate::IndraDriveConfig drive_config;
  drive_config.profibus_address = profibus_address;
  if (scale_.angular) {
    drive_config.min_position = node.get_parameter("min_position_deg").as_double();
    drive_config.max_position = node.get_parameter("max_position_deg").as_double();
  } else {
    drive_config.min_position = node.get_parameter("min_position_m").as_double();
    drive_config.max_position = node.get_parameter("max_position_m").as_double();
  }
  drive_config.max_velocity_rpm = node.get_parameter("max_velocity_rpm").as_double();
  drive_config.in_position_tol_deg = node.get_parameter("in_position_tol_deg").as_double();
  drive_config.step_timeout_ms = node.get_parameter("step_timeout_ms").as_int();

  if (drive_config.min_position >= drive_config.max_position ||
    drive_config.max_velocity_rpm <= 0.0 || drive_config.in_position_tol_deg < 0.0 ||
    drive_config.step_timeout_ms <= 0)
  {
    throw std::invalid_argument("invalid IndraDrive limits or timeout");
  }
  hold_velocity_rpm_ = node.get_parameter("hold_velocity_rpm").as_double();
  if (!(hold_velocity_rpm_ > 0.0) || hold_velocity_rpm_ > drive_config.max_velocity_rpm) {
    throw std::invalid_argument("hold_velocity_rpm must be > 0 and <= max_velocity_rpm");
  }
  max_velocity_rpm_ = drive_config.max_velocity_rpm;
  min_position_ = drive_config.min_position;
  max_position_ = drive_config.max_position;

  mgate_driver_ = std::make_unique<mgate::MGateDriver>(gateway_config);
  indradrive_ = std::make_unique<mgate::IndraDrive>(*mgate_driver_, drive_config);
}

void IndraDriveActuator::set_fault_callback(Actuator::FaultCallback callback)
{
  indradrive_->set_fault_callback(std::move(callback));
}

bool IndraDriveActuator::start(const rclcpp::Logger & logger)
{
  has_target_ = false;  // enable() parks the target on the current position
  mgate_driver_->start();
  if (!mgate_driver_->wait_for_data(5000)) {
    mgate_driver_->stop();
    RCLCPP_ERROR(logger, "Failed to receive data from the drive within 5 seconds.");
    return false;
  }

  const auto first_stats = mgate_driver_->stats();
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  const auto second_stats = mgate_driver_->stats();

  if (second_stats.cycles <= first_stats.cycles) {
    RCLCPP_ERROR(logger, "MGate polling is not progressing");
    mgate_driver_->stop();
    return false;
  }
  if (second_stats.read_errors > first_stats.read_errors ||
    second_stats.write_errors > first_stats.write_errors)
  {
    RCLCPP_ERROR(logger, "MGate communication errors increased");
    mgate_driver_->stop();
    return false;
  }
  if (!second_stats.last_error.empty()) {
    RCLCPP_ERROR(logger, "MGate reported an error: %s", second_stats.last_error.c_str());
    mgate_driver_->stop();
    return false;
  }

  const auto poll_interval_us = static_cast<long long>(poll_interval_ms_) * 1000;
  if (second_stats.last_cycle_us >= poll_interval_us) {
    RCLCPP_ERROR(logger, "MGate cycle time is too long");
    mgate_driver_->stop();
    return false;
  }

  if (!indradrive_->slave_live()) {
    RCLCPP_ERROR(logger, "Drive is not live on PROFIBUS");
    mgate_driver_->stop();
    return false;
  }

  if (!indradrive_->enable()) {
    RCLCPP_ERROR(logger, "Failed to enable the drive: %s", indradrive_->last_error().c_str());
    mgate_driver_->stop();
    return false;
  }

  RCLCPP_INFO(logger, "Drive status: %s", indradrive_->describe().c_str());
  return true;
}

void IndraDriveActuator::halt()
{
  if (!indradrive_ || indradrive_->faulted()) {
    return;
  }
  // Hold position instead of IndraDrive::halt(): Drive Halt makes the drive ignore
  // TargetPosition until enable() runs again, so the next goal or return would not move.
  // Hold the last commanded target rather than the polled position, which can be one
  // gateway cycle old, so a stop at speed does not move back.
  const double target = has_target_ ? last_target_.load() : indradrive_->position_deg();
  indradrive_->move_to(std::clamp(target, min_position_, max_position_), hold_velocity_rpm_);
}

void IndraDriveActuator::stop()
{
  has_target_ = false;
  if (indradrive_) {
    indradrive_->disable();
  }
  if (mgate_driver_) {
    mgate_driver_->stop();
  }
}

bool IndraDriveActuator::write_setpoint(const ActuatorSetpoint & setpoint)
{
  if (!indradrive_) {
    return false;
  }

  // setpoint.velocity is the positioning-speed limit for this move, not a target speed.
  // IndraDrive expects a positive feedrate in rpm; direction comes from the target position.
  const double velocity_rpm = std::max(0.1, std::abs(setpoint.velocity) / 6.0);
  if (!indradrive_->move_to(setpoint.position, velocity_rpm)) {
    return false;
  }
  last_target_ = setpoint.position;
  has_target_ = true;
  return true;
}

int IndraDriveActuator::update_period_ms() const
{
  // The MGate driver sends the staged outputs once per poll cycle.
  return poll_interval_ms_;
}

double IndraDriveActuator::max_velocity_mps() const
{
  // Inverse of write_setpoint(): rpm = |velocity in actuator units per second| / 6.
  return scale_.rate_to_m(max_velocity_rpm_ * 6.0);
}
double IndraDriveActuator::max_acceleration_mps2() const
{
  return std::numeric_limits<double>::infinity();
}

bool IndraDriveActuator::is_live() const
{
  return indradrive_ && indradrive_->slave_live();
}

bool IndraDriveActuator::faulted() const
{
  return indradrive_ && indradrive_->faulted();
}

std::string IndraDriveActuator::fault_reason() const
{
  if (!indradrive_) {
    return "actuator is not configured";
  }
  if (indradrive_->faulted()) {
    return indradrive_->fault_reason();
  }
  return indradrive_->last_error();
}

std::string IndraDriveActuator::status() const
{
  return indradrive_ ? indradrive_->describe() : std::string{};
}

double IndraDriveActuator::actual_position_m() const
{
  return scale_.to_m(indradrive_->position_deg());
}

ActuatorSetpoint IndraDriveActuator::to_actuator_setpoint(
  double position_m, double velocity_mps, double acceleration_mps2) const
{
  return {scale_.to_drive(position_m), scale_.rate_to_drive(velocity_mps),
    scale_.rate_to_drive(acceleration_mps2)};
}

}  // namespace rig_controller
