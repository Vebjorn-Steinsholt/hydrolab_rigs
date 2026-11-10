#pragma once

#include <cmath>
#include <stdexcept>
#include <string>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace rig_controller
{

struct DriveScale
{
  bool angular{false};
  double upright_drive{0.0};
  double upright_m{0.0};
  double m_per_unit{1.0};
  double velocity_per_unit_per_s{1.0 / 6.0}; // velocity tag units per (position unit /s)
  double direction{1.0}; // +1 or -1: rig (+ = up) to drive

  double to_m(double drive) const
  {
    return upright_m + direction * (drive - upright_drive) * m_per_unit;
  }
  double to_drive(double m) const{return upright_drive + direction * (m - upright_m) / m_per_unit;}
  double rate_to_m(double r) const{return r * direction * m_per_unit;}
  double rate_to_drive(double v) const{return direction * v / m_per_unit;}

  static DriveScale from_parameters(const rclcpp_lifecycle::LifecycleNode & node)
  {
    DriveScale scale;
    const std::string motion = node.get_parameter("drive_motion").as_string();
    scale.upright_m = node.get_parameter("upright_position_m").as_double();
    if (!std::isfinite(scale.upright_m)) {
      throw std::invalid_argument("upright_position_m must be finite");
    }
    scale.direction = node.get_parameter("drive_direction").as_int();
    if ((scale.direction != 1 && scale.direction != -1)) {
      throw std::invalid_argument("drive_direction must be +1 or -1");
    }
    scale.velocity_per_unit_per_s = node.get_parameter("drive_velocity_per_unit_per_s").as_double();
    if (!std::isfinite(scale.velocity_per_unit_per_s) || scale.velocity_per_unit_per_s <= 0.0) {
      throw std::invalid_argument("drive_velocity_per_unit_per_s must be a positive finite value");
    }
    if(motion == "linear") {
      scale.m_per_unit = node.get_parameter("drive_m_per_unit").as_double();
      if (!std::isfinite(scale.m_per_unit) || scale.m_per_unit <= 0.0) {
        throw std::invalid_argument("drive_m_per_unit must be a positive finite value");
      }
      scale.upright_drive = scale.direction * scale.upright_m / scale.m_per_unit;
      return scale;
    }
    if(motion != "angular") {
      throw std::invalid_argument("drive_motion must be 'linear' or 'angular'");
    }
    scale.angular = true;
    scale.upright_drive = node.get_parameter("actuator_upright_angle_deg").as_double();
    scale.m_per_unit = node.get_parameter("actuator_lead_m_per_degree").as_double();
    if (!std::isfinite(scale.upright_drive) || !std::isfinite(scale.m_per_unit) ||
      scale.m_per_unit <= 0.0)
    {
      throw std::invalid_argument("actuator drive needs a finite actuator_upright_angle_deg and "
                    "a positive actuator_lead_m_per_degree");
    }
    return scale;
  }
};
} // namespace rig_controller
