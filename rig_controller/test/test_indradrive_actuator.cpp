#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "indradrive_actuator.hpp"

namespace
{

rclcpp::NodeOptions actuator_options()
{
  return rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("driver_address", "127.0.0.1"),
      rclcpp::Parameter("driver_port", 1502),
      rclcpp::Parameter("driver_unit_id", 1),
      rclcpp::Parameter("input_base_reg", 0),
      rclcpp::Parameter("input_word_count", 7),
      rclcpp::Parameter("output_base_reg", 0),
      rclcpp::Parameter("output_word_count", 6),
      rclcpp::Parameter("input_uses_fc4", true),
      rclcpp::Parameter("status_base_reg", 768),
      rclcpp::Parameter("status_word_count", 9),
      rclcpp::Parameter("status_uses_fc4", true),
      rclcpp::Parameter("poll_interval_ms", 50),
      rclcpp::Parameter("timeout_ms", 1000),
      rclcpp::Parameter("reconnect_ms", 2000),
      rclcpp::Parameter("write_only_dirty", false),
      rclcpp::Parameter("profibus_address", 2),
      rclcpp::Parameter("min_position_deg", -550.0),
      rclcpp::Parameter("max_position_deg", 500.0),
      rclcpp::Parameter("min_position_m", 0.0),
      rclcpp::Parameter("max_position_m", 0.5),
      rclcpp::Parameter("max_velocity_rpm", 50.0),
      rclcpp::Parameter("in_position_tol_deg", 0.1),
      rclcpp::Parameter("step_timeout_ms", 10000),
      rclcpp::Parameter("drive_motion", "angular"),
      rclcpp::Parameter("actuator_lead_m_per_degree", 0.00053),
      rclcpp::Parameter("actuator_upright_angle_deg", 12.0),
      rclcpp::Parameter("upright_position_m", 0.25),
  });
}

class IndraDriveActuatorTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

}  // namespace

TEST_F(IndraDriveActuatorTest, ConvertsAngularWavemakerSetpoint)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "actuator_test", actuator_options());
  node->declare_parameter<std::string>("driver_address", "127.0.0.1");
  node->declare_parameter<double>("actuator_upright_angle_deg", 12.0);
  node->declare_parameter<double>("upright_position_m", 0.25);
  rig_controller::IndraDriveActuator actuator(*node);

  const auto setpoint = actuator.to_actuator_setpoint(0.25, 0.053, 0.53);

  EXPECT_DOUBLE_EQ(setpoint.position, 12.0);
  EXPECT_DOUBLE_EQ(setpoint.velocity, 100.0);
  // Converted like the velocity (per degree), although the IndraDrive does not use it.
  EXPECT_DOUBLE_EQ(setpoint.acceleration, 1000.0);
  EXPECT_TRUE(std::isinf(actuator.max_acceleration_mps2()));
  EXPECT_FALSE(actuator.faulted());
  EXPECT_TRUE(actuator.fault_reason().empty());

}

TEST_F(IndraDriveActuatorTest, RejectsHoldVelocityAboveDriveMaximum)
{
  auto options = actuator_options();
  options.append_parameter_override("hold_velocity_rpm", 60.0);  // max_velocity_rpm is 50
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "actuator_hold_velocity_test", options);
  node->declare_parameter<std::string>("driver_address", "127.0.0.1");
  node->declare_parameter<double>("actuator_upright_angle_deg", 12.0);
  node->declare_parameter<double>("upright_position_m", 0.25);

  EXPECT_THROW(rig_controller::IndraDriveActuator actuator(*node), std::invalid_argument);
}

TEST_F(IndraDriveActuatorTest, HaltBeforeStartDoesNotThrow)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "actuator_halt_test", actuator_options());
  node->declare_parameter<std::string>("driver_address", "127.0.0.1");
  node->declare_parameter<double>("actuator_upright_angle_deg", 12.0);
  node->declare_parameter<double>("upright_position_m", 0.25);
  rig_controller::IndraDriveActuator actuator(*node);

  // The drive is not enabled, so move_to() refuses the hold; halt() must still be safe.
  EXPECT_NO_THROW(actuator.halt());
  EXPECT_FALSE(actuator.faulted());
}

TEST_F(IndraDriveActuatorTest, CanBeRecreatedAfterParametersAreDeclared)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "actuator_reconfigure_test", actuator_options());
  node->declare_parameter<std::string>("driver_address", "127.0.0.1");
  node->declare_parameter<double>("actuator_upright_angle_deg", 12.0);
  node->declare_parameter<double>("upright_position_m", 0.25);

  EXPECT_NO_THROW({
    rig_controller::IndraDriveActuator first(*node);
    rig_controller::IndraDriveActuator second(*node);
  });
}
