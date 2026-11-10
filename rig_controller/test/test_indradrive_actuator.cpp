#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "fake_mgate.hpp"
#include "indradrive_actuator.hpp"

namespace
{

using rig_controller_test::FakeMGate;

rclcpp::NodeOptions actuator_options(int port = 1502)
{
  return rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("driver_address", "127.0.0.1"),
      rclcpp::Parameter("driver_port", port),
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

std::shared_ptr<rclcpp_lifecycle::LifecycleNode> actuator_node(
  const std::string & name, const rclcpp::NodeOptions & options)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(name, options);
  node->declare_parameter<std::string>("driver_address", "127.0.0.1");
  node->declare_parameter<double>("actuator_upright_angle_deg", 12.0);
  node->declare_parameter<double>("upright_position_m", 0.25);
  return node;
}


// The vertical oscillator's block in oscillator.yaml (T2): a linear drive whose position tag is
// in mm and velocity tag in 10 mm/min, + command = down. FakeMGate's *_deg and *_rpm accessors
// read the same raw tags (0.0001 per unit), so here they are mm and 10 mm/min.
rclcpp::NodeOptions vertical_options(int port = 1502, int direction = -1)
{
  return rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("driver_address", "127.0.0.1"),
      rclcpp::Parameter("driver_port", port),
      rclcpp::Parameter("input_base_reg", 0),
      rclcpp::Parameter("input_word_count", 7),
      rclcpp::Parameter("output_base_reg", 0),
      rclcpp::Parameter("output_word_count", 6),
      rclcpp::Parameter("input_uses_fc4", true),
      rclcpp::Parameter("status_base_reg", 768),
      rclcpp::Parameter("status_word_count", 9),
      rclcpp::Parameter("poll_interval_ms", 20),
      rclcpp::Parameter("profibus_address", 2),
      rclcpp::Parameter("drive_motion", "linear"),
      rclcpp::Parameter("drive_m_per_unit", 0.001),
      rclcpp::Parameter("drive_velocity_per_unit_per_s", 6.0),
      rclcpp::Parameter("drive_direction", direction),
      rclcpp::Parameter("upright_position_m", 0.0),
      rclcpp::Parameter("travel_minimum_m", -0.2),
      rclcpp::Parameter("travel_maximum_m", 0.2),
      rclcpp::Parameter("max_velocity_rpm", 600.0),
      rclcpp::Parameter("hold_velocity_rpm", 60.0),
  });
}

std::shared_ptr<rclcpp_lifecycle::LifecycleNode> vertical_node(
  const std::string & name, const rclcpp::NodeOptions & options)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(name, options);
  node->declare_parameter<std::string>("driver_address", "127.0.0.1");
  node->declare_parameter<double>("upright_position_m", 0.0);
  return node;
}

constexpr std::uint16_t kRunning =
  FakeMGate::kOperatingMode | FakeMGate::kDriveOn | FakeMGate::kDriveStart;
constexpr double kHoldVelocityRpm = 10.0;  // hold_velocity_rpm default

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

// The tests below stop the actuator before it goes out of scope: it has no destructor, and
// indradrive_ is destroyed before mgate_driver_ stops the poll thread that calls into it.

TEST_F(IndraDriveActuatorTest, HaltHoldsLastTargetWithHoldSpeed)
{
  FakeMGate fake;
  auto node = actuator_node("actuator_halt_target_test", actuator_options(fake.port()));
  rig_controller::IndraDriveActuator actuator(*node);
  const auto logger = node->get_logger();

  ASSERT_TRUE(actuator.start(logger));
  ASSERT_TRUE(actuator.write_setpoint({100.0, 300.0, 0.0}));  // 300 deg/s = 50 rpm
  ASSERT_TRUE(fake.wait_for([&] {
      return std::abs(fake.target_deg() - 100.0) < 1e-6 &&
             std::abs(fake.speed_cap_rpm() - 50.0) < 1e-6;
    }));

  // The drive reports a position behind the target; halt() must not hold that one.
  fake.freeze_position(95.0);
  const int reads = fake.input_reads();
  ASSERT_TRUE(fake.wait_for([&] {return fake.input_reads() >= reads + 2;}));

  EXPECT_NO_THROW(actuator.halt());
  EXPECT_TRUE(fake.wait_for([&] {
      return std::abs(fake.speed_cap_rpm() - kHoldVelocityRpm) < 1e-6;
    }));
  EXPECT_NEAR(fake.target_deg(), 100.0, 1e-6);
  EXPECT_EQ(fake.control_word(), kRunning);  // Drive Start stays set
  EXPECT_FALSE(actuator.faulted());

  actuator.stop();
}

TEST_F(IndraDriveActuatorTest, HaltWithoutTargetHoldsPolledPositionClamped)
{
  FakeMGate fake;
  fake.set_position_deg(600.0);  // max_position_deg is 500; start() parks the target here
  auto node = actuator_node("actuator_halt_clamp_test", actuator_options(fake.port()));
  rig_controller::IndraDriveActuator actuator(*node);
  const auto logger = node->get_logger();

  ASSERT_TRUE(actuator.start(logger));
  ASSERT_NEAR(fake.target_deg(), 600.0, 1e-6);

  EXPECT_NO_THROW(actuator.halt());
  EXPECT_TRUE(fake.wait_for([&] {return std::abs(fake.target_deg() - 500.0) < 1e-6;}));
  EXPECT_NEAR(fake.speed_cap_rpm(), kHoldVelocityRpm, 1e-6);
  EXPECT_EQ(fake.control_word(), kRunning);
  EXPECT_FALSE(actuator.faulted());

  actuator.stop();
}

TEST_F(IndraDriveActuatorTest, HaltAfterStopDoesNothing)
{
  FakeMGate fake;
  fake.set_position_deg(20.0);
  auto node = actuator_node("actuator_halt_stopped_test", actuator_options(fake.port()));
  rig_controller::IndraDriveActuator actuator(*node);
  const auto logger = node->get_logger();

  ASSERT_TRUE(actuator.start(logger));
  ASSERT_TRUE(actuator.write_setpoint({30.0, 60.0, 0.0}));
  ASSERT_TRUE(fake.wait_for([&] {return std::abs(fake.target_deg() - 30.0) < 1e-6;}));
  actuator.stop();
  // disable() leaves operating mode with torque off; the gateway saw that before stop() returned.
  ASSERT_EQ(fake.control_word(), FakeMGate::kOperatingMode);

  EXPECT_NO_THROW(actuator.halt());
  std::this_thread::sleep_for(std::chrono::milliseconds(150));  // three poll cycles
  EXPECT_EQ(fake.control_word(), FakeMGate::kOperatingMode);
  EXPECT_NEAR(fake.target_deg(), 30.0, 1e-6);
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


// ---------------------------------------------------------------- linear drives (T2)

TEST_F(IndraDriveActuatorTest, LinearScaleConvertsMetresToDriveUnits)
{
  auto node = vertical_node("actuator_linear_test", vertical_options(1502, 1));
  rig_controller::IndraDriveActuator actuator(*node);

  const auto setpoint = actuator.to_actuator_setpoint(0.1, 0.05, 1.0);

  EXPECT_DOUBLE_EQ(setpoint.position, 100.0);      // mm
  EXPECT_DOUBLE_EQ(setpoint.velocity, 50.0);       // mm/s
  EXPECT_DOUBLE_EQ(setpoint.acceleration, 1000.0);  // mm/s^2
  // 600 tag units = 6000 mm/min = 100 mm/s.
  EXPECT_DOUBLE_EQ(actuator.max_velocity_mps(), 0.1);
}

TEST_F(IndraDriveActuatorTest, NegativeDirectionFlipsPositionAndVelocity)
{
  auto node = vertical_node("actuator_direction_test", vertical_options());
  rig_controller::IndraDriveActuator actuator(*node);

  const auto setpoint = actuator.to_actuator_setpoint(0.1, 0.05, 1.0);

  EXPECT_DOUBLE_EQ(setpoint.position, -100.0);
  EXPECT_DOUBLE_EQ(setpoint.velocity, -50.0);
  EXPECT_DOUBLE_EQ(setpoint.acceleration, -1000.0);
  EXPECT_DOUBLE_EQ(actuator.max_velocity_mps(), 0.1);  // a speed: no sign
}

TEST_F(IndraDriveActuatorTest, AngularSpeedLimitUnchanged)
{
  // Ladertanken's numbers before T2: 50 rpm = 300 deg/s at 0.53 mm/deg.
  auto node = actuator_node("actuator_angular_speed_test", actuator_options());
  rig_controller::IndraDriveActuator actuator(*node);

  EXPECT_DOUBLE_EQ(actuator.max_velocity_mps(), 300.0 * 0.00053);
}

TEST_F(IndraDriveActuatorTest, RejectsInvalidDriveScale)
{
  const std::vector<rclcpp::Parameter> bad{
    rclcpp::Parameter("drive_direction", 0),
    rclcpp::Parameter("drive_direction", 2),
    rclcpp::Parameter("drive_m_per_unit", 0.0),
    rclcpp::Parameter("drive_m_per_unit", -0.001),
    rclcpp::Parameter("drive_velocity_per_unit_per_s", 0.0),
    rclcpp::Parameter("drive_motion", "rotary"),
  };
  int i = 0;
  for (const auto & parameter : bad) {
    auto options = vertical_options();
    options.append_parameter_override(parameter.get_name(), parameter.get_parameter_value());
    auto node = vertical_node("actuator_bad_scale_" + std::to_string(i++), options);
    EXPECT_THROW(rig_controller::IndraDriveActuator actuator(*node), std::invalid_argument)
      << parameter.get_name() << " = " << parameter.value_to_string();
  }
}

TEST_F(IndraDriveActuatorTest, WritesDriveUnitsToTheGateway)
{
  FakeMGate fake;
  auto node = vertical_node("actuator_linear_write_test", vertical_options(fake.port()));
  rig_controller::IndraDriveActuator actuator(*node);
  const auto logger = node->get_logger();

  ASSERT_TRUE(actuator.start(logger));
  // +0.1 m (up) at 0.05 m/s: -100 mm on the drive (+ = down), raw -1 000 000; the speed cap
  // is positive, 50 mm/s = 300 x 10 mm/min, raw 3 000 000.
  ASSERT_TRUE(actuator.write_setpoint(actuator.to_actuator_setpoint(0.1, 0.05, 0.0)));
  EXPECT_TRUE(fake.wait_for([&] {
      return std::abs(fake.target_deg() + 100.0) < 1e-6 &&
             std::abs(fake.speed_cap_rpm() - 300.0) < 1e-6;
    }));
  EXPECT_EQ(fake.control_word(), kRunning);

  actuator.stop();
}

TEST_F(IndraDriveActuatorTest, ReadsDrivePositionBackInMetres)
{
  FakeMGate fake;
  auto node = vertical_node("actuator_linear_read_test", vertical_options(fake.port()));
  rig_controller::IndraDriveActuator actuator(*node);
  const auto logger = node->get_logger();

  ASSERT_TRUE(actuator.start(logger));
  fake.freeze_position(50.0);  // +50 mm on the drive is 50 mm down
  EXPECT_TRUE(fake.wait_for([&] {return std::abs(actuator.actual_position_m() + 0.05) < 1e-9;}));

  actuator.stop();
}

TEST_F(IndraDriveActuatorTest, SpeedCapClampedToDriveMaximum)
{
  FakeMGate fake;
  auto node = vertical_node("actuator_linear_cap_test", vertical_options(fake.port()));
  rig_controller::IndraDriveActuator actuator(*node);
  const auto logger = node->get_logger();

  ASSERT_TRUE(actuator.start(logger));
  // 0.2 m/s would be 1200 tag units; max_velocity_rpm is 600.
  ASSERT_TRUE(actuator.write_setpoint(actuator.to_actuator_setpoint(0.05, 0.2, 0.0)));
  EXPECT_TRUE(fake.wait_for([&] {return std::abs(fake.speed_cap_rpm() - 600.0) < 1e-6;}));

  actuator.stop();
}

TEST_F(IndraDriveActuatorTest, DriveLimitsFromTravelWithDirectionSwapped)
{
  // Travel -0.2..+0.2 m with + = down becomes drive limits -200..+200 mm (min and max
  // swapped). halt() without a target holds the polled position clamped to them.
  for (const double outside_mm : {250.0, -250.0}) {
    FakeMGate fake;
    fake.set_position_deg(outside_mm);
    auto node = vertical_node(
      outside_mm > 0 ? "actuator_limit_high_test" : "actuator_limit_low_test",
      vertical_options(fake.port()));
    rig_controller::IndraDriveActuator actuator(*node);
    const auto logger = node->get_logger();

    ASSERT_TRUE(actuator.start(logger));
    actuator.halt();
    EXPECT_TRUE(fake.wait_for([&] {
        return std::abs(fake.target_deg() - std::copysign(200.0, outside_mm)) < 1e-6;
      })) << "from " << outside_mm << " mm";
    EXPECT_NEAR(fake.speed_cap_rpm(), 60.0, 1e-6);  // hold_velocity_rpm
    actuator.stop();
  }
}
