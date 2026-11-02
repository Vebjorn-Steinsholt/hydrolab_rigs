#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "linmot/testing/bridge_sim.h"
#include "linmot_actuator.hpp"

namespace
{

using linmot::Command;
using linmot_test::BridgeSim;

// Short timeouts so the simulated failures do not slow the test down.
rclcpp::NodeOptions actuator_options(
  const std::string & serial_port,
  const std::vector<rclcpp::Parameter> & extra = {})
{
  std::vector<rclcpp::Parameter> parameters{
    rclcpp::Parameter("serial_port", serial_port),
    rclcpp::Parameter("ready_timeout_ms", 1000),
    rclcpp::Parameter("step_timeout_ms", 300),
    rclcpp::Parameter("home_timeout_ms", 1000),
    rclcpp::Parameter("feedback_timeout_ms", 100),
  };
  parameters.insert(parameters.end(), extra.begin(), extra.end());
  return rclcpp::NodeOptions().parameter_overrides(parameters);
}

std::shared_ptr<rclcpp_lifecycle::LifecycleNode> make_node(
  const std::string & name, const rclcpp::NodeOptions & options)
{
  return std::make_shared<rclcpp_lifecycle::LifecycleNode>(name, options);
}

bool wait_until(const std::function<bool()> & condition, int timeout_ms = 1000)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (condition()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return condition();
}

bool contains(const std::string & text, const std::string & part)
{
  return text.find(part) != std::string::npos;
}

class LinMotActuatorTest : public ::testing::Test
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

TEST_F(LinMotActuatorTest, RejectsInvalidConfiguration)
{
  auto no_port = make_node("linmot_invalid_port", actuator_options(""));
  EXPECT_THROW(rig_controller::LinMotActuator actuator(*no_port), std::invalid_argument);

  const std::vector<std::vector<rclcpp::Parameter>> invalid{
    {rclcpp::Parameter("min_position_m", 0.1), rclcpp::Parameter("max_position_m", 0.1)},
    {rclcpp::Parameter("max_position_m", 1.0)},       // beyond the firmware's 195 mm
    {rclcpp::Parameter("min_position_m", 0.0)},       // below the firmware's 5 mm
    {rclcpp::Parameter("max_velocity_mps", 0.0)},
    {rclcpp::Parameter("max_acceleration_mps2", 25.0)},
    {rclcpp::Parameter("max_acceleration_mps2", 0.0)},
    {rclcpp::Parameter("serial_period_ms", 0)},
    {rclcpp::Parameter("drive_motion", "angular")},
  };
  int index = 0;
  for (const auto & parameters : invalid) {
    auto node = make_node(
      "linmot_invalid_" + std::to_string(index++), actuator_options("/dev/null", parameters));
    EXPECT_THROW(rig_controller::LinMotActuator actuator(*node), std::invalid_argument)
      << parameters.front().get_name();
  }
}

TEST_F(LinMotActuatorTest, ConvertsNothingAndReportsLimits)
{
  auto node = make_node("linmot_convert", actuator_options("/dev/null"));
  rig_controller::LinMotActuator actuator(*node);

  const auto setpoint = actuator.to_actuator_setpoint(0.1, 0.03, 1.5);
  EXPECT_DOUBLE_EQ(setpoint.position, 0.1);
  EXPECT_DOUBLE_EQ(setpoint.velocity, 0.03);
  EXPECT_DOUBLE_EQ(setpoint.acceleration, 1.5);
  EXPECT_DOUBLE_EQ(actuator.max_velocity_mps(), 2.0);
  EXPECT_DOUBLE_EQ(actuator.max_acceleration_mps2(), 20.0);
  EXPECT_EQ(actuator.update_period_ms(), 20);
}

TEST_F(LinMotActuatorTest, HaltAndStopBeforeStartAreSafe)
{
  auto node = make_node("linmot_halt", actuator_options("/dev/null"));
  rig_controller::LinMotActuator actuator(*node);

  EXPECT_NO_THROW(actuator.halt());
  EXPECT_NO_THROW(actuator.stop());
  EXPECT_FALSE(actuator.is_live());
  EXPECT_FALSE(actuator.faulted());
  EXPECT_FALSE(actuator.write_setpoint({0.1, 0.03, 1.0}));
}

TEST_F(LinMotActuatorTest, StartFailsOnMissingPort)
{
  auto node = make_node("linmot_no_port", actuator_options("/dev/does-not-exist"));
  rig_controller::LinMotActuator actuator(*node);

  EXPECT_FALSE(actuator.start(node->get_logger()));
  EXPECT_FALSE(actuator.is_live());
  EXPECT_TRUE(contains(actuator.fault_reason(), "cannot open")) << actuator.fault_reason();
}

TEST_F(LinMotActuatorTest, StartHomesFreshDriveAndStreamsSetpoints)
{
  BridgeSim sim({});
  auto node = make_node("linmot_start", actuator_options(sim.device()));
  rig_controller::LinMotActuator actuator(*node);

  ASSERT_TRUE(actuator.start(node->get_logger())) << actuator.fault_reason();
  EXPECT_TRUE(actuator.is_live());
  EXPECT_EQ(sim.count(Command::Home), 1);
  EXPECT_NEAR(actuator.actual_position_m(), 0.1, 1e-9);

  // Velocity and acceleration go to the drive as given; units 1e-6 m/s and 1e-5 m/s^2.
  ASSERT_TRUE(actuator.write_setpoint({0.105, 0.0314, 0.0987}));
  ASSERT_TRUE(wait_until([&] {return sim.position() == 1050000;}));
  auto frames = sim.commands();
  ASSERT_FALSE(frames.empty());
  EXPECT_EQ(frames.back().command, Command::GoToPosition);
  EXPECT_EQ(frames.back().velocity, 31400u);
  EXPECT_EQ(frames.back().acceleration, 9870u);

  // No acceleration, or one above max_acceleration_mps2 (20): refused; the drive keeps the
  // previous target.
  EXPECT_FALSE(actuator.write_setpoint({0.104, 0.0314, 0.0}));
  EXPECT_FALSE(actuator.write_setpoint({0.104, 0.0314, 25.0}));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(sim.position(), 1050000);

  // Outside the limits: refused, which the node treats as a fault.
  EXPECT_FALSE(actuator.write_setpoint({0.199, 0.0314, 1.0}));

  actuator.halt();
  EXPECT_TRUE(actuator.is_live());
  actuator.stop();
  EXPECT_FALSE(actuator.is_live());
  EXPECT_EQ(sim.count(Command::SwitchOff), 1);
  // Then pre-operational, so the drive's sync watchdog does not fire while it is idle.
  // stop() returns once the frame is written; the simulator handles it a moment later.
  ASSERT_TRUE(wait_until([&] {return !sim.operational();}));
  EXPECT_EQ(sim.count(Command::PreOperational), 1);
}

TEST_F(LinMotActuatorTest, HomedDriveDoesNotMove)
{
  BridgeSim sim({});
  sim.set_state(true, true);
  auto node = make_node("linmot_homed", actuator_options(sim.device()));
  rig_controller::LinMotActuator actuator(*node);

  ASSERT_TRUE(actuator.start(node->get_logger())) << actuator.fault_reason();
  EXPECT_EQ(sim.count(Command::Home), 0);
  EXPECT_EQ(sim.count(Command::SwitchOn), 0);
  actuator.stop();
}

TEST_F(LinMotActuatorTest, NoHomingWhenNotAllowed)
{
  BridgeSim sim({});
  auto node = make_node(
    "linmot_no_homing",
    actuator_options(sim.device(), {rclcpp::Parameter("home_if_needed", false)}));
  rig_controller::LinMotActuator actuator(*node);

  EXPECT_FALSE(actuator.start(node->get_logger()));
  EXPECT_EQ(sim.count(Command::Home), 0);
  EXPECT_TRUE(contains(actuator.fault_reason(), "not homed")) << actuator.fault_reason();
}

TEST_F(LinMotActuatorTest, DriveErrorFaultsTheActuator)
{
  BridgeSim sim({});
  sim.set_state(true, true);
  auto node = make_node("linmot_fault", actuator_options(sim.device()));
  rig_controller::LinMotActuator actuator(*node);

  std::mutex mutex;
  std::vector<std::string> reasons;
  actuator.set_fault_callback(
    [&](const std::string & reason) {
      std::lock_guard<std::mutex> lock(mutex);
      reasons.push_back(reason);
    });
  ASSERT_TRUE(actuator.start(node->get_logger())) << actuator.fault_reason();
  ASSERT_TRUE(actuator.write_setpoint({0.1, 0.0314, 1.0}));

  sim.raise_error();
  ASSERT_TRUE(wait_until([&] {return actuator.faulted();}));
  EXPECT_FALSE(actuator.is_live());
  EXPECT_TRUE(contains(actuator.fault_reason(), "drive error")) << actuator.fault_reason();
  EXPECT_FALSE(actuator.write_setpoint({0.1, 0.0314, 1.0}));
  {
    std::lock_guard<std::mutex> lock(mutex);
    EXPECT_EQ(reasons.size(), 1u);
  }

  // A new start() acknowledges the error and clears the fault.
  ASSERT_TRUE(actuator.start(node->get_logger())) << actuator.fault_reason();
  EXPECT_FALSE(actuator.faulted());
  EXPECT_EQ(sim.count(Command::AckError), 1);
  actuator.stop();
}
