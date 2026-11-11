#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdint>
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
#include <rcutils/logging.h>

#include "fake_mgate.hpp"
#include "indradrive_actuator.hpp"
#include "mgate/mgate_driver.h"

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

// T1: IndraDrive's layout is 7 words in, 6 out. A bigger output block would write the next
// slave's outputs every cycle (on the ladertanken daisy chain, the other rig's control word).
TEST_F(IndraDriveActuatorTest, RejectsBlockSizesOtherThanTheDriveLayout)
{
  const std::vector<rclcpp::Parameter> bad{
    rclcpp::Parameter("output_word_count", 12),
    rclcpp::Parameter("output_word_count", 5),
    rclcpp::Parameter("input_word_count", 6),
  };
  int i = 0;
  for (const auto & parameter : bad) {
    auto options = vertical_options();
    options.append_parameter_override(parameter.get_name(), parameter.get_parameter_value());
    auto node = vertical_node("actuator_bad_block_" + std::to_string(i++), options);
    EXPECT_THROW(rig_controller::IndraDriveActuator actuator(*node), std::invalid_argument)
      << parameter.get_name() << " = " << parameter.value_to_string();
  }

  // More inputs only read: allowed.
  auto options = vertical_options();
  options.append_parameter_override("input_word_count", 14);
  auto node = vertical_node("actuator_more_inputs", options);
  EXPECT_NO_THROW(rig_controller::IndraDriveActuator actuator(*node));
}

// Phase 2: the vertical oscillator as slave 3 at registers 7 / 6 behind the wavemaker
// (slave 2 at 0 / 0). It must enable and move without writing slave 2's outputs.
TEST_F(IndraDriveActuatorTest, DaisyChainedDriveLeavesTheOtherSlaveAlone)
{
  FakeMGate fake(7, 6, 0x0C);  // live list: addresses 2 and 3
  auto options = vertical_options(fake.port());
  options.append_parameter_override("input_base_reg", 7);
  options.append_parameter_override("output_base_reg", 6);
  options.append_parameter_override("profibus_address", 3);
  auto node = vertical_node("actuator_daisy_chain_test", options);
  rig_controller::IndraDriveActuator actuator(*node);
  const auto logger = node->get_logger();

  ASSERT_TRUE(actuator.start(logger));
  ASSERT_TRUE(actuator.write_setpoint(actuator.to_actuator_setpoint(0.02, 0.01, 0.0)));
  EXPECT_TRUE(fake.wait_for([&] {return std::abs(fake.target_deg() + 20.0) < 1e-6;}));
  EXPECT_EQ(fake.control_word(), kRunning);
  EXPECT_TRUE(fake.wait_for([&] {return std::abs(actuator.actual_position_m() - 0.02) < 1e-9;}));
  actuator.stop();

  EXPECT_EQ(fake.other_slave_writes(), 0);
}

// The live check uses profibus_address: slave 3 missing from the live list refuses to start.
TEST_F(IndraDriveActuatorTest, DaisyChainedDriveNotLiveRefusesToStart)
{
  FakeMGate fake(7, 6, 0x04);  // only address 2 live
  auto options = vertical_options(fake.port());
  options.append_parameter_override("input_base_reg", 7);
  options.append_parameter_override("output_base_reg", 6);
  options.append_parameter_override("profibus_address", 3);
  auto node = vertical_node("actuator_daisy_chain_down_test", options);
  rig_controller::IndraDriveActuator actuator(*node);

  EXPECT_FALSE(actuator.start(node->get_logger()));
  EXPECT_EQ(fake.other_slave_writes(), 0);
}

// ---------------------------------------------------------------- slow gateway cycles (T17)

namespace
{

// MGateDriver on its own, laid out as FakeMGate expects, polling every 20 ms.
mgate::GatewayConfig fake_gateway(int port)
{
  mgate::GatewayConfig cfg;
  cfg.host = "127.0.0.1";
  cfg.port = static_cast<std::uint16_t>(port);
  cfg.input_base_reg = 0;
  cfg.input_word_count = 7;
  cfg.output_base_reg = 0;
  cfg.output_word_count = 6;
  cfg.status_base_reg = 768;
  cfg.status_word_count = 9;
  cfg.poll_interval_ms = 20;
  cfg.write_only_dirty = false;
  return cfg;
}

bool wait_until(const std::function<bool()> & predicate, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return predicate();
}

constexpr auto kStall = std::chrono::milliseconds(150);  // over slow_cycle_ms (100)
constexpr long long kStallUs = 140000;                   // kStall, less scheduling slack
constexpr long long kFastUs = 50000;                     // a request that was not delayed

// rcutils output handler that keeps the formatted "MGate cycle" warnings.
std::mutex g_log_mtx;
std::vector<std::string> g_slow_logs;

void capture_slow_logs(
  const rcutils_log_location_t *, int severity, const char *, rcutils_time_point_value_t,
  const char * format, va_list * args)
{
  char buffer[512];
  va_list copy;
  va_copy(copy, *args);
  std::vsnprintf(buffer, sizeof(buffer), format, copy);
  va_end(copy);
  const std::string message(buffer);
  if (severity == RCUTILS_LOG_SEVERITY_WARN && message.rfind("MGate cycle", 0) == 0) {
    std::lock_guard<std::mutex> lk(g_log_mtx);
    g_slow_logs.push_back(message);
  }
}

}  // namespace

TEST_F(IndraDriveActuatorTest, NoSlowCyclesWithoutAStall)
{
  FakeMGate fake;
  mgate::MGateDriver driver(fake_gateway(fake.port()));
  driver.start();
  ASSERT_TRUE(wait_until([&] {return driver.stats().cycles >= 15;}, std::chrono::seconds(3)));
  const auto stats = driver.stats();
  driver.stop();

  EXPECT_EQ(stats.slow_cycles, 0u);
  EXPECT_GT(stats.max_cycle_us, 0);
  EXPECT_LT(stats.max_cycle_us, 100000);
}

// Each of the three requests delayed in turn: the slow cycle is counted, and only the delayed
// request's time is long.
TEST_F(IndraDriveActuatorTest, SlowCycleShowsWhichRequestStalled)
{
  using Request = FakeMGate::Request;
  for (const auto request : {Request::Write, Request::Inputs, Request::Status}) {
    SCOPED_TRACE(static_cast<int>(request));
    FakeMGate fake;
    mgate::MGateDriver driver(fake_gateway(fake.port()));
    driver.start();
    ASSERT_TRUE(wait_until([&] {return driver.stats().cycles >= 5;}, std::chrono::seconds(3)));

    fake.delay_next(request, kStall);
    ASSERT_TRUE(wait_until([&] {return driver.stats().slow_cycles >= 1;}, std::chrono::seconds(2)));
    // A few more cycles: a second slow one would show up here.
    const auto cycles = driver.stats().cycles;
    ASSERT_TRUE(wait_until([&] {
        return driver.stats().cycles >= cycles + 5;
      }, std::chrono::seconds(2)));
    const auto stats = driver.stats();
    driver.stop();

    EXPECT_EQ(stats.slow_cycles, 1u);
    EXPECT_GE(stats.slow_cycle_us, kStallUs);
    EXPECT_GE(stats.max_cycle_us, stats.slow_cycle_us);
    EXPECT_EQ(stats.slow_write_us >= kStallUs, request == Request::Write);
    EXPECT_EQ(stats.slow_input_us >= kStallUs, request == Request::Inputs);
    EXPECT_EQ(stats.slow_status_us >= kStallUs, request == Request::Status);
    EXPECT_LT(
      std::min({stats.slow_write_us, stats.slow_input_us, stats.slow_status_us}), kFastUs);
  }
}

// write_setpoint() logs each new slow cycle once, with the stalled request's time.
TEST_F(IndraDriveActuatorTest, WriteSetpointLogsEachSlowCycleOnce)
{
  FakeMGate fake;
  auto node = actuator_node("actuator_slow_cycle_test", actuator_options(fake.port()));
  rig_controller::IndraDriveActuator actuator(*node);
  ASSERT_TRUE(actuator.start(node->get_logger()));
  {
    std::lock_guard<std::mutex> lk(g_log_mtx);
    g_slow_logs.clear();
  }
  const auto previous_handler = rcutils_logging_get_output_handler();
  rcutils_logging_set_output_handler(capture_slow_logs);

  const auto setpoint = actuator.to_actuator_setpoint(0.25, 0.01, 0.0);
  auto write_for = [&](std::chrono::milliseconds duration) {
      const auto end = std::chrono::steady_clock::now() + duration;
      while (std::chrono::steady_clock::now() < end) {
        ASSERT_TRUE(actuator.write_setpoint(setpoint));
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    };
  write_for(std::chrono::milliseconds(300));  // no stall: nothing logged
  fake.delay_next(FakeMGate::Request::Status, kStall);
  write_for(std::chrono::milliseconds(800));  // one stall, many write_setpoint() calls after it

  rcutils_logging_set_output_handler(previous_handler);
  actuator.stop();

  std::lock_guard<std::mutex> lk(g_log_mtx);
  ASSERT_EQ(g_slow_logs.size(), 1u);
  EXPECT_NE(g_slow_logs[0].find("; 1 slow so far"), std::string::npos) << g_slow_logs[0];
  EXPECT_NE(g_slow_logs[0].find("status 1"), std::string::npos) << g_slow_logs[0];  // ~150 ms
}
