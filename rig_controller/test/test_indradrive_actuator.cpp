#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "indradrive_actuator.hpp"

namespace
{

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

// Modbus/TCP server standing in for the MGate 5101 and the IndraDrive behind it, laid out as
// actuator_options() expects: inputs (FC4) at 0, outputs (FC3/FC6/FC16) at 0, gateway status
// (FC4) at 768. The status word follows the control word the way IndraDrive::enable() waits
// for it, and the position follows TargetPosition while the drive is following.
class FakeMGate
{
public:
  static constexpr std::uint16_t kOperatingMode = 1u << 1;
  static constexpr std::uint16_t kDriveStart = 1u << 13;
  static constexpr std::uint16_t kDriveOn = 1u << 15;

  FakeMGate()
  {
    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0) {
      throw std::runtime_error("FakeMGate: socket() failed");
    }
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;
    socklen_t len = sizeof(addr);
    if (::bind(listen_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0 ||
      ::listen(listen_fd_, 1) != 0 ||
      ::getsockname(listen_fd_, reinterpret_cast<sockaddr *>(&addr), &len) != 0)
    {
      ::close(listen_fd_);
      throw std::runtime_error("FakeMGate: bind/listen failed");
    }
    port_ = ntohs(addr.sin_port);
    thread_ = std::thread([this] {serve();});
  }

  ~FakeMGate()
  {
    stopping_ = true;
    ::shutdown(listen_fd_, SHUT_RDWR);  // wakes accept()
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (conn_fd_ >= 0) {
        ::shutdown(conn_fd_, SHUT_RDWR);  // wakes recv()
      }
    }
    thread_.join();
    ::close(listen_fd_);
  }

  FakeMGate(const FakeMGate &) = delete;
  FakeMGate & operator=(const FakeMGate &) = delete;

  int port() const {return port_;}

  // The drive's measured position. While following, the next output write moves it to the
  // target again; freeze_position() keeps it, as a drive one gateway cycle behind would.
  void set_position_deg(double deg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    position_deg_ = deg;
  }

  void freeze_position(double deg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    position_deg_ = deg;
    frozen_ = true;
  }

  std::uint16_t control_word() const
  {
    std::lock_guard<std::mutex> lk(mtx_);
    return outputs_[0];
  }

  double target_deg() const
  {
    std::lock_guard<std::mutex> lk(mtx_);
    return i32(outputs_[1], outputs_[2]) * 0.0001;
  }

  double speed_cap_rpm() const
  {
    std::lock_guard<std::mutex> lk(mtx_);
    return i32(outputs_[3], outputs_[4]) * 0.0001;
  }

  int input_reads() const
  {
    std::lock_guard<std::mutex> lk(mtx_);
    return input_reads_;
  }

  bool wait_for(
    const std::function<bool()> & predicate,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(1000)) const
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

private:
  static constexpr std::uint16_t kStatusBase = 768;
  static constexpr std::uint16_t kStatusWords = 9;
  static constexpr std::uint16_t kInputWords = 7;
  static constexpr std::uint16_t kOutputWords = 6;

  static double i32(std::uint16_t high, std::uint16_t low)
  {
    return static_cast<double>(static_cast<std::int32_t>(
             (static_cast<std::uint32_t>(high) << 16) | low));
  }

  static bool read_exact(int fd, std::uint8_t * buf, std::size_t n)
  {
    while (n > 0) {
      const ssize_t got = ::recv(fd, buf, n, 0);
      if (got <= 0) {
        return false;
      }
      buf += got;
      n -= static_cast<std::size_t>(got);
    }
    return true;
  }

  static std::uint16_t be16(const std::uint8_t * p)
  {
    return static_cast<std::uint16_t>((p[0] << 8) | p[1]);
  }

  static void put16(std::vector<std::uint8_t> & out, std::uint16_t v)
  {
    out.push_back(static_cast<std::uint8_t>(v >> 8));
    out.push_back(static_cast<std::uint8_t>(v & 0xFF));
  }

  // Called with mtx_ held, after every output write.
  void follow_control_word()
  {
    const std::uint16_t cw = outputs_[0];
    if (!frozen_ && (cw & kOperatingMode) && (cw & kDriveOn) && (cw & kDriveStart)) {
      position_deg_ = target_deg_locked();
    }
  }

  double target_deg_locked() const {return i32(outputs_[1], outputs_[2]) * 0.0001;}

  // Called with mtx_ held. Mode and ready bits as IndraDrive::enable() waits for them; always
  // in standstill and target reached, since the position jumps to the target.
  std::array<std::uint16_t, kInputWords> inputs_locked() const
  {
    const std::uint16_t cw = outputs_[0];
    std::uint16_t sw = (1u << 3) | (1u << 4);  // standstill, target reached
    if (cw & kOperatingMode) {
      sw |= 0x0002;                                    // operating mode
      sw |= (cw & kDriveOn) ? 0xC000 : 0x8000;         // AF : Ab
    } else {
      sw |= 0x4000;                                    // bb
    }
    if (!((cw & kOperatingMode) && (cw & kDriveOn) && (cw & kDriveStart))) {
      sw |= 1u << 7;  // not following command values
    }
    const auto raw = static_cast<std::uint32_t>(
      static_cast<std::int32_t>(std::lround(position_deg_ / 0.0001)));
    return {sw, static_cast<std::uint16_t>(raw >> 16), static_cast<std::uint16_t>(raw & 0xFFFF),
      0, 0, 0, 0};
  }

  void serve()
  {
    while (!stopping_) {
      const int fd = ::accept(listen_fd_, nullptr, nullptr);
      if (fd < 0) {
        return;
      }
      {
        std::lock_guard<std::mutex> lk(mtx_);
        conn_fd_ = fd;
      }
      serve_connection(fd);
      {
        std::lock_guard<std::mutex> lk(mtx_);
        conn_fd_ = -1;
      }
      ::close(fd);
    }
  }

  void serve_connection(int fd)
  {
    std::array<std::uint8_t, 260> frame{};
    while (!stopping_) {
      if (!read_exact(fd, frame.data(), 7)) {  // MBAP header
        return;
      }
      const std::uint16_t length = be16(&frame[4]);
      if (length < 2 || length > frame.size() - 6 || !read_exact(fd, &frame[7], length - 1)) {
        return;
      }
      const std::uint8_t * pdu = &frame[7];
      const std::uint8_t fc = pdu[0];

      std::vector<std::uint8_t> body;
      {
        std::lock_guard<std::mutex> lk(mtx_);
        reply_locked(fc, pdu, length - 1, body);
      }

      std::vector<std::uint8_t> reply(frame.begin(), frame.begin() + 4);  // transaction, protocol
      put16(reply, static_cast<std::uint16_t>(body.size() + 1));
      reply.push_back(frame[6]);  // unit id
      reply.insert(reply.end(), body.begin(), body.end());
      if (::send(fd, reply.data(), reply.size(), MSG_NOSIGNAL) !=
        static_cast<ssize_t>(reply.size()))
      {
        return;
      }
    }
  }

  void reply_locked(
    std::uint8_t fc, const std::uint8_t * pdu, std::size_t pdu_len,
    std::vector<std::uint8_t> & body)
  {
    auto exception = [&](std::uint8_t code) {
        body = {static_cast<std::uint8_t>(fc | 0x80), code};
      };
    if (pdu_len < 5) {
      exception(0x03);
      return;
    }
    const std::uint16_t addr = be16(pdu + 1);
    const std::uint16_t value = be16(pdu + 3);  // count, or the value for FC6

    if (fc == 0x04 || fc == 0x03) {
      std::vector<std::uint16_t> regs;
      if (fc == 0x04 && addr + value <= kInputWords) {
        const auto in = inputs_locked();
        regs.assign(in.begin() + addr, in.begin() + addr + value);
        ++input_reads_;
      } else if (fc == 0x04 && addr >= kStatusBase && addr + value <= kStatusBase + kStatusWords) {
        // Word 0: gateway "Operate" (bits 1:0 = 11). Byte 2 of the area (high byte of word 1)
        // is the live list for addresses 0-7; profibus_address 2 is bit 2.
        std::array<std::uint16_t, kStatusWords> status{0x0003, 0x0400};
        regs.assign(status.begin() + (addr - kStatusBase),
            status.begin() + (addr - kStatusBase) + value);
      } else if (fc == 0x03 && addr + value <= kOutputWords) {
        regs.assign(outputs_.begin() + addr, outputs_.begin() + addr + value);
      } else {
        exception(0x02);
        return;
      }
      body = {fc, static_cast<std::uint8_t>(regs.size() * 2)};
      for (const auto r : regs) {
        put16(body, r);
      }
    } else if (fc == 0x06) {
      if (addr >= kOutputWords) {
        exception(0x02);
        return;
      }
      outputs_[addr] = value;
      follow_control_word();
      body.assign(pdu, pdu + 5);
    } else if (fc == 0x10) {
      if (pdu_len < 6 || addr + value > kOutputWords || pdu_len < 6u + 2u * value) {
        exception(0x02);
        return;
      }
      for (std::uint16_t i = 0; i < value; ++i) {
        outputs_[addr + i] = be16(pdu + 6 + 2 * i);
      }
      follow_control_word();
      body.assign(pdu, pdu + 5);
    } else {
      exception(0x01);
    }
  }

  int listen_fd_{-1};
  int port_{0};
  std::atomic<bool> stopping_{false};
  std::thread thread_;

  mutable std::mutex mtx_;
  int conn_fd_{-1};
  std::array<std::uint16_t, kOutputWords> outputs_{};
  double position_deg_{0.0};
  bool frozen_{false};
  int input_reads_{0};
};

std::shared_ptr<rclcpp_lifecycle::LifecycleNode> actuator_node(
  const std::string & name, const rclcpp::NodeOptions & options)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(name, options);
  node->declare_parameter<std::string>("driver_address", "127.0.0.1");
  node->declare_parameter<double>("actuator_upright_angle_deg", 12.0);
  node->declare_parameter<double>("upright_position_m", 0.25);
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
