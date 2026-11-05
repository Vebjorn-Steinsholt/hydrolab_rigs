#include <algorithm>
#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "rig_interfaces/action/move_rig.hpp"

#define main bridge_node_main
#include "../src/bridge_node.cpp"
#undef main

namespace
{

using namespace std::chrono_literals;
using GoalHandle = rclcpp_action::ServerGoalHandle<MoveRig>;

// Stands in for the controller's move_wavemaker server, like test/fake_controller.py: a regular
// wave runs until it is cancelled. The test chooses to reject goals, delay the goal response,
// refuse cancels (a stop the controller never confirms) or abort the running goal.
class FakeController
{
public:
  explicit FakeController(const std::string & ns)
  {
    node = std::make_shared<rclcpp::Node>(
      "controller", rclcpp::NodeOptions().arguments({"--ros-args", "-r", "__ns:=" + ns}));
    server = rclcpp_action::create_server<MoveRig>(
      node, "move_wavemaker",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveRig::Goal> goal) {
        std::this_thread::sleep_for(std::chrono::milliseconds(goal_response_delay_ms.load()));
        last_height = goal->height;
        last_period = goal->period;
        ++goals_received;
        return reject_goals ? rclcpp_action::GoalResponse::REJECT :
               rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<GoalHandle>) {
        ++cancels_received;
        return refuse_cancel ? rclcpp_action::CancelResponse::REJECT :
               rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<GoalHandle> handle) {
        std::lock_guard<std::mutex> lock(mutex_);
        threads_.emplace_back([this, handle]() {execute(handle);});
      });
  }

  ~FakeController()
  {
    finish();
  }

  // Ends the running goals (aborted) and joins their threads.
  void finish()
  {
    done_ = true;
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto & thread : threads_) {
      thread.join();
    }
    threads_.clear();
  }

  // Goes away without ending its goals, as a controller that crashed: nothing more reaches the
  // bridge. Remove the node from the executor first.
  void vanish()
  {
    vanish_ = true;
    finish();
    server.reset();
    node.reset();
  }

  std::shared_ptr<rclcpp::Node> node;
  rclcpp_action::Server<MoveRig>::SharedPtr server;
  std::atomic<bool> reject_goals{false};
  std::atomic<bool> refuse_cancel{false};
  std::atomic<bool> abort_now{false};
  std::atomic<int> goal_response_delay_ms{0};
  std::atomic<int> goals_received{0};
  std::atomic<int> cancels_received{0};
  std::atomic<double> last_height{0.0};
  std::atomic<double> last_period{0.0};

private:
  void execute(const std::shared_ptr<GoalHandle> & handle)
  {
    auto result = std::make_shared<MoveRig::Result>();
    while (!done_) {
      if (handle->is_canceling()) {
        result->message = "Goal canceled";
        handle->canceled(result);
        return;
      }
      if (abort_now.exchange(false)) {
        result->message = "fake drive fault";
        handle->abort(result);
        return;
      }
      std::this_thread::sleep_for(10ms);
    }
    if (vanish_) {
      std::lock_guard<std::mutex> lock(orphans_mutex_);
      orphans_.push_back(handle);  // destroyed after the server, so no result is sent
      return;
    }
    result->message = "test ended";
    handle->abort(result);
  }

  std::atomic<bool> done_{false};
  std::atomic<bool> vanish_{false};
  std::mutex orphans_mutex_;
  std::vector<std::shared_ptr<GoalHandle>> orphans_;
  std::mutex mutex_;
  std::vector<std::thread> threads_;
};

// A client of the bridge, as the web server is: publishes the commands and the heartbeat, and
// records every wavemaker_state and wavemaker_message the bridge publishes.
class Client
{
public:
  explicit Client(const std::string & ns)
  {
    node = std::make_shared<rclcpp::Node>(
      "client", rclcpp::NodeOptions().arguments({"--ros-args", "-r", "__ns:=" + ns}));
    height = node->create_publisher<std_msgs::msg::Float64>("height", 10);
    period = node->create_publisher<std_msgs::msg::Float64>("period", 10);
    start = node->create_publisher<std_msgs::msg::Bool>("start", rclcpp::QoS(10).reliable());
    stop = node->create_publisher<std_msgs::msg::Bool>("stop", 10);
    heartbeat = node->create_publisher<std_msgs::msg::Bool>(
      "heartbeat", rclcpp::SensorDataQoS());
    stop_service = node->create_client<std_srvs::srv::Trigger>("stop");
    change_state = node->create_client<lifecycle_msgs::srv::ChangeState>("bridge/change_state");
    const auto status_qos = rclcpp::QoS(10).reliable().transient_local();
    state_subscription = node->create_subscription<std_msgs::msg::String>(
      "wavemaker_state", status_qos, [this](std_msgs::msg::String::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        states_.push_back(msg->data);
      });
    message_subscription = node->create_subscription<std_msgs::msg::String>(
      "wavemaker_message", status_qos, [this](std_msgs::msg::String::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        messages_.push_back(msg->data);
      });
    // 10 Hz, as the web server.
    heartbeat_timer = node->create_wall_timer(100ms, [this]() {
          if (beating) {
            std_msgs::msg::Bool msg;
            msg.data = true;
            heartbeat->publish(msg);
          }
      });
  }

  // Height and period first, then start, as the web server does: they are separate topics.
  void send_wave(double wave_height, double wave_period)
  {
    std_msgs::msg::Float64 value;
    value.data = wave_height;
    height->publish(value);
    value.data = wave_period;
    period->publish(value);
    std::this_thread::sleep_for(200ms);
    send(start);
  }

  void send(const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr & publisher)
  {
    std_msgs::msg::Bool msg;
    msg.data = true;
    publisher->publish(msg);
  }

  std::string state() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return states_.empty() ? "" : states_.back();
  }

  std::string message() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return messages_.empty() ? "" : messages_.back();
  }

  bool saw_state(const std::string & state) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::find(states_.begin(), states_.end(), state) != states_.end();
  }

  bool saw_message(const std::string & message) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::find(messages_.begin(), messages_.end(), message) != messages_.end();
  }

  // Forgets what was recorded, so the next checks see only what follows.
  void clear()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    states_.clear();
    messages_.clear();
  }

  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr height;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr period;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr heartbeat;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_service;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_subscription;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr message_subscription;
  rclcpp::TimerBase::SharedPtr heartbeat_timer;
  std::atomic<bool> beating{true};

private:
  mutable std::mutex mutex_;
  std::vector<std::string> states_;
  std::vector<std::string> messages_;
};

bool wait_until(const std::function<bool()> & condition, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!condition()) {
    if (std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
    std::this_thread::sleep_for(5ms);
  }
  return true;
}

class BridgeNodeTest : public ::testing::Test
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

  // A configured and active bridge, the fake controller and the client in their own namespace,
  // spun by a multi-threaded executor so the bridge keeps running while the fake controller
  // delays its goal response.
  void SetUp() override
  {
    static int count = 0;
    ns = "/test_bridge_" + std::to_string(count++);
    controller = std::make_unique<FakeController>(ns);
    client = std::make_unique<Client>(ns);
    bridge = std::make_shared<BridgeNode>(
      rclcpp::NodeOptions()
      .arguments({"--ros-args", "-r", "__ns:=" + ns})
      .parameter_overrides({rclcpp::Parameter("heartbeat_timeout", 0.5)}));
    ASSERT_EQ(
      bridge->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE).id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(
      bridge->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE).id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    executor.add_node(bridge->get_node_base_interface());
    executor.add_node(controller->node);
    executor.add_node(client->node);
    spinner = std::thread([this]() {executor.spin();});

    // Wait until the bridge sees the controller and the client's commands reach the bridge.
    auto probe = rclcpp_action::create_client<MoveRig>(client->node, "move_wavemaker");
    ASSERT_TRUE(probe->wait_for_action_server(2s));
    ASSERT_TRUE(
      wait_until(
        [this]() {
          return client->start->get_subscription_count() > 0 &&
                 client->stop->get_subscription_count() > 0 &&
                 client->height->get_subscription_count() > 0 &&
                 client->period->get_subscription_count() > 0 &&
                 client->heartbeat->get_subscription_count() > 0;
        }, 2s));
    ASSERT_TRUE(wait_until([this]() {return client->state() == "idle";}, 2s));
    // A few heartbeats, so a start is not refused for want of one.
    std::this_thread::sleep_for(300ms);
  }

  void TearDown() override
  {
    executor.cancel();
    if (spinner.joinable()) {
      spinner.join();
    }
    if (!bridge) {
      return;
    }
    // Deactivate, so the bridge removes its pre-shutdown callback before it is destroyed.
    if (bridge->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      bridge->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    }
    if (controller) {
      controller->finish();
    }
  }

  // Starts a wave and waits until the bridge reports it running.
  void start_running()
  {
    client->send_wave(0.02, 1.0);
    ASSERT_TRUE(
      wait_until(
        [this]() {
          return client->state() == "running" && client->message() == "Wave running";
        }, 2s));
  }

  bool idle_with(const std::string & message, std::chrono::milliseconds timeout = 2s)
  {
    return wait_until(
      [this, &message]() {
        return client->state() == "idle" && client->message() == message;
      }, timeout);
  }

  std::string ns;
  std::unique_ptr<FakeController> controller;
  std::unique_ptr<Client> client;
  std::shared_ptr<BridgeNode> bridge;
  rclcpp::executors::MultiThreadedExecutor executor;
  std::thread spinner;
};

}  // namespace

TEST_F(BridgeNodeTest, StartSendsHeightAndPeriodAndRuns)
{
  client->send_wave(0.03, 1.5);
  ASSERT_TRUE(wait_until([this]() {return client->state() == "running";}, 2s));
  EXPECT_TRUE(client->saw_state("starting"));
  EXPECT_TRUE(client->saw_message("Starting wave: height 0.0300 m, period 1.500 s"));
  EXPECT_EQ(controller->goals_received, 1);
  EXPECT_DOUBLE_EQ(controller->last_height, 0.03);
  EXPECT_DOUBLE_EQ(controller->last_period, 1.5);
}

TEST_F(BridgeNodeTest, StartRefusedWithoutHeightAndPeriod)
{
  client->send(client->start);
  ASSERT_TRUE(
    idle_with("start refused: height and period must be positive and finite"));
  EXPECT_EQ(controller->goals_received, 0);
}

TEST_F(BridgeNodeTest, StartRefusedWithoutHeartbeat)
{
  client->beating = false;
  std::this_thread::sleep_for(700ms);  // longer than heartbeat_timeout
  client->send_wave(0.02, 1.0);
  ASSERT_TRUE(idle_with("start refused: no client heartbeat (LabVIEW or web page)"));
  EXPECT_EQ(controller->goals_received, 0);
}

TEST_F(BridgeNodeTest, StartRefusedWhileRunning)
{
  start_running();
  client->send(client->start);
  ASSERT_TRUE(
    wait_until(
      [this]() {
        return client->message() == "start refused: wave is running; stop it first";
      }, 2s));
  EXPECT_EQ(client->state(), "running");
  EXPECT_EQ(controller->goals_received, 1);
}

TEST_F(BridgeNodeTest, StartRefusedWithoutActionServer)
{
  executor.remove_node(controller->node);
  controller->vanish();
  auto probe = rclcpp_action::create_client<MoveRig>(client->node, "move_wavemaker");
  ASSERT_TRUE(wait_until([&probe]() {return !probe->action_server_is_ready();}, 2s));
  client->send_wave(0.02, 1.0);
  ASSERT_TRUE(idle_with("start refused: action server not available"));
}

TEST_F(BridgeNodeTest, StopTopicCancelsTheWave)
{
  start_running();
  client->send(client->stop);
  ASSERT_TRUE(idle_with("Stopped, holding position."));
  EXPECT_TRUE(client->saw_state("stopping"));
  EXPECT_TRUE(client->saw_message("Operator pressed stop"));
  EXPECT_EQ(controller->cancels_received, 1);

  // Idle again, so the next start runs.
  client->clear();
  start_running();
  EXPECT_EQ(controller->goals_received, 2);
}

TEST_F(BridgeNodeTest, StopServiceRepliesWhenTheWaveHasStopped)
{
  start_running();
  ASSERT_TRUE(client->stop_service->wait_for_service(1s));
  auto future = client->stop_service->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(future.wait_for(2s), std::future_status::ready);
  const auto response = future.get();
  EXPECT_TRUE(response->success);
  EXPECT_EQ(response->message, "Stopped, holding position.");
  EXPECT_EQ(client->state(), "idle");
  EXPECT_TRUE(client->saw_message("Stopped by stop service"));
}

TEST_F(BridgeNodeTest, StopServiceWithNothingRunning)
{
  ASSERT_TRUE(client->stop_service->wait_for_service(1s));
  auto future = client->stop_service->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(future.wait_for(2s), std::future_status::ready);
  const auto response = future.get();
  EXPECT_TRUE(response->success);
  EXPECT_EQ(response->message, "Nothing running");
  EXPECT_EQ(client->state(), "idle");
}

TEST_F(BridgeNodeTest, StopDuringStartCancelsOnAccept)
{
  controller->goal_response_delay_ms = 500;
  client->send_wave(0.02, 1.0);
  ASSERT_TRUE(wait_until([this]() {return client->state() == "starting";}, 2s));
  client->send(client->stop);
  ASSERT_TRUE(wait_until([this]() {return client->state() == "stopping";}, 1s));
  ASSERT_TRUE(idle_with("Stopped, holding position."));
  EXPECT_FALSE(client->saw_state("running"));
  EXPECT_GE(controller->cancels_received, 1);
}

TEST_F(BridgeNodeTest, LostHeartbeatStopsTheWave)
{
  start_running();
  client->beating = false;
  ASSERT_TRUE(idle_with("Stopped, holding position."));
  EXPECT_TRUE(client->saw_state("stopping"));
  EXPECT_TRUE(client->saw_message("Stopped: client heartbeat (LabVIEW or web page) lost"));
  EXPECT_EQ(controller->cancels_received, 1);
}

TEST_F(BridgeNodeTest, ControllerRejectionReturnsToIdle)
{
  controller->reject_goals = true;
  client->send_wave(0.02, 1.0);
  ASSERT_TRUE(idle_with("Goal rejected by controller (see its log)"));
  EXPECT_FALSE(client->saw_state("running"));

  // A new start is possible.
  controller->reject_goals = false;
  client->clear();
  start_running();
}

TEST_F(BridgeNodeTest, ControllerAbortReturnsToIdleWithItsMessage)
{
  start_running();
  controller->abort_now = true;
  ASSERT_TRUE(idle_with("Wave aborted: fake drive fault"));
}

TEST_F(BridgeNodeTest, DeactivationCancelsTheWaveAndStaysInactive)
{
  start_running();
  ASSERT_TRUE(client->change_state->wait_for_service(1s));
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
  auto future = client->change_state->async_send_request(request);
  ASSERT_EQ(future.wait_for(2s), std::future_status::ready);
  EXPECT_TRUE(future.get()->success);
  EXPECT_TRUE(wait_until([this]() {return controller->cancels_received == 1;}, 1s));
  // The controller's result arrives after the deactivation; the bridge must stay inactive.
  ASSERT_TRUE(
    wait_until(
      [this]() {
        return client->state() == "inactive" &&
               client->message() == "Stopped, holding position.";
      }, 2s));
  EXPECT_TRUE(client->saw_message("Bridge is deactivating"));
}

TEST_F(BridgeNodeTest, UnconfirmedStopGivesUpAfterTimeout)
{
  controller->refuse_cancel = true;
  start_running();
  const auto stopped = std::chrono::steady_clock::now();
  client->send(client->stop);
  ASSERT_TRUE(wait_until([this]() {return client->state() == "stopping";}, 1s));
  ASSERT_TRUE(
    idle_with(
      "No result from the controller; stop not confirmed by controller, verify", 4s));
  // The bridge waits for its 2 s stop timeout.
  EXPECT_GE(std::chrono::steady_clock::now() - stopped, 1900ms);
  EXPECT_EQ(controller->cancels_received, 1);
}

TEST_F(BridgeNodeTest, UnconfirmedStopServiceRepliesFailure)
{
  controller->refuse_cancel = true;
  start_running();
  ASSERT_TRUE(client->stop_service->wait_for_service(1s));
  auto future = client->stop_service->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(future.wait_for(4s), std::future_status::ready);
  const auto response = future.get();
  EXPECT_FALSE(response->success);
  EXPECT_EQ(
    response->message,
    "No result from the controller; stop not confirmed by controller, verify");
  EXPECT_EQ(client->state(), "idle");
}

TEST_F(BridgeNodeTest, StopWithActionServerGoneGivesUpAtOnce)
{
  start_running();
  executor.remove_node(controller->node);
  controller->vanish();
  auto probe = rclcpp_action::create_client<MoveRig>(client->node, "move_wavemaker");
  ASSERT_TRUE(wait_until([&probe]() {return !probe->action_server_is_ready();}, 2s));
  client->send(client->stop);
  ASSERT_TRUE(
    idle_with(
      "move_wavemaker action is not available; stop not confirmed by controller, verify", 1s));
  EXPECT_TRUE(client->saw_state("stopping"));

  // The bridge does not wait for the lost goal: a later start is refused for the missing
  // server, not because a wave is still running.
  client->send_wave(0.02, 1.0);
  ASSERT_TRUE(idle_with("start refused: action server not available"));
}
