#include <rmw/types.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "bondcpp/bond.hpp"
#include "rig_interfaces/action/move_rig.hpp"
#include "rig_interfaces/srv/cancel_all_goals.hpp"
#include "rig_interfaces/srv/return_to_upright.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "indradrive_actuator.hpp"
#include "linmot_actuator.hpp"
#include "wave_math.hpp"
#include "wave_trajectory.hpp"
#include "drive_scale.hpp"

using MoveRig = rig_interfaces::action::MoveRig;
using MoveRigGoalHandle = rclcpp_action::ServerGoalHandle<MoveRig>;

using CancelAllGoals = rig_interfaces::srv::CancelAllGoals;
using ReturnToUpright = rig_interfaces::srv::ReturnToUpright;

using rig_controller::Blend;
using rig_controller::interpolate_gain;
using rig_controller::kQuinticPeakAccelerationFactor;
using rig_controller::kQuinticPeakVelocityFactor;
using rig_controller::quintic_blend;

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace
{

// The actuator for drive_name: "indradrive" (MGate gateway, the wavemakers) or "linmot"
// (Arduino serial-to-CAN bridge, the oscillator). Throws on anything else, which fails configure.
std::unique_ptr<rig_controller::Actuator> make_actuator(rclcpp_lifecycle::LifecycleNode & node)
{
  const std::string name = node.get_parameter("drive_name").as_string();
  if (name == "indradrive") {
    return std::make_unique<rig_controller::IndraDriveActuator>(node);
  }
  if (name == "linmot") {
    return std::make_unique<rig_controller::LinMotActuator>(node);
  }
  if (name.empty()) {
    throw std::invalid_argument(
        "drive_name must be set to 'indradrive' or 'linmot, or implemented as new drives in the future.");
  }
  throw std::invalid_argument(
          "drive_name must be 'indradrive' or 'linmot', not '" + name + "'");
}

}  // namespace

class ControllerNode final : public rclcpp_lifecycle::LifecycleNode
{
public:
  using ActuatorFactory = std::function<std::unique_ptr<rig_controller::Actuator>(
        rclcpp_lifecycle::LifecycleNode &)>;

  explicit ControllerNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    ActuatorFactory actuator_factory = make_actuator)
  : LifecycleNode("controller", options), actuator_factory_(std::move(actuator_factory))
  {
    // Must match the lifecycle manager's bond_timeout; set by the launch file.
    declare_parameter<double>("bond_timeout", 4.0);
    declare_parameter<double>("soft_stop_timeout_s", 0.2);
    // Required, no default: every rig's config names its drive (make_actuator()).
    declare_parameter<std::string>("drive_name", "");
    // "wavemaker" or "oscillator": names the action and topics (move_<rig>, <rig>_setpoint, ...).
    declare_parameter<std::string>("rig", "wavemaker");
    declare_parameter<std::string>("wavemaker_type", "piston");
    declare_parameter<std::string>("driver_address", "");
    declare_parameter<double>("upright_position_m", 0.0);
    declare_parameter<bool>("upright_is_minimum", false);
    declare_parameter<double>("water_depth", 0.6);
    declare_parameter<double>("wavemaker_transfer_gain", 1.0);
    // Transfer gain per wave period, interpolated; replaces wavemaker_transfer_gain when set.
    declare_parameter<std::vector<double>>("wavemaker_transfer_gain_periods",
      std::vector<double>{});
    declare_parameter<std::vector<double>>("wavemaker_transfer_gain_values", std::vector<double>{});

    declare_parameter<double>("flap_attachment_height", 0.0);
    declare_parameter<double>("hinge_height", 0.0);
    declare_parameter<double>("actuator_upright_angle_deg", 0.0);
    declare_parameter<std::string>("drive_motion", "linear");
    if (get_parameter("drive_motion").as_string() == "linear") {
      declare_parameter<double>("travel_minimum_m", 0.0);
      declare_parameter<double>("travel_maximum_m", 1.0);
    }
    declare_parameter<double>("actuator_lead_m_per_degree", 0.0);
    declare_parameter<double>("min_position_deg", -550.0);
    declare_parameter<double>("max_position_deg", 500.0);
    declare_parameter<double>("min_position_m", 0.0);
    declare_parameter<double>("max_position_m", 1.0);
    declare_parameter<double>("return_to_upright_max_velocity_mps", 0.1);
    declare_parameter<double>("return_to_upright_min_duration_s", 1.0);
    declare_parameter<double>("return_to_upright_default_tolerance_m", 0.002);
    declare_parameter<double>("return_to_upright_timeout_margin_s", 2.0);
    // The drive gets the motion's peak speed times this as its positioning-speed limit.
    declare_parameter<double>("positioning_velocity_margin", 1.2);
    declare_parameter<double>("positioning_acceleration_margin", 1.2);
    declare_parameter<double>("max_goal_acceleration_mps2", 0.0);
    declare_parameter<double>("min_period_s", 0.0);

    // Ctrl-C shuts the context down before the destructor runs. Stop the drive and drop the
    // bond while the context is still valid: a bond left alive tries to create a timer once the
    // lifecycle manager breaks it, which throws and aborts the process before the destructor.
    pre_shutdown_callback_handle_ =
      get_node_base_interface()->get_context()->add_pre_shutdown_callback(
      [this]() {
        RCLCPP_INFO(get_logger(), "Context shutting down: stopping the drive");
        stop_activity();
      });
  }

  ~ControllerNode() override
  {
    get_node_base_interface()->get_context()->remove_pre_shutdown_callback(
      pre_shutdown_callback_handle_);
    abort_active_goal();
    stop_execution();
    // Ctrl-C skips the lifecycle transitions, so this is the last chance to stop the drive.
    try {
      if (actuator_) {
        actuator_->stop();
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Exception caught while stopping actuator: %s", error.what());
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "Unknown exception caught while stopping actuator");
    }
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override
  {
    rig_ = get_parameter("rig").as_string();
    if (rig_ != "wavemaker" && rig_ != "oscillator") {
      RCLCPP_ERROR(get_logger(), "rig must be 'wavemaker' or 'oscillator'");
      return CallbackReturn::FAILURE;
    }
    // Renamed on 2026-10-22. ROS 2 ignores parameters a node does not declare, so an old name in a
    // yaml file or on the command line would silently leave the default (a 0-1 m travel limit).
    const std::map<std::string, std::string> renamed{
      {"actuator_type", "drive_name"},
      {"actuator_drive_type", "drive_motion"},
      {"wavemaker_upright_position_m", "upright_position_m"},
      {"wavemaker_upright_is_minimum", "upright_is_minimum"},
      {"wavemaker_minimum", "travel_minimum_m"},
      {"wavemaker_maximum", "travel_maximum_m"},
      {"wavemaker_mode_pregenerated", ""},  // removed 2026-10-30: the goal picks its type
      {"mode_pregenerated", ""},            // removed 2026-10-30: the goal picks its type
      {"wavemaker_id", ""},  // removed, it was not used
    };
    const auto & overrides = get_node_parameters_interface()->get_parameter_overrides();
    for (const auto & [old_name, new_name] : renamed) {
      if (overrides.count(old_name) == 0) {
        continue;
      }
      if (new_name.empty()) {
        RCLCPP_ERROR(get_logger(), "Parameter %s was removed; delete it", old_name.c_str());
      } else {
        RCLCPP_ERROR(
          get_logger(), "Parameter %s was renamed to %s", old_name.c_str(), new_name.c_str());
      }
      return CallbackReturn::FAILURE;
    }
    wavemaker_type_ = get_parameter("wavemaker_type").as_string();
    driver_address_ = get_parameter("driver_address").as_string();
    wavemaker_position_offset_ = get_parameter("upright_position_m").as_double();
    upright_is_minimum_ = get_parameter("upright_is_minimum").as_bool();
    water_depth_ = get_parameter("water_depth").as_double();
    wavemaker_transfer_gain_ = get_parameter("wavemaker_transfer_gain").as_double();
    gain_periods_ = get_parameter("wavemaker_transfer_gain_periods").as_double_array();
    gain_values_ = get_parameter("wavemaker_transfer_gain_values").as_double_array();
    flap_attachment_height_ = get_parameter("flap_attachment_height").as_double();
    hinge_height_ = get_parameter("hinge_height").as_double();


    if (!std::isfinite(wavemaker_position_offset_) || !std::isfinite(water_depth_) ||
      !std::isfinite(flap_attachment_height_))
    {
      RCLCPP_ERROR(get_logger(), "All numeric parameters must be finite");
      return CallbackReturn::FAILURE;
    }

    try {
      drive_scale_ = rig_controller::DriveScale::from_parameters(*this);
    } catch (const std::invalid_argument & error) {
      RCLCPP_ERROR(get_logger(), "%s", error.what());
      return CallbackReturn::FAILURE;
    }

    if (drive_scale_.angular) {
      const double minimum_deg = get_parameter("min_position_deg").as_double();
      const double maximum_deg = get_parameter("max_position_deg").as_double();
      if (!std::isfinite(minimum_deg) || !std::isfinite(maximum_deg) ||
        minimum_deg >= maximum_deg || drive_scale_.upright_drive < minimum_deg ||
        drive_scale_.upright_drive > maximum_deg)
      {
        RCLCPP_ERROR(get_logger(), "Invalid angular actuator limits");
        return CallbackReturn::FAILURE;
      }
      goal_position_minimum_ = drive_scale_.to_m(minimum_deg);
      goal_position_maximum_ = drive_scale_.to_m(maximum_deg);
    } else {
      travel_minimum_ = get_parameter("travel_minimum_m").as_double();
      travel_maximum_ = get_parameter("travel_maximum_m").as_double();
      if (!std::isfinite(travel_minimum_) || !std::isfinite(travel_maximum_) ||
        travel_minimum_ >= travel_maximum_)
      {
        RCLCPP_ERROR(
          get_logger(), "travel_minimum_m must be less than travel_maximum_m");
        return CallbackReturn::FAILURE;
      }
      if (wavemaker_position_offset_ < travel_minimum_ ||
        wavemaker_position_offset_ > travel_maximum_)
      {
        RCLCPP_ERROR(
          get_logger(),
          "upright_position_m must be within travel_minimum_m and travel_maximum_m");
        return CallbackReturn::FAILURE;
      }
      goal_position_minimum_ = travel_minimum_;
      goal_position_maximum_ = travel_maximum_;
    }

    if (rig_ == "wavemaker") {
      if (wavemaker_type_ == "flap" &&
        (flap_attachment_height_ <= 0.0 || flap_attachment_height_ < water_depth_))
      {
        RCLCPP_ERROR(get_logger(),
          "flap_attachment_height must be set (> 0) and greater than water_depth "
          "for flap-type wavemakers");
        return CallbackReturn::FAILURE;
      }
      if (wavemaker_type_ == "flap" &&
        !(hinge_height_ >= 0.0 && hinge_height_ < water_depth_))
      {
        RCLCPP_ERROR(get_logger(), "hinge_height must be >= 0 and below water_depth");
        return CallbackReturn::FAILURE;
      }
      if (wavemaker_type_ != "flap" && wavemaker_type_ != "piston") {
        RCLCPP_ERROR(
          get_logger(), "wavemaker_type must be either 'flap' or 'piston'");
        return CallbackReturn::FAILURE;
      }
      if (gain_periods_.size() != gain_values_.size()) {
        RCLCPP_ERROR(
          get_logger(),
          "wavemaker_transfer_gain_periods and wavemaker_transfer_gain_values must have the same "
          "length");
        return CallbackReturn::FAILURE;
      }
      if (gain_periods_.size() == 1) {
        RCLCPP_ERROR(
          get_logger(), "wavemaker_transfer_gain_periods/values need at least 2 points when set");
        return CallbackReturn::FAILURE;
      }
      const auto not_finite_positive = [](double v) {return !std::isfinite(v) || v <= 0.0;};
      if (std::any_of(gain_periods_.begin(), gain_periods_.end(), not_finite_positive)) {
        RCLCPP_ERROR(get_logger(), "All wavemaker_transfer_gain_periods must be finite and > 0");
        return CallbackReturn::FAILURE;
      }
      if (std::any_of(gain_values_.begin(), gain_values_.end(), not_finite_positive)) {
        RCLCPP_ERROR(get_logger(), "All wavemaker_transfer_gain_values must be finite and > 0");
        return CallbackReturn::FAILURE;
      }
      if (std::adjacent_find(
          gain_periods_.begin(), gain_periods_.end(),
          [](double a, double b) {return a >= b;}) != gain_periods_.end())
      {
        RCLCPP_ERROR(get_logger(), "wavemaker_transfer_gain_periods must be strictly ascending");
        return CallbackReturn::FAILURE;
      }

      if (water_depth_ <= 0.0) {
        RCLCPP_ERROR(get_logger(), "water_depth must be set (> 0)");
        return CallbackReturn::FAILURE;
      }
      if (wavemaker_transfer_gain_ <= 0.0 || !std::isfinite(wavemaker_transfer_gain_)) {
        RCLCPP_ERROR(get_logger(), "wavemaker_transfer_gain must be finite and > 0");
        return CallbackReturn::FAILURE;
      }
    } else if (upright_is_minimum_) {
      RCLCPP_ERROR(
        get_logger(), "An oscillator moves around upright: upright_is_minimum must be false");
      return CallbackReturn::FAILURE;
    }
    if (upright_is_minimum_ &&
      std::abs(wavemaker_position_offset_ - goal_position_minimum_) > 1e-9)
    {
      RCLCPP_ERROR(
        get_logger(), "upright_position_m must equal the minimum when "
        "upright_is_minimum is true");
      return CallbackReturn::FAILURE;
    }

    return_max_velocity_mps_ = get_parameter("return_to_upright_max_velocity_mps").as_double();
    return_min_duration_s_ = get_parameter("return_to_upright_min_duration_s").as_double();
    return_default_tolerance_m_ =
      get_parameter("return_to_upright_default_tolerance_m").as_double();
    return_timeout_margin_s_ = get_parameter("return_to_upright_timeout_margin_s").as_double();
    positioning_velocity_margin_ = get_parameter("positioning_velocity_margin").as_double();
    if (!std::isfinite(positioning_velocity_margin_) || positioning_velocity_margin_ < 1.0) {
      RCLCPP_ERROR(get_logger(), "positioning_velocity_margin must be finite and >= 1");
      return CallbackReturn::FAILURE;
    }
    positioning_acceleration_margin_ =
      get_parameter("positioning_acceleration_margin").as_double();
    if (!std::isfinite(positioning_acceleration_margin_) ||
      positioning_acceleration_margin_ < 1.0)
    {
      RCLCPP_ERROR(get_logger(), "positioning_acceleration_margin must be finite and >= 1");
      return CallbackReturn::FAILURE;
    }
    max_goal_acceleration_mps2_ = get_parameter("max_goal_acceleration_mps2").as_double();
    min_period_s_ = get_parameter("min_period_s").as_double();
    if (!std::isfinite(max_goal_acceleration_mps2_) || max_goal_acceleration_mps2_ < 0.0 ||
      !std::isfinite(min_period_s_) || min_period_s_ < 0.0)
    {
      RCLCPP_ERROR(get_logger(),
        "max_goal_acceleration_mps2 and min_period_s must be finite and >= 0");
      return CallbackReturn::FAILURE;
    }
    for (const double value : {
      return_max_velocity_mps_, return_min_duration_s_, return_default_tolerance_m_,
      return_timeout_margin_s_})
    {
      if (!std::isfinite(value) || value <= 0.0) {
        RCLCPP_ERROR(get_logger(), "All return_to_upright_* parameters must be finite and > 0");
        return CallbackReturn::FAILURE;
      }
    }
    soft_stop_configured_ns_ = steady_now_ns();
    soft_stop_missing_logged_ = false;
    try {
      actuator_ = actuator_factory_(*this);
      actuator_->set_fault_callback(
        [this](const std::string & reason) {handle_driver_fault(reason);});
    } catch (const std::exception & error) {
      stop_activity();
      release_resources();
      RCLCPP_ERROR(get_logger(), "Failed to configure actuator: %s", error.what());
      return CallbackReturn::FAILURE;
    }
    // Run the control loops at the rate setpoints reach the drive.
    control_rate_hz_ = 1000.0 / std::max(1, actuator_->update_period_ms());
    if (return_max_velocity_mps_ > actuator_->max_velocity_mps()) {
      RCLCPP_ERROR(
        get_logger(),
        "return_to_upright_max_velocity_mps (%f) exceeds the drive's maximum speed (%f m/s)",
        return_max_velocity_mps_, actuator_->max_velocity_mps());
      release_resources();
      return CallbackReturn::FAILURE;
    }
    action_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    action_server_ = rclcpp_action::create_server<MoveRig>(
      shared_from_this(),
      "move_" + rig_,
      std::bind(&ControllerNode::goal_callback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ControllerNode::cancel_callback, this, std::placeholders::_1),
      std::bind(&ControllerNode::handle_accepted_callback, this, std::placeholders::_1),
      rcl_action_server_get_default_options(),
      action_callback_group_);

    cancel_service_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cancel_client_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    return_to_upright_service_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    cancel_client_ = rclcpp_action::create_client<MoveRig>(
      shared_from_this(), "move_" + rig_, cancel_client_callback_group_);

    cancel_service_ = create_service<CancelAllGoals>(
      "cancel_all_goals",
      [this](
        std::shared_ptr<rclcpp::Service<CancelAllGoals>> service,
        std::shared_ptr<rmw_request_id_t> request_id,
        std::shared_ptr<CancelAllGoals::Request> request)
      {
        auto respond =
        [service, request_id, requester = request->requester](
          uint8_t status, uint32_t count, const std::string & message)
        {
          auto response = std::make_shared<CancelAllGoals::Response>();
          response->status = status;
          response->goals_canceling = count;
          response->requester = requester;
          response->message = message;
          service->send_response(*request_id, *response);
        };

        // Our own action server; discovery can lag right after configure.
        if (!cancel_client_ ||
        !cancel_client_->wait_for_action_server(std::chrono::seconds(1)))
        {
          respond(
            CancelAllGoals::Response::SERVER_UNAVAILABLE,
            0, "Action server is unavailable");
          return;
        }

        cancel_client_->async_cancel_all_goals(
          [respond](auto cancel_reply)
          {
            const auto count =
            static_cast<uint32_t>(cancel_reply->goals_canceling.size());

            if (cancel_reply->return_code !=
            action_msgs::srv::CancelGoal::Response::ERROR_NONE)
            {
              respond(
                CancelAllGoals::Response::REQUEST_REJECTED,
                count, "Action server rejected cancellation");
            } else if (count == 0) {
              respond(
                CancelAllGoals::Response::NO_ACTIVE_GOALS,
                0, "No active goals");
            } else {
              respond(
                CancelAllGoals::Response::REQUEST_ACCEPTED,
                count, "Cancellation accepted");
            }
          });
      },
      rclcpp::ServicesQoS(),
      cancel_service_callback_group_);

    return_to_upright_service_ = create_service<ReturnToUpright>(
      "return_to_upright",
      [this](
        std::shared_ptr<rclcpp::Service<ReturnToUpright>> service,
        std::shared_ptr<rmw_request_id_t> request_id,
        std::shared_ptr<ReturnToUpright::Request> request)
      {
        auto respond = [service, request_id](
          uint8_t status, double position, const std::string & message)
        {
          ReturnToUpright::Response response;
          response.status = status;
          response.final_position = position;
          response.message = message;
          service->send_response(*request_id, response);
        };
        const double not_a_number = std::numeric_limits<double>::quiet_NaN();

        double tolerance = request->tolerance;
        if (tolerance == 0.0) {
          tolerance = return_default_tolerance_m_;
        }
        bool reject = true;
        uint8_t reject_status = ReturnToUpright::Response::CONTROLLER_INACTIVE;
        std::string reject_message;
        {
          std::lock_guard<std::mutex> lock(goal_mutex_);
          if (!accepting_goals_) {
            reject_status = ReturnToUpright::Response::CONTROLLER_INACTIVE;
            reject_message = "Controller not active or not accepting goals";
          } else if (soft_stop_active()) {
            reject_status = ReturnToUpright::Response::CONTROLLER_INACTIVE;
            reject_message = "Controller soft stopped";
          } else if (!std::isfinite(tolerance) || tolerance < 0.0) {
            reject_status = ReturnToUpright::Response::POSITION_INVALID;
            reject_message = "invalid tolerance";
          } else if (goal_pending_ || returning_to_upright_ || goal_handle_) {
            reject_status = ReturnToUpright::Response::BUSY;
            reject_message = "Controller is busy";
          } else if (actuator_->faulted()) {
            reject_status = ReturnToUpright::Response::ACTUATOR_FAULT;
            reject_message = actuator_->fault_reason();
          } else if (!actuator_->is_live()) {
            reject_status = ReturnToUpright::Response::ACTUATOR_NOT_READY;
            reject_message = "Actuator not ready";
          } else {
            // All checks passed: claim the controller before anyone else can.
            returning_to_upright_ = true;
            reject = false;
          }
        }
        if (reject) {
          respond(reject_status, not_a_number, reject_message);
          return;
        }
        const bool started = start_execution([this, respond, tolerance, not_a_number]() {
          double final_position = not_a_number;
          std::string failure;
          ReturnStatus status = ReturnStatus::Fault;
          try {
            status = move_to_upright(tolerance, final_position);
          } catch (const std::exception & error) {
            failure = error.what();
            RCLCPP_ERROR(get_logger(), "Return to upright failed: %s", error.what());
            try {
              actuator_->halt();
            } catch (...) {
                // best effort: the drive may already be unreachable
            }
          } catch (...) {
            failure = "unknown exception";
            RCLCPP_ERROR(get_logger(), "Return to upright failed with an unknown exception");
            try {
              actuator_->halt();
            } catch (...) {
                // best effort: the drive may already be unreachable
            }
          }

          {
            std::lock_guard<std::mutex> lock(goal_mutex_);
            returning_to_upright_ = false;
          }

          try {
            switch (status) {
              case ReturnStatus::Success:
                respond(
                    ReturnToUpright::Response::SUCCESS, final_position,
                    "At upright");
                break;
              case ReturnStatus::Timeout:
                respond(
                    ReturnToUpright::Response::TIMEOUT, final_position,
                    "Timed out before reaching upright");
                break;
              case ReturnStatus::PositionInvalid:
                respond(
                    ReturnToUpright::Response::POSITION_INVALID, final_position,
                    "Current position is invalid");
                break;
              case ReturnStatus::Fault:
                respond(
                    ReturnToUpright::Response::ACTUATOR_FAULT, final_position,
                    failure.empty() ? actuator_->fault_reason() :
                    "Return to upright failed: " + failure);
                break;
              case ReturnStatus::Stopped:
                respond(
                    ReturnToUpright::Response::CONTROLLER_INACTIVE, final_position,
                    "Interrupted by lifecycle transition");
                break;
              case ReturnStatus::SoftStop:
                respond(
                    ReturnToUpright::Response::SOFT_STOP, final_position,
                    "Soft stop engaged; actuator halted and remains enabled");
                break;
            }
          } catch (const std::exception & error) {
            RCLCPP_WARN(
                get_logger(), "Could not send return_to_upright reply: %s", error.what());
          } catch (...) {
            RCLCPP_WARN(
                get_logger(), "Could not send return_to_upright reply: unknown exception");
          }
          });
        if (!started) {
          {
            std::lock_guard<std::mutex> lock(goal_mutex_);
            returning_to_upright_ = false;
          }
          respond(
            ReturnToUpright::Response::CONTROLLER_INACTIVE, not_a_number,
            "Controller not active or not accepting goals");
        }
      },
      rclcpp::ServicesQoS(),
      return_to_upright_service_callback_group_);

    // Paddle setpoint sent to the drive (m), its velocity (m/s) and the measured position (m),
    // published every control cycle during goals and returns.
    setpoint_publisher_ = create_publisher<std_msgs::msg::Float64>(rig_name("setpoint"), 10);
    velocity_publisher_ = create_publisher<std_msgs::msg::Float64>(rig_name("velocity"), 10);
    position_publisher_ = create_publisher<std_msgs::msg::Float64>(rig_name("position"), 10);
    // Debug: the setpoint position as sent to the drive, in actuator units (degrees for an
    // angular drive), published with the others.
    actuator_setpoint_publisher_ =
      create_publisher<std_msgs::msg::Float64>(rig_name("actuator_setpoint"), 10);
    soft_stop_timeout_ns_ = static_cast<int64_t>(
      get_parameter("soft_stop_timeout_s").as_double() * 1e9
    );
    soft_stopped_.store(true);
    soft_stop_last_msg_ns_.store(0);
    soft_stop_callback_group_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options;
    options.callback_group = soft_stop_callback_group_;

    soft_stop_subscription_ = create_subscription<std_msgs::msg::Bool>(
        "soft_stop", rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Bool & msg) {
        if (soft_stop_last_msg_ns_.exchange(steady_now_ns()) == 0) {
          RCLCPP_INFO(get_logger(), "Soft stop connected");
        }
        const bool was_stopped = soft_stopped_.exchange(msg.data);
        if (msg.data && !was_stopped) {
          RCLCPP_WARN(
                get_logger(), "Soft stop engaged: stopping the motion");
        } else if (!msg.data && was_stopped) {
          RCLCPP_WARN(
                get_logger(), "Soft stop released: new motion can start");
        }
        });
    soft_stop_silent_logged_ = false;
    RCLCPP_INFO(
      get_logger(), "Waiting for soft_stop; goals are rejected until soft stop is released");
    fault_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() {
        process_driver_fault();
        check_soft_stop_liveness();
      });

    const std::string what =
      rig_ == "oscillator" ? std::string("an oscillator") : "a " + wavemaker_type_ + " wavemaker";

    RCLCPP_INFO(
      get_logger(), "Configuring from state '%s' as %s, control loop at %.1f Hz",
      previous_state.label().c_str(), what.c_str(), control_rate_hz_);

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override
  {
    RCLCPP_INFO(
      get_logger(), "Activating from state '%s'", previous_state.label().c_str());

    if (!actuator_ || !actuator_->start(get_logger())) {
      return CallbackReturn::FAILURE;
    }
    try {
      bond_ = std::make_unique<bond::Bond>(
        "bond", get_name(), shared_from_this());
      bond_->setHeartbeatPeriod(0.10);
      bond_->setHeartbeatTimeout(get_parameter("bond_timeout").as_double());
      bond_->start();
    } catch (const std::exception & e) {
      // Back to inactive with the drive stopped; the node stays configured.
      RCLCPP_ERROR(get_logger(), "Exception caught during bond creation: %s", e.what());
      bond_.reset();
      actuator_->stop();
      return CallbackReturn::FAILURE;
    }

    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      accepting_goals_ = true;
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override
  {
    RCLCPP_INFO(
      get_logger(), "Deactivating from state '%s'", previous_state.label().c_str());

    stop_activity();
    if (fault_transition_pending_.exchange(false)) {
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override
  {
    RCLCPP_INFO(
      get_logger(), "Cleaning up from state '%s'", previous_state.label().c_str());

    stop_activity();
    release_resources();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override
  {
    RCLCPP_ERROR(
      get_logger(), "Error occurred in state '%s'", previous_state.label().c_str());

    stop_activity();
    release_resources();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override
  {
    RCLCPP_INFO(
      get_logger(), "Shutting down from state '%s'", previous_state.label().c_str());

    stop_activity();
    release_resources();
    return CallbackReturn::SUCCESS;
  }

private:
  enum class ReturnStatus { Success, Timeout, Fault, PositionInvalid, Stopped, SoftStop };

  void check_soft_stop_liveness()
  {
    const int64_t last_msg_ns = soft_stop_last_msg_ns_.load();
    if (last_msg_ns == 0) {
      if (!soft_stop_missing_logged_ &&
        steady_now_ns() - soft_stop_configured_ns_ > 5'000'000'000)
      {
        RCLCPP_ERROR(
          get_logger(),
          "Soft stop has not published within 5 s of configure; is the node running?");
        soft_stop_missing_logged_ = true;

      }
      return;
    }
    const bool silent = steady_now_ns() - last_msg_ns > soft_stop_timeout_ns_;
    if (silent && !soft_stop_silent_logged_) {
      RCLCPP_ERROR(
        get_logger(), "No Soft stop message received for over %.0f ms; treating as stopped",
        soft_stop_timeout_ns_ / 1e6);
    }
    soft_stop_silent_logged_ = silent;
  }

  // A topic name for this rig: "<rig>_<suffix>", e.g. wavemaker_setpoint or oscillator_setpoint.
  std::string rig_name(const std::string & suffix) const
  {
    return rig_ + "_" + suffix;
  }

  static int64_t steady_now_ns()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  bool soft_stop_active() const
  {
    return soft_stopped_.load() ||
           steady_now_ns() - soft_stop_last_msg_ns_.load() > soft_stop_timeout_ns_;
  }

  // Stops goals, the return worker, the drive and the bond. Used by every transition
  // that leaves the active state.
  void stop_activity()
  {
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      accepting_goals_ = false;
    }
    abort_active_goal();
    stop_execution();
    if (actuator_) {
      actuator_->stop();
    }
    if (bond_) {
      bond_->breakBond();
      bond_.reset();
    }
  }

  // Frees everything on_configure created; the node is unconfigured afterwards.
  void release_resources()
  {
    action_server_.reset();
    action_callback_group_.reset();
    cancel_service_.reset();
    cancel_client_.reset();
    return_to_upright_service_.reset();
    cancel_service_callback_group_.reset();
    cancel_client_callback_group_.reset();
    return_to_upright_service_callback_group_.reset();
    soft_stop_callback_group_.reset();
    setpoint_publisher_.reset();
    velocity_publisher_.reset();
    position_publisher_.reset();
    actuator_setpoint_publisher_.reset();
    soft_stop_subscription_.reset();
    fault_timer_.reset();
    actuator_.reset();
  }

  bool prepare_goal(const MoveRig::Goal & goal)
  {
    const double start_position = actuator_->actual_position_m();
    if (!std::isfinite(start_position) ||
      start_position < goal_position_minimum_ - return_default_tolerance_m_ ||
      start_position > goal_position_maximum_ + return_default_tolerance_m_)
    {
      RCLCPP_WARN(
        get_logger(), "Current position %f is outside configured actuator limits [%f, %f]",
        start_position, goal_position_minimum_, goal_position_maximum_);
      return false;
    }
    // The goal picks its type: positions -> pregenerated, otherwise a regular wave.
    goal_pregenerated_ = !goal.positions.empty();
    if (goal_pregenerated_) {
      if (goal.height != 0.0 || goal.period != 0.0) {
        RCLCPP_WARN(
          get_logger(),
          "Goal has both positions and height/period; send one or the other, rejecting");
        return false;
      }
      if (!std::isfinite(goal.sample_interval) || goal.sample_interval <= 0.0) {
        RCLCPP_WARN(
          get_logger(), "Pregenerated goal needs a positive sample_interval, rejecting");
        return false;
      }
      for (const double position : goal.positions) {
        if (!std::isfinite(position) || position < goal_position_minimum_ ||
          position > goal_position_maximum_)
        {
          RCLCPP_WARN(
            get_logger(), "Pregenerated position %f is outside configured actuator limits [%f, %f]",
            position, goal_position_minimum_, goal_position_maximum_);
          return false;
        }
      }
      // Blend from the paddle position to the first pregenerated position.
      const double distance = std::abs(goal.positions.front() - start_position);
      const double blend_duration = distance <= return_default_tolerance_m_ ? 0.0 :
        std::max(
        kQuinticPeakVelocityFactor * distance / return_max_velocity_mps_,
        return_min_duration_s_);
      pregenerated_.configure(goal.positions, goal.sample_interval, start_position, blend_duration);
      if (!std::isfinite(pregenerated_.duration())) {
        RCLCPP_WARN(get_logger(), "Pregenerated trajectory duration is not finite");
        return false;
      }
      return accept_peak(pregenerated_.peak_velocity(), pregenerated_.peak_acceleration());
    }
    if (!std::isfinite(goal.height) || !std::isfinite(goal.period) ||
      goal.height <= 0.0 || goal.period <= 0.0)
    {
      RCLCPP_WARN(
        get_logger(),
        "Received goal with non-positive height or period and no positions, rejecting");
      return false;
    }
    if (min_period_s_ > 0.0 && goal.period < min_period_s_) {
      RCLCPP_WARN(get_logger(), "Received goal with period %.2f below minimum %.2f, rejecting",
        goal.period, min_period_s_);
      return false;
    }
    double gain = wavemaker_transfer_gain_;
    if (rig_ == "wavemaker" && !gain_periods_.empty()) {
      const auto lookup = interpolate_gain(gain_periods_, gain_values_, goal.period);
      gain = lookup.gain;
      if (lookup.clamped) {
        RCLCPP_WARN(
          get_logger(),
          "Period %.2f is outside the configured gain table, using nearest end value %.3f",
          goal.period, gain);
      }
    }

    trajectory_.configure(
      {rig_ == "oscillator" ? "oscillator" : wavemaker_type_, water_depth_,
        flap_attachment_height_, wavemaker_position_offset_,
        upright_is_minimum_, gain, hinge_height_},
      goal.height, goal.period, start_position, return_max_velocity_mps_);

    const double required_minimum = trajectory_.required_minimum();
    const double required_maximum = trajectory_.required_maximum();
    if (required_minimum < goal_position_minimum_ ||
      required_maximum > goal_position_maximum_)
    {
      RCLCPP_WARN(
        get_logger(),
        "Received goal requiring position range [%f, %f] outside configured actuator "
        "limits [%f, %f], rejecting",
        required_minimum, required_maximum, goal_position_minimum_, goal_position_maximum_);
      return false;
    }
    return accept_peak(trajectory_.peak_velocity(), trajectory_.peak_acceleration());
  }

  // Checks the goal's peak speed against the drive and stores its positioning-speed cap.
  bool accept_peak(double peak_velocity_mps, double peak_acceleration_mps2)
  {
    if (!within_speed_limit(peak_velocity_mps) ||
      !within_acceleration_limit(peak_acceleration_mps2))
    {
      return false;
    }
    goal_velocity_cap_mps_ = velocity_cap(peak_velocity_mps);
    goal_acceleration_cap_mps2_ = acceleration_cap(peak_acceleration_mps2);
    return true;
  }

  // The drive's positioning velocity is a speed limit, not a target. Sending the motion's
  // peak speed with a small margin, instead of the instantaneous speed, lets the drive
  // reach the turning points and catch up after a lag, while still bounding its speed.
  double velocity_cap(double peak_velocity_mps) const
  {
    return std::min(
      peak_velocity_mps * positioning_velocity_margin_, actuator_->max_velocity_mps());
  }

   // Like velocity_cap(): a limit for the drive's profile, the motion's peak with a margin. At
  // least 1 mm/s^2: a goal that never accelerates still needs a positive limit, and the LinMot
  // drive refuses 0.
  double acceleration_cap(double peak_acceleration_mps2) const
  {
    return std::clamp(
      peak_acceleration_mps2 * positioning_acceleration_margin_, 1e-3,
      actuator_->max_acceleration_mps2());
  }

  // Rejects a trajectory the drive would refuse part-way through (write_setpoint fails
  // above max_velocity_mps(), which faults the node).
  bool within_speed_limit(double peak_velocity_mps) const
  {
    const double limit = actuator_->max_velocity_mps();
    if (!std::isfinite(peak_velocity_mps) || peak_velocity_mps > limit) {
      RCLCPP_WARN(
        get_logger(),
        "Received goal needing up to %f m/s, above the drive's maximum of %f m/s, rejecting",
        peak_velocity_mps, limit);
      return false;
    }
    return true;
  }

  bool within_acceleration_limit(double peak_acceleration_mps2) const
  {
    if (max_goal_acceleration_mps2_ > 0.0 && peak_acceleration_mps2 > max_goal_acceleration_mps2_) {
      RCLCPP_WARN(
        get_logger(),
        "Received goal needing up to %f m/s^2, above the configured maximum of %f m/s^2, rejecting",
        peak_acceleration_mps2, max_goal_acceleration_mps2_);
      return false;
    }
    const double limit = actuator_->max_acceleration_mps2();
    if (!std::isfinite(peak_acceleration_mps2) || peak_acceleration_mps2 > limit) {
      RCLCPP_WARN(
        get_logger(),
        "Received goal needing up to %f m/s^2, above the drive's maximum of %f m/s^2, rejecting",
        peak_acceleration_mps2, limit);
      return false;
    }
    return true;
  }

  bool sample_goal(double elapsed, double & position, double & velocity) const
  {
    if (goal_pregenerated_) {
      return pregenerated_.sample(elapsed, position, velocity);
    }
    trajectory_.sample(elapsed, position, velocity);
    return false;
  }


  // Lock order: execution_mutex_ before goal_mutex_. Workers never take execution_mutex_.
  void stop_execution()
  {
    std::lock_guard<std::mutex> exec_lock(execution_mutex_);
    stop_execution_.store(true);

    if (execution_thread_.joinable()) {
      execution_thread_.join();
    }
  }

  // Joins any previous worker and starts work, unless the node stopped accepting goals.
  // The check and the start happen under execution_mutex_, so stop_execution() either
  // joins the new worker or runs before it and makes this return false.
  bool start_execution(std::function<void()> work)
  {
    std::lock_guard<std::mutex> exec_lock(execution_mutex_);
    stop_execution_.store(true);
    if (execution_thread_.joinable()) {
      execution_thread_.join();
    }
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      if (!accepting_goals_) {
        return false;
      }
    }
    stop_execution_.store(false);
    execution_thread_ = std::thread(std::move(work));
    return true;
  }

  void abort_goal(
    const std::shared_ptr<MoveRigGoalHandle> & goal_handle, const std::string & message)
  {
    try {
      if (goal_handle && goal_handle->is_active()) {
        auto result = std::make_shared<MoveRig::Result>();
        result->success = false;
        result->message = message;
        goal_handle->abort(result);
      }
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "Could not abort goal: %s", error.what());
    } catch (...) {
      RCLCPP_WARN(get_logger(), "Could not abort goal: unknown exception");
    }
  }

  void abort_active_goal()
  {
    std::shared_ptr<MoveRigGoalHandle> goal_to_abort;
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      goal_to_abort = goal_handle_;
      goal_handle_.reset();
      goal_pending_ = false;
    }

    abort_goal(goal_to_abort, "Goal aborted by lifecycle transition");
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr setpoint_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr position_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr actuator_setpoint_publisher_;
  std::unique_ptr<bond::Bond> bond_;

  rclcpp_action::GoalResponse goal_callback(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveRig::Goal> goal)
  {
    (void)uuid;
    // Read before taking goal_mutex_ to avoid nesting the lifecycle state lock inside it.
    const std::string state_label = get_current_state().label();
    std::lock_guard<std::mutex> lock(goal_mutex_);

    if (!accepting_goals_) {
      RCLCPP_WARN(
        get_logger(),
        "Rejecting goal because the node is not accepting goals. State: %s",
        state_label.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (actuator_->faulted() || !actuator_->is_live()) {
      RCLCPP_WARN(get_logger(), "Rejecting goal because the actuator is faulted or not live");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (soft_stop_active()) {
      RCLCPP_WARN(get_logger(), "Rejecting goal because the soft stop is engaged");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal_pending_ || returning_to_upright_ || (goal_handle_ && goal_handle_->is_active())) {
      RCLCPP_WARN(
        get_logger(),
        "Rejecting goal because another goal is active or a return to upright is running");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (!prepare_goal(*goal)) {
      return rclcpp_action::GoalResponse::REJECT;
    }

    goal_pending_ = true;

    return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
  }

  rclcpp_action::CancelResponse cancel_callback(
    const std::shared_ptr<MoveRigGoalHandle> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted_callback(
    std::shared_ptr<MoveRigGoalHandle> goal_handle)
  {
    bool can_start = false;

    {
      std::lock_guard<std::mutex> lock(goal_mutex_);

      if (accepting_goals_) {
        goal_handle_ = goal_handle;
        can_start = true;
      }
      goal_pending_ = false;
    }

    const std::string inactive_message = "Goal rejected because the node is no longer active";
    if (!can_start) {
      abort_goal(goal_handle, inactive_message);
      return;
    }

    const bool started = start_execution([this, goal_handle]() {
          try {
            execute_goal(goal_handle);
          } catch (const std::exception & error) {
            RCLCPP_ERROR(get_logger(), "Goal execution failed: %s", error.what());
            release_goal(goal_handle, std::string("Goal execution failed: ") + error.what());
          } catch (...) {
            RCLCPP_ERROR(get_logger(), "Goal execution failed with an unknown exception");
            release_goal(goal_handle, "Goal execution failed with an unknown exception");
          }
      });
    if (!started) {
      {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        if (goal_handle_ == goal_handle) {
          goal_handle_.reset();
        }
      }
      abort_goal(goal_handle, inactive_message);
    }
  }

  void release_goal(
    const std::shared_ptr<MoveRigGoalHandle> & goal_handle, const std::string & message)
  {
    try {
      if (actuator_) {
        actuator_->halt();
      }
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "Exception while releasing goal");
    }
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      if (goal_handle_ == goal_handle) {
        goal_handle_.reset();
        goal_pending_ = false;
      }
    }
    abort_goal(goal_handle, message);
  }

  ReturnStatus move_to_upright(double tolerance, double & final_position)
  {
    const double start = actuator_->actual_position_m();
    final_position = start;
    // Allow the tolerance outside the limits so a paddle resting at an end stop can return.
    if (!std::isfinite(start) ||
      start < goal_position_minimum_ - return_default_tolerance_m_ ||
      start > goal_position_maximum_ + return_default_tolerance_m_)
    {
      RCLCPP_ERROR(get_logger(), "Invalid start position: %f", start);
      return ReturnStatus::PositionInvalid;
    }

    const double target = wavemaker_position_offset_;
    const double delta = target - start;
    // Already at upright: nothing to move.
    if (std::abs(delta) <= tolerance) {
      actuator_->halt();
      return ReturnStatus::Success;
    }
    const double duration = std::max(
      kQuinticPeakVelocityFactor * std::abs(delta) / return_max_velocity_mps_,
      return_min_duration_s_);
    const double deadline = duration + return_timeout_margin_s_;
    const double velocity_cap_mps =
      velocity_cap(kQuinticPeakVelocityFactor * std::abs(delta) / duration);
    const double acceleration_cap_mps2 =
      acceleration_cap(kQuinticPeakAccelerationFactor * std::abs(delta) / (duration * duration));
    const auto t0 = std::chrono::steady_clock::now();
    const auto elapsed = [&t0] {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      };
    rclcpp::WallRate loop_rate(control_rate_hz_);  // steady clock, like elapsed

    while (rclcpp::ok() && !stop_execution_.load()) {
      if (soft_stop_active()) {
        RCLCPP_WARN(get_logger(), "Soft stop engaged during return to upright");
        actuator_->halt();
        return ReturnStatus::SoftStop;
      }
      if (actuator_->faulted()) {
        RCLCPP_ERROR(get_logger(), "Actuator fault detected");
        actuator_->halt();
        return ReturnStatus::Fault;
      }

      const double t = elapsed();
      const Blend b = quintic_blend(t, duration);
      if (!publish_and_write_setpoint(
        start + b.s * delta, b.ds * delta, velocity_cap_mps, acceleration_cap_mps2))
      {
        actuator_->halt();
        return ReturnStatus::Fault;
      }

      final_position = actuator_->actual_position_m();
      if (t >= duration && std::abs(final_position - target) <= tolerance) {
        actuator_->halt();
        return ReturnStatus::Success;
      }
      if (t >= deadline) {
        RCLCPP_ERROR(get_logger(), "Move to upright timed out");
        actuator_->halt();
        return ReturnStatus::Timeout;
      }
      loop_rate.sleep();
    }
    actuator_->halt();
    // A driver fault deactivates the node, which also ends this loop.
    return actuator_->faulted() ? ReturnStatus::Fault : ReturnStatus::Stopped;
  }

  void execute_goal(
    std::shared_ptr<MoveRigGoalHandle> goal_handle)
  {
    auto result = std::make_shared<MoveRig::Result>();
    auto feedback = std::make_shared<MoveRig::Feedback>();

    goal_handle->execute();
    const auto start_time = std::chrono::steady_clock::now();
    rclcpp::WallRate loop_rate(control_rate_hz_);  // steady clock, like elapsed

    while (rclcpp::ok() && !stop_execution_.load()) {
      // Check for cancellation
      if (goal_handle->is_canceling()) {
        actuator_->halt();

        {
          std::lock_guard<std::mutex> lock(goal_mutex_);
          if (goal_handle_ == goal_handle) {
            goal_handle_.reset();
            goal_pending_ = false;
          }
        }

        result->success = false;
        result->message = "Goal canceled; actuator halted and remains enabled";
        goal_handle->canceled(result);
        return;
      }
      if (soft_stop_active()) {
        actuator_->halt();

        {
          std::lock_guard<std::mutex> lock(goal_mutex_);
          if (goal_handle_ == goal_handle) {
            goal_handle_.reset();
            goal_pending_ = false;
          }
        }
        abort_goal(goal_handle, "Soft stop engaged; actuator halted and remains enabled");
        return;
      }

      const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() -
        start_time).count();
      double x, v;
      const bool trajectory_complete = sample_goal(t, x, v);

      if (!publish_and_write_setpoint(x, v, goal_velocity_cap_mps_, goal_acceleration_cap_mps2_)) {
        RCLCPP_ERROR(get_logger(), "Failed to write actuator setpoint");
        auto failure = std::make_shared<MoveRig::Result>();
        failure->success = false;
        const std::string reason = actuator_->fault_reason();
        failure->message = reason.empty() ?
          "Actuator rejected setpoint" : "Actuator rejected setpoint: " + reason;
        RCLCPP_ERROR(get_logger(), "%s", failure->message.c_str());
        {
          std::lock_guard<std::mutex> lock(goal_mutex_);
          accepting_goals_ = false;
          if (goal_handle_ == goal_handle) {
            goal_handle_.reset();
            goal_pending_ = false;
          }
        }
        handle_driver_fault(failure->message);
        actuator_->stop();
        abort_goal(goal_handle, failure->message);
        return;
      }

      feedback->desired_position = x;
      feedback->actual_position = actuator_->actual_position_m();
      feedback->elapsed_time = t;
      goal_handle->publish_feedback(feedback);

      if (trajectory_complete) {
        actuator_->halt();
        result->success = true;
        result->message = "Pregenerated trajectory completed";
        goal_handle->succeed(result);
        std::lock_guard<std::mutex> lock(goal_mutex_);
        if (goal_handle_ == goal_handle) {
          goal_handle_.reset();
          goal_pending_ = false;
        }
        return;
      }

      loop_rate.sleep();
    }
    actuator_->halt();
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      if (goal_handle_ == goal_handle) {
        goal_handle_.reset();
        goal_pending_ = false;
      }
    }
    abort_goal(goal_handle, "Goal stopped");
  }

  // Takes the paddle setpoint in metres. Clamps it to the configured limits before
  // converting, because the start-position check allows a paddle up to
  // return_default_tolerance_m_ outside them and the drive rejects any target outside its
  // limits. Publishes the setpoint and its trajectory velocity (m, m/s), the measured
  // position, and the setpoint in actuator units. The drive gets velocity_cap_mps as its
  // positioning-speed limit.
  bool publish_and_write_setpoint(
    double position_m, double velocity_mps, double velocity_cap_mps, double acceleration_cap_mps2)
  {

    const double clamped_position =
      std::clamp(position_m, goal_position_minimum_, goal_position_maximum_);

    std_msgs::msg::Float64 setpoint_msg;
    setpoint_msg.data = clamped_position;
    setpoint_publisher_->publish(setpoint_msg);
    std_msgs::msg::Float64 velocity_msg;
    velocity_msg.data = velocity_mps;
    velocity_publisher_->publish(velocity_msg);
    std_msgs::msg::Float64 position_msg;
    position_msg.data = actuator_->actual_position_m();
    position_publisher_->publish(position_msg);

    const auto actuator_setpoint =
      actuator_->to_actuator_setpoint(clamped_position, velocity_cap_mps, acceleration_cap_mps2);
    std_msgs::msg::Float64 actuator_setpoint_msg;
    actuator_setpoint_msg.data = actuator_setpoint.position;
    actuator_setpoint_publisher_->publish(actuator_setpoint_msg);
    return actuator_->write_setpoint(actuator_setpoint);
  }

  void handle_driver_fault(const std::string & reason)
  {
    {
      std::lock_guard<std::mutex> lock(fault_mutex_);
      if (!fault_pending_.load()) {
        pending_fault_reason_ = reason;
      }
    }
    fault_pending_.store(true);
  }

  void process_driver_fault()
  {
    if (!fault_pending_.exchange(false)) {
      return;
    }

    std::string reason;
    {
      std::lock_guard<std::mutex> lock(fault_mutex_);
      reason = pending_fault_reason_;
      pending_fault_reason_.clear();
    }
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      accepting_goals_ = false;
    }
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      RCLCPP_WARN(
        get_logger(), "Driver fault occurred but node is not active: %s", reason.c_str());
      return;
    }

    RCLCPP_ERROR(get_logger(), "Driver fault: %s", reason.c_str());
    abort_active_goal();
    fault_transition_pending_.store(true);
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
  }

  std::string rig_{"wavemaker"};
  std::string wavemaker_type_;
  std::string driver_address_;
  double travel_minimum_{0.0};
  double travel_maximum_{0.0};
  double goal_position_minimum_{0.0};
  double goal_position_maximum_{0.0};
  bool upright_is_minimum_{false};
  rig_controller::WaveTrajectory trajectory_;
  rig_controller::PregeneratedWaveTrajectory pregenerated_;
  double wavemaker_position_offset_{0.0};
  bool goal_pregenerated_{false};  // type of the accepted goal, set in prepare_goal
  double flap_attachment_height_{0.0};
  double hinge_height_{0.0};
  double water_depth_{0.0};
  rig_controller::DriveScale drive_scale_;
  std::mutex goal_mutex_;
  std::shared_ptr<MoveRigGoalHandle> goal_handle_;
  bool goal_pending_{false};
  bool returning_to_upright_{false};  // guarded by goal_mutex_
  double return_max_velocity_mps_{0.1};
  double control_rate_hz_{100.0};  // from actuator_->update_period_ms(), set in on_configure
  double positioning_velocity_margin_{1.2};
  double positioning_acceleration_margin_{1.2};
  double goal_velocity_cap_mps_{0.0};  // set in prepare_goal, used by execute_goal
  double goal_acceleration_cap_mps2_{0.0};  // set in prepare_goal, used by execute_goal
  double max_goal_acceleration_mps2_{0.0};
  double min_period_s_{0.0};
  double return_min_duration_s_{1.0};
  double return_default_tolerance_m_{0.002};
  double return_timeout_margin_s_{2.0};
  double wavemaker_transfer_gain_{1.0};
  std::vector<double> gain_periods_;
  std::vector<double> gain_values_;
  rclcpp_action::Server<MoveRig>::SharedPtr action_server_;
  rclcpp::CallbackGroup::SharedPtr action_callback_group_;
  rclcpp::CallbackGroup::SharedPtr cancel_service_callback_group_;
  rclcpp::CallbackGroup::SharedPtr cancel_client_callback_group_;
  rclcpp::CallbackGroup::SharedPtr soft_stop_callback_group_;
  rclcpp::CallbackGroup::SharedPtr return_to_upright_service_callback_group_;
  rclcpp_action::Client<MoveRig>::SharedPtr cancel_client_;
  rclcpp::Service<CancelAllGoals>::SharedPtr cancel_service_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr soft_stop_subscription_;
  rclcpp::Service<ReturnToUpright>::SharedPtr return_to_upright_service_;
  rclcpp::TimerBase::SharedPtr fault_timer_;
  rclcpp::PreShutdownCallbackHandle pre_shutdown_callback_handle_;
  std::atomic<bool> stop_execution_{false};
  std::atomic<bool> soft_stopped_{true};
  std::atomic<int64_t> soft_stop_last_msg_ns_{0};
  int64_t soft_stop_timeout_ns_{0};
  int64_t soft_stop_configured_ns_{0};
  bool soft_stop_missing_logged_{false};
  std::atomic<bool> fault_pending_{false};
  std::atomic<bool> fault_transition_pending_{false};
  std::mutex fault_mutex_;
  std::string pending_fault_reason_;
  bool accepting_goals_{false};
  bool soft_stop_silent_logged_{false};
  std::mutex execution_mutex_;  // guards execution_thread_
  std::thread execution_thread_;
  ActuatorFactory actuator_factory_;
  std::unique_ptr<rig_controller::Actuator> actuator_;

  // std::unique_ptr<WavemakerDriver> driver_;
  // rclcpp::TimerBase::SharedPtr command_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControllerNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  try {
    executor.spin();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("controller_node"), "Exception in executor: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger("controller_node"), "Unknown exception in executor");
  }

  rclcpp::shutdown();
  return 0;
}
