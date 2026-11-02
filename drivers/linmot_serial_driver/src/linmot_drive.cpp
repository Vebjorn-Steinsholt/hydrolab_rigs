#include "linmot/linmot_drive.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <utility>

namespace linmot {
namespace {

std::string hex16(std::uint16_t v) {
    char buf[8];
    std::snprintf(buf, sizeof(buf), "0x%04X", v);
    return buf;
}

std::uint32_t to_wire(double value, double scale) {
    return static_cast<std::uint32_t>(std::lround(value * scale));
}

}  // namespace

std::string describe_status(std::uint16_t sw) {
    std::string out = hex16(sw);
    const struct {
        std::uint16_t bit;
        const char* name;
    } names[] = {
        {linmot_sw::kOperationEnabled, "operation enabled"},
        {linmot_sw::kError, "error"},
        {linmot_sw::kSwitchOnLocked, "switch on locked"},
        {linmot_sw::kWarning, "warning"},
        {linmot_sw::kInTargetPosition, "in target"},
        {linmot_sw::kHomed, "homed"},
        {linmot_sw::kFatalError, "fatal error"},
        {linmot_sw::kMotionActive, "motion active"},
    };
    std::string list;
    for (const auto& n : names) {
        if (sw & n.bit) list += (list.empty() ? "" : ", ") + std::string(n.name);
    }
    return out + " (" + (list.empty() ? "-" : list) + ")";
}

LinMotDrive::LinMotDrive(SerialBridge& bridge, LinMotDriveConfig cfg)
    : bridge_(bridge), cfg_(std::move(cfg)) {
    bridge_.set_reply_callback(
        [this](const Reply& reply, std::uint64_t sequence) { on_reply(reply, sequence); });
    bridge_.set_cycle_callback([this] { on_cycle(); });
}

LinMotDrive::~LinMotDrive() {
    bridge_.set_reply_callback(nullptr);
    bridge_.set_cycle_callback(nullptr);
}

// --- state -------------------------------------------------------------------

bool LinMotDrive::feedback_valid() const { return bridge_.latest().valid; }

std::uint16_t LinMotDrive::status_word() const { return bridge_.latest().status_word; }

double LinMotDrive::position_m() const {
    return bridge_.latest().position / kPositionPerMetre;
}

std::string LinMotDrive::fault_reason() const {
    std::lock_guard<std::mutex> lk(fault_mtx_);
    return fault_reason_;
}

std::string LinMotDrive::describe() const {
    const Feedback fb = bridge_.latest();
    if (!bridge_.port_ok()) return "serial port down: " + bridge_.stats().last_error;
    if (!bridge_.ready()) return "waiting for the Arduino";
    if (!fb.valid) return "no feedback from the drive";
    char pos[32];
    std::snprintf(pos, sizeof(pos), "%.3f mm", fb.position / kPositionPerMetre * 1000.0);
    std::string s = std::string(pos) + ", status " + describe_status(fb.status_word);
    if (faulted()) s += ", FAULT: " + fault_reason();
    return s;
}

void LinMotDrive::set_fault_callback(FaultCallback callback) {
    std::lock_guard<std::mutex> lk(callback_mtx_);
    fault_callback_ = std::move(callback);
}

// --- commands ----------------------------------------------------------------

bool LinMotDrive::fail(const std::string& why) {
    last_error_ = why;
    return false;
}

bool LinMotDrive::wait_status(const char* what, std::uint64_t after, int timeout_ms,
                              const std::function<bool(std::uint16_t)>& pred) {
    const bool ok = bridge_.wait_feedback(
        after, [&](const Feedback& fb) { return pred(fb.status_word); }, timeout_ms, &abort_);
    if (ok) return true;
    if (abort_) return fail(std::string("aborted while waiting: ") + what);
    return fail(std::string("timed out waiting: ") + what + "; status " +
                describe_status(status_word()));
}

bool LinMotDrive::bring_up(const LogCallback& log) {
    const auto say = [&](const std::string& s) { if (log) log(s); };
    enabled_ = false;
    abort_ = false;
    bridge_.clear_target();
    {
        std::lock_guard<std::mutex> lk(fault_mtx_);
        fault_ = false;
        fault_reason_.clear();
    }

    if (!bridge_.port_ok()) return fail("serial port down: " + bridge_.stats().last_error);
    if (!bridge_.wait_ready(cfg_.ready_timeout_ms)) {
        return fail("the Arduino does not answer (no READY); check the USB cable and firmware");
    }

    std::uint64_t seq = bridge_.send(Command::Operational);
    if (!bridge_.wait_feedback(seq, [](const Feedback&) { return true; }, cfg_.step_timeout_ms,
                               &abort_)) {
        return fail(abort_ ? "aborted" :
                    "the drive does not answer on CAN; check its power, CAN cable, node ID "
                    "and that TPDO1 is synchronous");
    }

    std::uint16_t sw = status_word();
    if (sw & linmot_sw::kFatalError) {
        return fail("fatal drive error " + describe_status(sw) +
                    "; restart the drive or check it in LinMot-Talk");
    }
    if (sw & linmot_sw::kError) {
        say("drive error " + describe_status(sw) + ", acknowledging");
        seq = bridge_.send(Command::AckError);
        if (!wait_status("error acknowledged", seq, cfg_.step_timeout_ms,
                         [](std::uint16_t s) { return !(s & linmot_sw::kError); })) {
            return false;
        }
    }

    sw = status_word();
    if (!(sw & linmot_sw::kOperationEnabled)) {
        seq = bridge_.send(Command::SwitchOn);
        if (!wait_status("operation enabled", seq, cfg_.step_timeout_ms,
                         [](std::uint16_t s) { return s & linmot_sw::kOperationEnabled; })) {
            last_error_ += " (if LinMot-Talk's control panel has control, release it)";
            return false;
        }
    }

    sw = status_word();
    if (!(sw & linmot_sw::kHomed)) {
        if (!cfg_.home_if_needed) return fail("the drive is not homed " + describe_status(sw));
        say("drive not homed: homing, the slide moves to its home stop");
        seq = bridge_.send(Command::Home);
        if (!wait_status("homed", seq, cfg_.home_timeout_ms, [](std::uint16_t s) {
                return (s & linmot_sw::kHomed) && !(s & linmot_sw::kMotionActive);
            })) {
            return false;
        }
        // Clears the home bit in the control word; the drive stays switched on.
        seq = bridge_.send(Command::SwitchOn);
        if (!wait_status("operation enabled after homing", seq, cfg_.step_timeout_ms,
                         [](std::uint16_t s) {
                             return (s & linmot_sw::kOperationEnabled) && (s & linmot_sw::kHomed);
                         })) {
            return false;
        }
        say("homed");
    }

    restarts_at_enable_ = bridge_.restarts();
    enabled_ = true;
    last_error_.clear();
    return true;
}

bool LinMotDrive::move_to(double position_m, double max_velocity_mps, double acceleration_mps2) {
    if (fault_) return fail("faulted: " + fault_reason());
    if (!enabled_) return fail("not enabled");
    if (!std::isfinite(position_m) || position_m < cfg_.min_position_m ||
        position_m > cfg_.max_position_m) {
        return fail("position outside the limits");
    }
    if (!std::isfinite(max_velocity_mps) || max_velocity_mps <= 0.0 ||
        max_velocity_mps > cfg_.max_velocity_mps) {
        return fail("velocity outside the limits");
    }
    if (!std::isfinite(acceleration_mps2) || acceleration_mps2 <= 0.0 ||
        acceleration_mps2 > cfg_.max_acceleration_mps2) {
        return fail("acceleration outside the limits");
    }
    const std::uint32_t acceleration = to_wire(acceleration_mps2, kAccelerationPerMps2);
    bridge_.set_target(static_cast<std::int32_t>(std::lround(position_m * kPositionPerMetre)),
                       to_wire(max_velocity_mps, kVelocityPerMps), acceleration, acceleration);
    return true;
}

void LinMotDrive::halt() { bridge_.clear_target(); }

bool LinMotDrive::disable() {
    enabled_ = false;
    bridge_.clear_target();
    if (!bridge_.port_ok() || !bridge_.ready()) return fail("no link to the Arduino");
    const std::uint64_t seq = bridge_.send(Command::SwitchOff);
    const bool off = wait_status("switched off", seq, cfg_.step_timeout_ms, [](std::uint16_t s) {
        return !(s & linmot_sw::kOperationEnabled);
    });
    // Only once it is off: pre-operational turns off the drive's sync watchdog, so the idle drive
    // does not go into error when the SYNC frames stop. A drive that is still on stays
    // operational, and its sync watchdog stops it once the bridge stops sending.
    if (off) {
        const std::uint64_t pre = bridge_.send(Command::PreOperational);
        if (!bridge_.wait_sent(pre, cfg_.step_timeout_ms)) {
            return fail("could not send NMT pre-operational");
        }
    }
    return off;
}

// --- monitoring (bridge thread) ----------------------------------------------

void LinMotDrive::latch_fault(const std::string& why) {
    bridge_.clear_target();
    enabled_ = false;
    {
        std::lock_guard<std::mutex> lk(fault_mtx_);
        if (fault_) return;
        fault_ = true;
        fault_reason_ = why;
    }
    FaultCallback cb;
    {
        std::lock_guard<std::mutex> lk(callback_mtx_);
        cb = fault_callback_;
    }
    if (cb) cb(why);
}

void LinMotDrive::on_reply(const Reply& reply, std::uint64_t) {
    if (!enabled_) return;
    if (reply.type == ReplyType::Ready) {
        if (bridge_.restarts() != restarts_at_enable_) {
            latch_fault("the Arduino restarted");
        }
        return;
    }
    if (reply.flags & flag::kWatchdogTripped) {
        latch_fault("the Arduino's watchdog stopped the motion (no frame for 200 ms)");
    } else if (reply.flags & flag::kCanSendError) {
        latch_fault("CAN send error on the Arduino");
    } else if (reply.flags & flag::kRejected) {
        latch_fault("the Arduino rejected a command (outside its limits)");
    } else if (reply.status_word & (linmot_sw::kError | linmot_sw::kFatalError)) {
        latch_fault("drive error " + describe_status(reply.status_word));
    } else if (!(reply.status_word & linmot_sw::kOperationEnabled)) {
        latch_fault("the drive switched off " + describe_status(reply.status_word));
    }
}

void LinMotDrive::on_cycle() {
    if (!enabled_) return;
    if (!bridge_.port_ok()) {
        latch_fault("serial port down: " + bridge_.stats().last_error);
        return;
    }
    const Feedback fb = bridge_.latest();
    const auto age = std::chrono::steady_clock::now() - fb.received;
    if (!fb.valid || age > std::chrono::milliseconds(cfg_.feedback_timeout_ms)) {
        latch_fault("no feedback from the drive for " + std::to_string(cfg_.feedback_timeout_ms) +
                    " ms");
    }
}

}  // namespace linmot
