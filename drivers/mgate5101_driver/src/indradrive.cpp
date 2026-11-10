#include "mgate/indradrive.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace mgate {
namespace {

constexpr int kPollSleepMs = 20;

bool in_operating_mode(std::uint16_t sw) { return (sw & indra_sw::kModeMask) == indra_sw::kModeOperating; }
bool power_ready(std::uint16_t sw) { return (sw & indra_sw::kReadyMask) >= indra_sw::kReadyAb; }
bool with_torque(std::uint16_t sw) { return (sw & indra_sw::kReadyMask) == indra_sw::kReadyAF; }
bool without_torque(std::uint16_t sw) { return (sw & indra_sw::kReadyMask) != indra_sw::kReadyAF; }
bool following(std::uint16_t sw) { return (sw & indra_sw::kNotFollowing) == 0; }
bool standstill(std::uint16_t sw) { return (sw & indra_sw::kInStandstill) != 0; }
bool no_error(std::uint16_t sw) { return (sw & indra_sw::kClass1Error) == 0; }

}  // namespace

IndraDrive::IndraDrive(MGateDriver& drv, IndraDriveConfig cfg) : drv_(drv), cfg_(std::move(cfg)) {
    // The layout below is 14 bytes in, 12 bytes out. A larger output block would write over the
    // next slave's outputs every cycle (write_only_dirty = false writes the whole area); on the
    // ladertanken daisy chain that is the other rig's control word. More inputs only read.
    const GatewayConfig& gw = drv_.config();
    if (gw.output_word_count != 6 || gw.input_word_count < 7) {
        throw std::invalid_argument(
            "IndraDrive needs output_word_count 6 and input_word_count >= 7, not " +
            std::to_string(gw.output_word_count) + " / " + std::to_string(gw.input_word_count));
    }
    drv_.add_tags({
        {tag("StatusWord"), Area::Input, DataType::U16, 0,  0, 0, ByteOrder::BigEndian, 1.0,    0.0, "",    ""},
        {tag("Position"),   Area::Input, DataType::I32, 2,  0, 0, ByteOrder::BigEndian, 0.0001, 0.0, "deg", ""},
        {tag("Velocity"),   Area::Input, DataType::I32, 6,  0, 0, ByteOrder::BigEndian, 0.0001, 0.0, "rpm", ""},
        {tag("DiagNumber"), Area::Input, DataType::U32, 10, 0, 0, ByteOrder::BigEndian, 1.0,    0.0, "",    ""},

        {tag("ControlWord"),    Area::Output, DataType::U16, 0,  0, 0, ByteOrder::BigEndian, 1.0,    0.0, "",    ""},
        {tag("TargetPosition"), Area::Output, DataType::I32, 2,  0, 0, ByteOrder::BigEndian, 0.0001, 0.0, "deg", ""},
        {tag("PosVelocity"),    Area::Output, DataType::I32, 6,  0, 0, ByteOrder::BigEndian, 0.0001, 0.0, "rpm", ""},
        {tag("Override"),       Area::Output, DataType::U16, 10, 0, 0, ByteOrder::BigEndian, 0.01,   0.0, "%",   ""},
    });
    drv_.set_cycle_callback([this](bool ok, const Result&) { on_cycle(ok); });
}

// --- state -------------------------------------------------------------------

std::uint16_t IndraDrive::status_word() const {
    return static_cast<std::uint16_t>(drv_.read_int(tag("StatusWord")));
}
double IndraDrive::position_deg() const { return drv_.read(tag("Position")); }
double IndraDrive::velocity_rpm() const { return drv_.read(tag("Velocity")); }
std::uint32_t IndraDrive::diag_number() const {
    return static_cast<std::uint32_t>(drv_.read_int(tag("DiagNumber")));
}

bool IndraDrive::slave_live() const {
    // Gateway input memory: 1536..1537 status word (bits 1:0 = 11 Operate),
    // 1538.. live list, one bit per PROFIBUS address. Status area starts at
    // byte 1536, so byte b of the area is register b/2, high byte when even.
    // The snapshot is not cleared when polling stops, so it is stale while offline.
    if (!drv_.online()) return false;
    const auto st = drv_.status_snapshot();
    if (st.empty() || (st[0] & 0x3u) != 0x3u) return false;

    const std::size_t b   = 2 + static_cast<std::size_t>(cfg_.profibus_address) / 8;
    const std::size_t reg = b / 2;
    if (reg >= st.size()) return false;
    const unsigned byte = (b % 2 == 0) ? (st[reg] >> 8) : (st[reg] & 0xFFu);
    return (byte >> (cfg_.profibus_address % 8)) & 1u;
}

std::string IndraDrive::fault_reason() const {
    std::lock_guard<std::mutex> lk(fault_mtx_);
    return fault_reason_;
}

void IndraDrive::set_fault_callback(FaultCallback callback) {
    std::lock_guard<std::mutex> lk(callback_mtx_);
    fault_callback_ = std::move(callback);
}

std::string IndraDrive::describe() const {
    const std::uint16_t sw   = status_word();
    const std::uint32_t diag = diag_number();
    static const char* const kReady[] = {"--", "bb", "Ab", "AF"};

    char buf[200];
    std::snprintf(buf, sizeof(buf),
                  "SW=0x%04X %s %s%s%s%s pos=%10.4f deg vel=%9.4f rpm diag=%X%04X slave=%s",
                  sw, in_operating_mode(sw) ? "OM" : "PM", kReady[(sw >> 14) & 0x3u],
                  (sw & indra_sw::kClass1Error) ? " ERROR" : "",
                  (sw & indra_sw::kClass2Warning) ? " warn" : "",
                  (sw & indra_sw::kTargetReached) ? " inpos" : "", position_deg(), velocity_rpm(),
                  static_cast<unsigned>((diag >> 16) & 0xFu), static_cast<unsigned>(diag & 0xFFFFu),
                  slave_live() ? "live" : "DOWN");
    return buf;
}

// --- internals ---------------------------------------------------------------

void IndraDrive::set_control(std::uint16_t cw) { drv_.write_int(tag("ControlWord"), cw); }

void IndraDrive::latch_fault(const std::string& why) {
    set_control(0);
    enabled_.store(false);
    bool newly_latched = false;
    {
        std::lock_guard<std::mutex> lk(fault_mtx_);
        if (!fault_.load()) {
            fault_reason_ = why;
            newly_latched = true;
        }
    }
    fault_.store(true);
    if (newly_latched) {
        FaultCallback callback;
        {
            std::lock_guard<std::mutex> lk(callback_mtx_);
            callback = fault_callback_;
        }
        if (callback) callback(why);
    }
}

void IndraDrive::on_cycle(bool ok) {
    // Runs on the poll thread after every cycle.
    if (!ok) {
        // Always drop the control word, so a reconnect cannot restart the drive.
        set_control(0);
        if (enabled_.load()) latch_fault("Modbus link to gateway lost");
        return;
    }
    if (!enabled_.load()) return;
    if (!slave_live()) {
        latch_fault("drive dropped out of PROFIBUS data exchange");
    } else if (status_word() & indra_sw::kClass1Error) {
        char buf[64];
        const std::uint32_t d = diag_number();
        std::snprintf(buf, sizeof(buf), "drive error %X%04X", static_cast<unsigned>((d >> 16) & 0xFu),
                      static_cast<unsigned>(d & 0xFFFFu));
        latch_fault(buf);
    }
}

std::string IndraDrive::last_error() const {
    std::lock_guard<std::mutex> lk(error_mtx_);
    return last_error_;
}

bool IndraDrive::fail(const std::string& why) {
    {
        std::lock_guard<std::mutex> lk(error_mtx_);
        last_error_ = why;
    }
    std::fprintf(stderr, "IndraDrive: %s\n", why.c_str());
    return false;
}

bool IndraDrive::wait_for(const char* what, int timeout_ms, bool (*pred)(std::uint16_t), bool stoppable) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    for (;;) {
        if (stoppable && abort_.load()) return fail(std::string("aborted while waiting for ") + what);
        if (stoppable && fault_.load())
            return fail(std::string("fault while waiting for ") + what + ": " + fault_reason());
        if (!drv_.online()) return fail(std::string("gateway offline while waiting for ") + what);
        const std::uint16_t sw = status_word();
        if (pred(sw)) return true;
        if (std::chrono::steady_clock::now() >= deadline) {
            char buf[160];
            std::snprintf(buf, sizeof(buf), "timeout waiting for %s (status word 0x%04X)", what, sw);
            return fail(buf);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kPollSleepMs));
    }
}

// --- commands ----------------------------------------------------------------

bool IndraDrive::clear_errors() {
    const auto base = static_cast<std::uint16_t>(drv_.read_output_int(tag("ControlWord")) & ~indra_cw::kClearErrors);
    set_control(base | indra_cw::kClearErrors);
    const bool ok = wait_for("errors to clear", 3000, no_error);
    set_control(base);  // 1->0 terminates C5
    return ok;
}

bool IndraDrive::enable() {
    using namespace indra_cw;

    if (!drv_.online()) return fail("gateway offline");
    if (!slave_live()) return fail("drive is not in PROFIBUS data exchange");
    {
        std::lock_guard<std::mutex> lk(fault_mtx_);
        fault_reason_.clear();
    }
    fault_.store(false);

    if (status_word() & indra_sw::kClass1Error) {
        return fail("drive has an active error (" + describe() + "), call clear_errors() first");
    }

    // Park the command values on the current position before anything else.
    drv_.write(tag("Override"), 100.0);
    drv_.write(tag("PosVelocity"), std::fmin(10.0, cfg_.max_velocity_rpm));
    drv_.write(tag("TargetPosition"), position_deg());

    auto abort_enable = [this](bool ok) {
        if (!ok) {
            enabled_.store(false);
            set_control(kOperatingMode);
        }
        return ok;
    };

    set_control(kOperatingMode);
    if (!wait_for("operating mode", cfg_.step_timeout_ms, in_operating_mode)) return abort_enable(false);
    if (!wait_for("power section ready (Ab) - is mains/DC bus on?", cfg_.step_timeout_ms, power_ready))
        return abort_enable(false);

    enabled_.store(true);
    set_control(kOperatingMode | kDriveOn);  // torque on, Drive Halt still active
    if (!wait_for("drive ON (AF)", cfg_.step_timeout_ms, with_torque)) return abort_enable(false);

    drv_.write(tag("TargetPosition"), position_deg());  // re-park after torque came on
    set_control(kOperatingMode | kDriveOn | kDriveStart);
    if (!wait_for("drive to follow command values", cfg_.step_timeout_ms, following)) return abort_enable(false);
    return true;
}

bool IndraDrive::move_to(double position, double velocity_rpm) {
    if (!enabled_.load() || fault_.load()) return fail("move_to: drive not enabled" +
                                                       (fault_.load() ? " (" + fault_reason() + ")" : std::string()));
    if (position < cfg_.min_position || position > cfg_.max_position) {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "move_to: %.4f outside [%.1f, %.1f]", position,
                      cfg_.min_position, cfg_.max_position);
        return fail(buf);
    }
    if (!(velocity_rpm > 0.0) || velocity_rpm > cfg_.max_velocity_rpm) {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "move_to: velocity %.2f rpm outside (0, %.1f]", velocity_rpm,
                      cfg_.max_velocity_rpm);
        return fail(buf);
    }
    // Velocity first: if a cycle goes out between the two writes, the new
    // target is never paired with a stale velocity.
    drv_.write(tag("PosVelocity"), velocity_rpm);
    drv_.write(tag("TargetPosition"), position);
    return true;
}

bool IndraDrive::wait_in_position(int timeout_ms) {
    const double target   = drv_.read_output(tag("TargetPosition"));
    const auto   deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    for (;;) {
        if (abort_.load()) return fail("aborted while moving");
        if (fault_.load()) return fail("fault while moving: " + fault_reason());
        if (!drv_.online()) return fail("gateway offline while moving");
        const bool reached = (status_word() & indra_sw::kTargetReached) != 0;
        if (reached && std::fabs(position_deg() - target) <= cfg_.in_position_tol_deg) return true;
        if (std::chrono::steady_clock::now() >= deadline) return fail("timeout waiting for target position");
        std::this_thread::sleep_for(std::chrono::milliseconds(kPollSleepMs));
    }
}

void IndraDrive::halt() {
    if (enabled_.load()) set_control(indra_cw::kOperatingMode | indra_cw::kDriveOn);
}

bool IndraDrive::disable() {
    using namespace indra_cw;
    bool ok = true;
    if (enabled_.load() && drv_.online()) {
        set_control(kOperatingMode | kDriveOn);  // Drive Halt: decelerate and hold
        ok = wait_for("standstill", cfg_.step_timeout_ms, standstill, false);
    }
    enabled_.store(false);
    set_control(kOperatingMode);  // torque off
    if (drv_.online()) ok = wait_for("torque off", cfg_.step_timeout_ms, without_torque, false) && ok;
    return ok;
}

void IndraDrive::shutdown() {
    enabled_.store(false);
    set_control(0);
}

}  // namespace mgate
