// linmot_drive.h - LinMot drive (Motion Control SW, CANopen) on top of SerialBridge.
//
// Positions are metres in the drive's own frame: the slide homes against one end stop, which
// the drive calls its Home Position (200 mm on the sloshing rig), and moves to its Initial
// Position afterwards.
//
// bring_up() reads the status word and acts on it, one step at a time:
//   bridge silent ............. fail (no READY within ready_timeout_ms)
//   drive silent on CAN ....... fail (no feedback after NMT operational)
//   fatal error ............... fail; needs a drive restart or LinMot-Talk
//   error ..................... acknowledge; fail if it stays
//   not operation enabled ..... switch on; fail if it does not (LinMot-Talk may hold control)
//   not homed ................. home if home_if_needed, else fail; the slide moves to its stop
//   operation enabled, homed .. enabled
//
// Safety: once enabled, a drive error, a drive that switches off, the Arduino's watchdog, a CAN
// send error, a rejected command, an Arduino reset, a lost serial port or feedback older than
// feedback_timeout_ms latches a fault. Streaming stops; the drive holds its last target (or has
// stopped itself on a drive error). Call bring_up() again to resume.

#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>

#include "linmot/serial_bridge.h"

namespace linmot {

// Status word (LinMot Motion Control SW manual, section 3.23).
namespace linmot_sw {
constexpr std::uint16_t kOperationEnabled = 1u << 0;
constexpr std::uint16_t kError = 1u << 3;
constexpr std::uint16_t kSwitchOnLocked = 1u << 6;
constexpr std::uint16_t kWarning = 1u << 7;
constexpr std::uint16_t kInTargetPosition = 1u << 10;
constexpr std::uint16_t kHomed = 1u << 11;
constexpr std::uint16_t kFatalError = 1u << 12;
constexpr std::uint16_t kMotionActive = 1u << 13;
}  // namespace linmot_sw

struct LinMotDriveConfig {
    // Limits enforced by move_to(), in the drive's frame. Keep them inside the drive's own
    // position limits (LinMot-Talk) and the firmware's (5-195 mm, 3 m/s, 20 m/s^2).
    double min_position_m = 0.005;
    double max_position_m = 0.195;
    double max_velocity_mps = 2.0;
    double max_acceleration_mps2 = 20.0;

    bool home_if_needed = true;      // bring_up() homes an unhomed drive; the slide moves

    int ready_timeout_ms = 3000;     // Arduino reset at port open takes about 0.5 s
    int step_timeout_ms = 2000;      // NMT, error acknowledge, switch on (hardware test ~300 ms)
    int home_timeout_ms = 60000;     // homing at 10 mm/s plus the move to the initial position
    int feedback_timeout_ms = 100;   // five periods at 20 ms
};

class LinMotDrive {
public:
    using FaultCallback = std::function<void(const std::string&)>;
    using LogCallback = std::function<void(const std::string&)>;

    // Installs the bridge's reply and cycle callbacks (it has one slot each).
    LinMotDrive(SerialBridge& bridge, LinMotDriveConfig cfg = {});
    ~LinMotDrive();

    // --- state ---------------------------------------------------------------
    bool feedback_valid() const;
    std::uint16_t status_word() const;
    double position_m() const;
    bool enabled() const { return enabled_.load(); }
    bool faulted() const { return fault_.load(); }
    std::string fault_reason() const;
    std::string describe() const;  // one-line human-readable status
    void set_fault_callback(FaultCallback callback);

    // --- commands (blocking; return false and set last_error() on failure) ---
    // Messages about steps that take time (acknowledging, homing) go to log.
    bool bring_up(const LogCallback& log = {});
    // Streams a VAI go-to-position command until the next move_to() or halt().
    bool move_to(double position_m, double max_velocity_mps, double acceleration_mps2);
    // Stops streaming; the drive finishes its last target and holds there, switched on.
    void halt();
    // Halt and switch off; the slide is no longer held.
    bool disable();

    // Abort a blocking call from another thread.
    void request_abort() { abort_.store(true); }

    const std::string& last_error() const { return last_error_; }

private:
    bool fail(const std::string& why);
    void latch_fault(const std::string& why);
    void on_reply(const Reply& reply, std::uint64_t sequence);
    void on_cycle();
    bool wait_status(const char* what, std::uint64_t after, int timeout_ms,
                     const std::function<bool(std::uint16_t)>& pred);

    SerialBridge& bridge_;
    LinMotDriveConfig cfg_;

    std::atomic<bool> enabled_{false};
    std::atomic<bool> fault_{false};
    std::atomic<bool> abort_{false};
    std::atomic<std::uint64_t> restarts_at_enable_{0};
    mutable std::mutex fault_mtx_;
    std::string fault_reason_;
    std::string last_error_;
    std::mutex callback_mtx_;
    FaultCallback fault_callback_;
};

std::string describe_status(std::uint16_t status_word);

}  // namespace linmot
