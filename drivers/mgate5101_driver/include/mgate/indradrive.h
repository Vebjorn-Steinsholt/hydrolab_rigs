// indradrive.h - Rexroth IndraDrive (MPx04 firmware) on top of MGateDriver.
//
// Assumes the drive runs the freely configurable field bus profile
// (P-0-4084 = 0xFFFE) in "drive-internal interpolation" (S-0-0032) with this
// cyclic layout (P-0-4080 / P-0-4081):
//
//   inputs  (drive -> PC)              outputs (PC -> drive)
//   0  P-0-4078 status word   U16      0  P-0-4077 control word      U16
//   2  S-0-0386 position      I32      2  S-0-0258 target position   I32
//   6  S-0-0040 velocity      I32      6  S-0-0259 pos. velocity     I32
//   10 S-0-0390 diag number   U32      10 S-0-0108 feedrate override U16
//
// 32-bit values are high word first (MPx04 Functional Description,
// "Drive-Internal Interpolation"). The drive follows S-0-0258 directly; there
// is no acceptance handshake in this operating mode.
//
// Safety: whenever the Modbus link fails, the PROFIBUS slave drops out of the
// gateway's live list, or the drive reports a class 1 error, the control word
// is forced to 0 and the object latches a fault. A reconnect therefore sends
// "off", never the last run command (MPx04 manual: "Automatic restart after
// bus failure!"). Call enable() again to resume.

#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>

#include "mgate/mgate_driver.h"

namespace mgate {

// P-0-4077, Field bus: control word (Rexroth profile).
namespace indra_cw {
constexpr std::uint16_t kOperatingMode = 1u << 1;   // 1 = operating mode, 0 = parameter mode
constexpr std::uint16_t kClearErrors   = 1u << 5;   // 0->1 starts C5 "clear errors"
constexpr std::uint16_t kDriveStart    = 1u << 13;  // 0 = Drive Halt, 1 = run
constexpr std::uint16_t kDriveOn       = 1u << 15;  // controller enable
}  // namespace indra_cw

// P-0-4078, Field bus: status word.
namespace indra_sw {
constexpr std::uint16_t kModeMask        = 0x0003;   // bits 1/0
constexpr std::uint16_t kModeOperating   = 0x0002;   // 10 = operating mode
constexpr std::uint16_t kInStandstill    = 1u << 3;
constexpr std::uint16_t kTargetReached   = 1u << 4;
constexpr std::uint16_t kModeError       = 1u << 6;
constexpr std::uint16_t kNotFollowing    = 1u << 7;  // 1 = ignores command values (e.g. halt)
constexpr std::uint16_t kClass2Warning   = 1u << 12;
constexpr std::uint16_t kClass1Error     = 1u << 13;
constexpr std::uint16_t kReadyMask       = 0xC000;   // bits 15/14
constexpr std::uint16_t kReadyBb         = 0x4000;   // 01 ready for power on
constexpr std::uint16_t kReadyAb         = 0x8000;   // 10 control + power section ready
constexpr std::uint16_t kReadyAF         = 0xC000;   // 11 in operation, with torque
}  // namespace indra_sw

struct IndraDriveConfig {
    std::string name_prefix = "Drv.";  // tag names: <prefix>StatusWord etc.
    int         profibus_address = 2;  // for the gateway live list check

    // Command limits enforced by move_to(). Position units are selected by
    // the controller: degrees for angular drives and meters for linear drives.
    double min_position = -550.0;
    double max_position = 500.0;
    double max_velocity_rpm = 100.0;
    double in_position_tol_deg = 0.1;  // S-0-0057 position window

    int step_timeout_ms = 10000;  // per state-machine step in enable()/disable()
};

class IndraDrive {
public:
    using FaultCallback = std::function<void(const std::string&)>;

    // Registers the drive's tags on `drv`. Call before drv.start().
    // Installs drv's cycle callback (the driver has one slot).
    IndraDrive(MGateDriver& drv, IndraDriveConfig cfg = {});

    // --- state -------------------------------------------------------------
    std::uint16_t status_word() const;
    double        position_deg() const;
    double        velocity_rpm() const;
    std::uint32_t diag_number() const;
    bool          slave_live() const;   // from the gateway's PROFIBUS live list
    bool          faulted() const { return fault_.load(); }
    std::string   fault_reason() const;
    std::string   describe() const;     // one-line human-readable status
    void          set_fault_callback(FaultCallback callback);

    // --- commands (blocking; return false and log why on failure) ----------
    // Pulse the clear-errors bit and wait for the class 1 error to go away.
    bool clear_errors();
    // Operating mode -> drive ON (halted) -> drive start. Target is set to the
    // current position first so the axis does not jump.
    bool enable();
    // Stage a new target. The drive starts moving on the next cycle.
    bool move_to(double position_deg, double velocity_rpm);
    bool wait_in_position(int timeout_ms);
    // Drive Halt (stays powered, holds position).
    void halt();
    // Halt, wait for standstill, remove drive ON. Stays in operating mode.
    bool disable();
    // Control word 0: drive off and back to parameter mode (as found at start).
    void shutdown();

    // Abort any blocking call from another thread (e.g. Ctrl+C handler).
    void request_abort() { abort_.store(true); }

    std::string last_error() const;

private:
    std::string tag(const char* s) const { return cfg_.name_prefix + s; }
    void        set_control(std::uint16_t cw);
    void        latch_fault(const std::string& why);
    // stoppable = false for the shutdown path: keep waiting after abort/fault.
    bool        wait_for(const char* what, int timeout_ms, bool (*pred)(std::uint16_t), bool stoppable = true);
    bool        fail(const std::string& why);
    void        on_cycle(bool ok);

    MGateDriver&     drv_;
    IndraDriveConfig cfg_;

    std::atomic<bool> enabled_{false};  // we are commanding drive ON
    std::atomic<bool> fault_{false};
    std::atomic<bool> abort_{false};
    mutable std::mutex fault_mtx_;
    std::string        fault_reason_;
    mutable std::mutex error_mtx_;
    std::string        last_error_;
    mutable std::mutex callback_mtx_;
    FaultCallback      fault_callback_;
};

}  // namespace mgate
