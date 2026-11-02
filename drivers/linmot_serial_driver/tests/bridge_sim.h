// bridge_sim.h - the Arduino bridge and a LinMot drive, simulated on a pseudo-terminal.
//
// SerialBridge opens device(); the simulator answers on the other end like the firmware does:
// start-up text, READY, one feedback per command (two for ACK ERROR and for SWITCH ON from a
// switched-off drive), with a drive that switches on, homes, follows targets and reports errors.
// After PRE OPERATIONAL it sends nothing until the next OPERATIONAL, like the real drive.

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include "linmot/linmot_drive.h"
#include "linmot/protocol.h"

namespace linmot_test {

using linmot::Command;
using linmot::CommandFrame;
using linmot::Reply;
using linmot::ReplyType;
namespace sw = linmot::linmot_sw;

class BridgeSim {
public:
    struct Options {
        int boot_ms = 50;              // READY this long after start, after the start-up text
        bool send_ready = true;        // false: the Arduino never answers
        bool drive_on_can = true;      // false: no feedback at all
        bool ignore_switch_on = false; // LinMot-Talk holds the control word
        int home_ms = 100;
        double home_end_m = 0.1;       // the drive's initial position after homing
    };

    explicit BridgeSim(Options options) : opt_(options) {
        master_ = posix_openpt(O_RDWR | O_NOCTTY);
        grantpt(master_);
        unlockpt(master_);
        device_ = ptsname(master_);
        termios tio{};
        tcgetattr(master_, &tio);
        cfmakeraw(&tio);
        tcsetattr(master_, TCSANOW, &tio);
        // Keep a slave descriptor open so the master never sees a hang-up between opens.
        hold_ = open(device_.c_str(), O_RDWR | O_NOCTTY);
        running_ = true;
        thread_ = std::thread([this] { run(); });
    }

    ~BridgeSim() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
        close(hold_);
        close(master_);
    }

    const std::string& device() const { return device_; }

    // --- drive state, for tests ----------------------------------------------
    void set_state(bool enabled, bool homed, bool error = false, bool fatal = false) {
        std::lock_guard<std::mutex> lk(mtx_);
        enabled_ = enabled;
        homed_ = homed;
        error_ = error;
        fatal_ = fatal;
    }
    void raise_error() { std::lock_guard<std::mutex> lk(mtx_); error_ = true; enabled_ = false; }
    void switch_off_externally() { std::lock_guard<std::mutex> lk(mtx_); enabled_ = false; }
    void set_flags_once(std::uint8_t flags) { std::lock_guard<std::mutex> lk(mtx_); flags_ |= flags; }
    void set_silent(bool silent) { silent_ = silent; }
    // Sends READY on its own, as the firmware does after a reset.
    void restart() {
        std::lock_guard<std::mutex> lk(mtx_);
        Reply r;
        r.type = ReplyType::Ready;
        r.sequence = 0;
        send(r);
    }

    std::vector<CommandFrame> commands() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return commands_;
    }
    int count(Command c) const {
        std::lock_guard<std::mutex> lk(mtx_);
        int n = 0;
        for (const auto& f : commands_) n += (f.command == c);
        return n;
    }
    std::int32_t position() const { std::lock_guard<std::mutex> lk(mtx_); return position_; }
    bool operational() const { std::lock_guard<std::mutex> lk(mtx_); return operational_; }

private:
    void send(const Reply& r) {
        const auto bytes = linmot::encode_reply(r);
        (void)!write(master_, bytes.data(), bytes.size());
    }

    std::uint16_t status() const {
        std::uint16_t s = 0;
        if (enabled_) s |= sw::kOperationEnabled;
        if (error_) s |= sw::kError;
        if (fatal_) s |= sw::kFatalError;
        if (homed_) s |= sw::kHomed;
        if (homing_) s |= sw::kMotionActive;
        if (!homed_) s |= sw::kWarning;
        return s;
    }

    void feedback() {
        if (!opt_.drive_on_can || silent_ || !operational_) return;
        Reply r;
        r.type = ReplyType::Feedback;
        r.sequence = last_sequence_;
        r.position = position_;
        r.status_word = status();
        r.flags = flags_;
        flags_ &= static_cast<std::uint8_t>(~(linmot::flag::kCanSendError | linmot::flag::kRejected |
                                              linmot::flag::kWatchdogTripped));
        send(r);
    }

    void handle(const CommandFrame& f) {
        std::lock_guard<std::mutex> lk(mtx_);
        commands_.push_back(f);
        if (silent_ && f.command != Command::Ping) return;
        last_sequence_ = f.sequence;
        switch (f.command) {
            case Command::Ping: {
                if (!booted_) return;
                Reply r;
                r.type = ReplyType::Ready;
                r.sequence = f.sequence;
                send(r);
                return;
            }
            case Command::Operational: operational_ = true; break;
            case Command::Home:
                if (enabled_) {
                    homing_ = true;
                    home_done_ = std::chrono::steady_clock::now() +
                                 std::chrono::milliseconds(opt_.home_ms);
                }
                break;
            case Command::GoToPosition: position_ = f.position; break;
            case Command::SwitchOn:
                if (!enabled_) feedback();  // the firmware clears bit 0 first
                if (!opt_.ignore_switch_on && !error_) enabled_ = true;
                break;
            case Command::SwitchOff: enabled_ = false; break;
            case Command::RequestStatus: break;
            case Command::AckError:
                feedback();
                if (!fatal_) error_ = false;
                enabled_ = false;
                break;
            case Command::PreOperational:
                operational_ = false;  // no PDOs from now on, so no feedback either
                return;
        }
        feedback();
    }

    void run() {
        const auto start = std::chrono::steady_clock::now();
        std::vector<std::uint8_t> buf;
        while (running_) {
            if (!booted_ && opt_.send_ready &&
                std::chrono::steady_clock::now() - start > std::chrono::milliseconds(opt_.boot_ms)) {
                std::lock_guard<std::mutex> lk(mtx_);
                const char text[] = "Entering Configuration Mode Successful!\r\n";
                (void)!write(master_, text, sizeof(text) - 1);
                booted_ = true;
                Reply r;
                r.type = ReplyType::Ready;
                send(r);
            }
            {
                std::lock_guard<std::mutex> lk(mtx_);
                if (homing_ && std::chrono::steady_clock::now() >= home_done_) {
                    homing_ = false;
                    homed_ = true;
                    position_ = static_cast<std::int32_t>(opt_.home_end_m * linmot::kPositionPerMetre);
                }
            }
            pollfd p{master_, POLLIN, 0};
            if (poll(&p, 1, 5) <= 0 || !(p.revents & POLLIN)) continue;
            std::uint8_t tmp[256];
            const ssize_t n = read(master_, tmp, sizeof(tmp));
            if (n <= 0) continue;
            buf.insert(buf.end(), tmp, tmp + n);
            while (buf.size() >= linmot::kCommandFrameSize) {
                if (buf[0] != linmot::kSync1 || buf[1] != linmot::kSync2) {
                    buf.erase(buf.begin());
                    continue;
                }
                CommandFrame f;
                if (!linmot::decode_command(buf.data(), f)) {
                    buf.erase(buf.begin());
                    continue;
                }
                buf.erase(buf.begin(), buf.begin() + linmot::kCommandFrameSize);
                if (booted_ || f.command == Command::Ping) handle(f);
            }
        }
    }

    Options opt_;
    int master_ = -1;
    int hold_ = -1;
    std::string device_;
    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> silent_{false};

    mutable std::mutex mtx_;
    bool booted_ = false;
    bool operational_ = false;
    bool enabled_ = false;
    bool homed_ = false;
    bool homing_ = false;
    bool error_ = false;
    bool fatal_ = false;
    std::uint8_t flags_ = 0;
    std::uint8_t last_sequence_ = 0;
    std::int32_t position_ = 470;  // 0.047 mm, as an unhomed drive reported on the rig
    std::chrono::steady_clock::time_point home_done_{};
    std::vector<CommandFrame> commands_;
};

}  // namespace linmot_test
