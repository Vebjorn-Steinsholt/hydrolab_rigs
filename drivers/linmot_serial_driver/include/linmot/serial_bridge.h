// serial_bridge.h - cyclic link to the LinMot serial-to-CAN bridge (an Arduino on USB serial).
//
// A background thread sends exactly one frame per period:
//   before READY ... PING every ping_period_ms (opening the port resets an Uno; it answers
//                    after about 0.5 s, its start-up text is skipped by the parser)
//   after READY .... the oldest queued command, else GO TO POSITION with the streamed target,
//                    else REQUEST STATUS
// Every frame after READY ends in a CAN SYNC, so the drive answers each with a feedback, and the
// Arduino's watchdog (200 ms without a valid frame) only trips when this side stops sending.
//
// Sequence numbers are counted here as 64-bit values; replies carry the low byte of the last
// command the Arduino accepted, which is extended back to 64 bits. A feedback with
// sequence >= n therefore shows the drive's state after command n.

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include "linmot/protocol.h"
#include "linmot/serial_port.h"

namespace linmot {

struct BridgeConfig {
    std::string device = "/dev/ttyACM0";
    int baud = 115200;
    int period_ms = 20;       // one frame per period; 20 ms leaves room for the ~11 ms round trip
    int ping_period_ms = 200;
};

struct Feedback {
    bool valid = false;               // a feedback has arrived since start()
    std::uint64_t sequence = 0;       // extended sequence of the last command the Arduino accepted
    std::int32_t position = 0;        // 0.1 um
    std::uint16_t status_word = 0;
    std::uint8_t flags = 0;
    std::uint8_t bad_frames = 0;      // frames the Arduino dropped, as it counts them
    std::chrono::steady_clock::time_point received{};
};

struct BridgeStats {
    std::uint64_t frames_sent = 0;
    std::uint64_t replies = 0;
    std::uint64_t dropped_bytes = 0;  // before READY this includes the Arduino's start-up text
    std::uint64_t bad_crc = 0;
    bool ready = false;
    bool port_ok = false;
    std::string last_error;
};

class SerialBridge {
public:
    // Callbacks run on the bridge thread under a lock that set_*_callback() also takes, so after
    // set_*_callback(nullptr) returns, no call is running. They must not set callbacks themselves.
    // Called for every valid reply, with its extended sequence number.
    using ReplyCallback = std::function<void(const Reply&, std::uint64_t sequence)>;
    // Called once per period, after that period's frame was sent.
    using CycleCallback = std::function<void()>;

    explicit SerialBridge(BridgeConfig cfg);
    ~SerialBridge();

    SerialBridge(const SerialBridge&) = delete;
    SerialBridge& operator=(const SerialBridge&) = delete;

    // Opens the port and starts the thread. False (see stats().last_error) if the port does not
    // open.
    bool start();
    void stop();

    bool ready() const { return ready_.load(); }
    bool port_ok() const { return port_ok_.load(); }
    // How often the Arduino sent READY on its own after the first READY, i.e. was reset. Its
    // watchdog and NMT state are gone then; the drive keeps its own state.
    std::uint64_t restarts() const { return restarts_.load(); }
    bool wait_ready(int timeout_ms);
    // Waits until the frame with this sequence number has been written to the port, for
    // commands without a reply. False on timeout or when the port fails.
    bool wait_sent(std::uint64_t sequence, int timeout_ms);

    // Queues a one-shot command for the next free period; returns its sequence number.
    std::uint64_t send(Command command);
    // Streams a go-to-position command with these values every period until clear_target().
    void set_target(std::int32_t position, std::uint32_t velocity, std::uint32_t acceleration,
                    std::uint32_t deceleration);
    void clear_target();
    bool streaming() const;

    Feedback latest() const;
    // Waits until a feedback with sequence >= min_sequence satisfies pred (checked on every
    // feedback as it arrives). False on timeout, abort, or when the thread stops.
    bool wait_feedback(std::uint64_t min_sequence, const std::function<bool(const Feedback&)>& pred,
                       int timeout_ms, const std::atomic<bool>* abort = nullptr);

    void set_reply_callback(ReplyCallback callback);
    void set_cycle_callback(CycleCallback callback);

    BridgeStats stats() const;

private:
    struct Queued {
        Command command;
        std::uint64_t sequence;
    };

    void run();
    void send_frame(std::chrono::steady_clock::time_point now);
    void handle(const Reply& reply);
    void set_error(const std::string& error);

    BridgeConfig cfg_;
    SerialPort port_;
    ReplyParser parser_;

    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> ready_{false};
    std::atomic<bool> port_ok_{false};
    std::atomic<std::uint64_t> restarts_{0};

    mutable std::mutex mtx_;
    std::condition_variable cv_;
    std::uint64_t next_sequence_ = 1;
    std::uint64_t last_sent_ = 0;
    std::uint64_t last_written_ = 0;   // sequence of the last frame written to the port
    std::deque<Queued> queue_;
    bool streaming_ = false;
    CommandFrame target_;
    Feedback latest_;
    // Every feedback since the last check, so wait_feedback() sees each one.
    std::uint64_t feedback_count_ = 0;
    std::deque<Feedback> recent_;
    BridgeStats stats_;
    std::chrono::steady_clock::time_point last_ping_{};
    std::uint8_t last_ping_sequence_ = 0;  // a READY with this sequence answers our ping

    std::mutex callback_mtx_;
    ReplyCallback reply_callback_;
    CycleCallback cycle_callback_;
};

}  // namespace linmot
