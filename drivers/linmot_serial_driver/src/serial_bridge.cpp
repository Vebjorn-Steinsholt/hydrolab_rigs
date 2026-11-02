#include "linmot/serial_bridge.h"

#include <algorithm>
#include <utility>

namespace linmot {
namespace {

using Clock = std::chrono::steady_clock;

constexpr std::size_t kRecentFeedback = 64;

int ms_until(Clock::time_point t) {
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t - Clock::now()).count();
    return static_cast<int>(std::max<long long>(ms, 0));
}

}  // namespace

SerialBridge::SerialBridge(BridgeConfig cfg) : cfg_(std::move(cfg)) {}

SerialBridge::~SerialBridge() { stop(); }

bool SerialBridge::start() {
    stop();
    if (!port_.open(cfg_.device, cfg_.baud)) {
        std::lock_guard<std::mutex> lk(mtx_);
        stats_.last_error = port_.last_error();
        return false;
    }
    {
        std::lock_guard<std::mutex> lk(mtx_);
        parser_ = ReplyParser();
        queue_.clear();
        streaming_ = false;
        latest_ = Feedback();
        recent_.clear();
        stats_ = BridgeStats();
        stats_.port_ok = true;
        last_ping_ = Clock::time_point{};
    }
    ready_ = false;
    restarts_ = 0;
    port_ok_ = true;
    running_ = true;
    thread_ = std::thread([this] { run(); });
    return true;
}

void SerialBridge::stop() {
    running_ = false;
    cv_.notify_all();
    if (thread_.joinable()) thread_.join();
    port_.close();
    port_ok_ = false;
}

bool SerialBridge::wait_sent(std::uint64_t sequence, int timeout_ms) {
    std::unique_lock<std::mutex> lk(mtx_);
    return cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms), [&] {
        return last_written_ >= sequence || !port_ok_.load() || !running_.load();
    }) && last_written_ >= sequence;
}

bool SerialBridge::wait_ready(int timeout_ms) {
    std::unique_lock<std::mutex> lk(mtx_);
    return cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms),
                        [this] { return ready_.load() || !running_.load(); }) && ready_.load();
}

std::uint64_t SerialBridge::send(Command command) {
    std::lock_guard<std::mutex> lk(mtx_);
    const std::uint64_t sequence = next_sequence_++;
    queue_.push_back({command, sequence});
    return sequence;
}

void SerialBridge::set_target(std::int32_t position, std::uint32_t velocity,
                              std::uint32_t acceleration, std::uint32_t deceleration) {
    std::lock_guard<std::mutex> lk(mtx_);
    target_.command = Command::GoToPosition;
    target_.position = position;
    target_.velocity = velocity;
    target_.acceleration = acceleration;
    target_.deceleration = deceleration;
    streaming_ = true;
}

void SerialBridge::clear_target() {
    std::lock_guard<std::mutex> lk(mtx_);
    streaming_ = false;
}

bool SerialBridge::streaming() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return streaming_;
}

Feedback SerialBridge::latest() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return latest_;
}

bool SerialBridge::wait_feedback(std::uint64_t min_sequence,
                                 const std::function<bool(const Feedback&)>& pred, int timeout_ms,
                                 const std::atomic<bool>* abort) {
    const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
    std::unique_lock<std::mutex> lk(mtx_);
    if (latest_.valid && latest_.sequence >= min_sequence && pred(latest_)) return true;
    std::uint64_t seen = feedback_count_;
    while (running_ && !(abort && abort->load())) {
        if (cv_.wait_until(lk, deadline) == std::cv_status::timeout && feedback_count_ == seen) {
            return false;
        }
        // recent_ holds the feedbacks numbered feedback_count_ - recent_.size() + 1 .. count.
        const std::uint64_t first = feedback_count_ - recent_.size() + 1;
        for (std::uint64_t n = std::max(seen + 1, first); n <= feedback_count_; ++n) {
            const Feedback& fb = recent_[static_cast<std::size_t>(n - first)];
            if (fb.sequence >= min_sequence && pred(fb)) return true;
        }
        seen = feedback_count_;
    }
    return false;
}

void SerialBridge::set_reply_callback(ReplyCallback callback) {
    std::lock_guard<std::mutex> lk(callback_mtx_);
    reply_callback_ = std::move(callback);
}

void SerialBridge::set_cycle_callback(CycleCallback callback) {
    std::lock_guard<std::mutex> lk(callback_mtx_);
    cycle_callback_ = std::move(callback);
}

BridgeStats SerialBridge::stats() const {
    std::lock_guard<std::mutex> lk(mtx_);
    BridgeStats s = stats_;
    s.dropped_bytes = parser_.dropped_bytes();
    s.bad_crc = parser_.bad_crc();
    s.ready = ready_.load();
    s.port_ok = port_ok_.load();
    return s;
}

void SerialBridge::set_error(const std::string& error) {
    std::lock_guard<std::mutex> lk(mtx_);
    stats_.last_error = error;
}

void SerialBridge::run() {
    auto next_send = Clock::now();
    std::uint8_t buf[256];
    std::vector<Reply> replies;
    while (running_) {
        const auto now = Clock::now();
        if (now >= next_send) {
            send_frame(now);
            next_send += std::chrono::milliseconds(cfg_.period_ms);
            if (next_send <= now) next_send = now + std::chrono::milliseconds(cfg_.period_ms);
            {
                // Held during the call, so clearing the callback waits for a running call.
                std::lock_guard<std::mutex> lk(callback_mtx_);
                if (cycle_callback_) cycle_callback_();
            }
        }
        if (!port_ok_) {
            std::unique_lock<std::mutex> lk(mtx_);
            cv_.wait_until(lk, next_send, [this] { return !running_.load(); });
            continue;
        }
        const long n = port_.read(buf, sizeof(buf), std::max(1, ms_until(next_send)));
        if (n < 0) {
            port_ok_ = false;
            set_error(port_.last_error());
            cv_.notify_all();
            continue;
        }
        if (n == 0) continue;
        replies.clear();
        {
            std::lock_guard<std::mutex> lk(mtx_);
            parser_.feed(buf, static_cast<std::size_t>(n), replies);
        }
        for (const Reply& r : replies) handle(r);
    }
}

void SerialBridge::send_frame(Clock::time_point now) {
    if (!port_ok_) return;
    CommandFrame frame;
    std::uint64_t sent_sequence = 0;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!ready_) {
            if (now - last_ping_ < std::chrono::milliseconds(cfg_.ping_period_ms)) return;
            last_ping_ = now;
            frame.command = Command::Ping;
            frame.sequence = static_cast<std::uint8_t>(next_sequence_);
            last_ping_sequence_ = frame.sequence;
            last_sent_ = next_sequence_++;
        } else if (!queue_.empty()) {
            frame.command = queue_.front().command;
            frame.sequence = static_cast<std::uint8_t>(queue_.front().sequence);
            last_sent_ = queue_.front().sequence;
            queue_.pop_front();
        } else {
            frame = streaming_ ? target_ : CommandFrame{};
            if (!streaming_) frame.command = Command::RequestStatus;
            frame.sequence = static_cast<std::uint8_t>(next_sequence_);
            last_sent_ = next_sequence_++;
        }
        sent_sequence = last_sent_;
        ++stats_.frames_sent;
    }
    const auto bytes = encode(frame);
    if (!port_.write(bytes.data(), bytes.size())) {
        port_ok_ = false;
        set_error(port_.last_error());
        cv_.notify_all();
        return;
    }
    {
        std::lock_guard<std::mutex> lk(mtx_);
        last_written_ = sent_sequence;
    }
    cv_.notify_all();
}

void SerialBridge::handle(const Reply& reply) {
    std::uint64_t sequence;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        ++stats_.replies;
        // The Arduino echoes the low byte of the last command it accepted, which is at most a few
        // frames behind the last one sent.
        const auto behind = static_cast<std::uint8_t>(
            static_cast<std::uint8_t>(last_sent_) - reply.sequence);
        sequence = last_sent_ >= behind ? last_sent_ - behind : 0;

        if (reply.type == ReplyType::Ready) {
            // Pings stop at the first READY, but one may still be in flight; any other READY
            // after that is the Arduino starting up again.
            if (ready_ && reply.sequence != last_ping_sequence_) ++restarts_;
            ready_ = true;
        } else if (reply.type == ReplyType::Feedback) {
            latest_.valid = true;
            latest_.sequence = sequence;
            latest_.position = reply.position;
            latest_.status_word = reply.status_word;
            latest_.flags = reply.flags;
            latest_.bad_frames = reply.bad_frames;
            latest_.received = Clock::now();
            recent_.push_back(latest_);
            if (recent_.size() > kRecentFeedback) recent_.pop_front();
            ++feedback_count_;
        }
    }
    cv_.notify_all();
    std::lock_guard<std::mutex> lk(callback_mtx_);
    if (reply_callback_) reply_callback_(reply, sequence);
}

}  // namespace linmot
