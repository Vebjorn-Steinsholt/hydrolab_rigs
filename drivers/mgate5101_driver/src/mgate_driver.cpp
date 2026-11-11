#include "mgate/mgate_driver.h"

#include <algorithm>
#include <stdexcept>

namespace mgate {
namespace {

constexpr std::uint16_t kMaxReadRegs  = 125;  // Modbus FC3/FC4 limit
constexpr std::uint16_t kMaxWriteRegs = 123;  // Modbus FC16 limit

}  // namespace

MGateDriver::MGateDriver(GatewayConfig cfg) : cfg_(std::move(cfg)) {
    in_.resize(static_cast<std::size_t>(cfg_.input_word_count) * 2);
    out_.resize(static_cast<std::size_t>(cfg_.output_word_count) * 2);
    status_.assign(cfg_.status_word_count, 0);
    client_.set_unit_id(cfg_.unit_id);
    client_.set_timeout(cfg_.timeout_ms);
}

MGateDriver::~MGateDriver() { stop(); }

// --- tag registry ---------------------------------------------------------

void MGateDriver::add_tag(const Tag& t) {
    const std::size_t area_size = (t.area == Area::Input) ? in_.size() : out_.size();
    const std::size_t end       = t.byte_offset + t.size_bytes();
    if (t.size_bytes() == 0) {
        throw std::invalid_argument("mgate: tag '" + t.name + "' has zero size (set raw_len for Raw tags)");
    }
    if (end > area_size) {
        throw std::invalid_argument("mgate: tag '" + t.name + "' at byte " + std::to_string(t.byte_offset) +
                                    " (+" + std::to_string(t.size_bytes()) + ") falls outside the " +
                                    (t.area == Area::Input ? "input" : "output") + " area of " +
                                    std::to_string(area_size) + " bytes");
    }
    if (!tags_.emplace(t.name, t).second) {
        throw std::invalid_argument("mgate: duplicate tag name '" + t.name + "'");
    }
}

void MGateDriver::add_tags(const std::vector<Tag>& tags) {
    for (const Tag& t : tags) add_tag(t);
}

bool MGateDriver::has_tag(const std::string& name) const { return tags_.find(name) != tags_.end(); }

const Tag& MGateDriver::tag(const std::string& name) const {
    auto it = tags_.find(name);
    if (it == tags_.end()) throw std::invalid_argument("mgate: unknown tag '" + name + "'");
    return it->second;
}

const Tag& MGateDriver::tag_checked(const std::string& name, Area expected) const {
    const Tag& t = tag(name);
    if (t.area != expected) {
        throw std::invalid_argument("mgate: tag '" + name + "' is " +
                                    (t.area == Area::Input ? "an input" : "an output") +
                                    " but was accessed as " +
                                    (expected == Area::Input ? "an input" : "an output"));
    }
    return t;
}

// --- lifecycle ------------------------------------------------------------

void MGateDriver::start() {
    if (running_.exchange(true)) return;
    thread_ = std::thread(&MGateDriver::poll_loop, this);
}

void MGateDriver::stop() {
    if (!running_.exchange(false)) return;
    wake_cv_.notify_all();
    if (thread_.joinable()) thread_.join();
    client_.disconnect();
    online_.store(false);
}

void MGateDriver::set_cycle_callback(std::function<void(bool, const Result&)> cb) {
    std::lock_guard<std::mutex> lk(stats_mtx_);
    cycle_cb_ = std::move(cb);
}

bool MGateDriver::wait_for_data(int timeout_ms) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        if (have_data_.load(std::memory_order_acquire)) return true;
        if (!running_.load(std::memory_order_relaxed)) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return have_data_.load(std::memory_order_acquire);
}

// --- reading --------------------------------------------------------------

double MGateDriver::read(const std::string& name) const {
    const Tag&                  t = tag_checked(name, Area::Input);
    std::lock_guard<std::mutex> lk(mtx_);
    return in_.get_scaled(t);
}

std::int64_t MGateDriver::read_int(const std::string& name) const {
    const Tag&                  t = tag_checked(name, Area::Input);
    std::lock_guard<std::mutex> lk(mtx_);
    return in_.get_int(t);
}

bool MGateDriver::read_bool(const std::string& name) const {
    const Tag&                  t = tag_checked(name, Area::Input);
    std::lock_guard<std::mutex> lk(mtx_);
    return in_.get_bool(t);
}

std::vector<std::uint8_t> MGateDriver::read_raw(const std::string& name) const {
    const Tag&                t = tag_checked(name, Area::Input);
    std::vector<std::uint8_t> v(t.size_bytes());
    std::lock_guard<std::mutex> lk(mtx_);
    in_.get_raw(t.byte_offset, v.data(), v.size());
    return v;
}

// --- writing --------------------------------------------------------------

void MGateDriver::touch(std::size_t byte_offset, std::size_t len) {
    if (dirty_lo_ >= dirty_hi_) {
        dirty_lo_ = byte_offset;
        dirty_hi_ = byte_offset + len;
    } else {
        dirty_lo_ = (std::min)(dirty_lo_, byte_offset);
        dirty_hi_ = (std::max)(dirty_hi_, byte_offset + len);
    }
}

void MGateDriver::write(const std::string& name, double engineering_value) {
    const Tag&                  t = tag_checked(name, Area::Output);
    std::lock_guard<std::mutex> lk(mtx_);
    out_.set_scaled(t, engineering_value);
    touch(t.byte_offset, t.size_bytes());
}

void MGateDriver::write_int(const std::string& name, std::int64_t raw) {
    const Tag&                  t = tag_checked(name, Area::Output);
    std::lock_guard<std::mutex> lk(mtx_);
    out_.set_int(t, raw);
    touch(t.byte_offset, t.size_bytes());
}

void MGateDriver::write_bool(const std::string& name, bool v) {
    const Tag&                  t = tag_checked(name, Area::Output);
    std::lock_guard<std::mutex> lk(mtx_);
    out_.set_bool(t, v);
    touch(t.byte_offset, t.size_bytes());
}

void MGateDriver::write_raw(const std::string& name, const std::vector<std::uint8_t>& bytes) {
    const Tag& t = tag_checked(name, Area::Output);
    if (bytes.size() != t.size_bytes()) {
        throw std::invalid_argument("mgate: tag '" + name + "' expects " + std::to_string(t.size_bytes()) +
                                    " bytes, got " + std::to_string(bytes.size()));
    }
    std::lock_guard<std::mutex> lk(mtx_);
    out_.set_raw(t.byte_offset, bytes.data(), bytes.size());
    touch(t.byte_offset, t.size_bytes());
}

double MGateDriver::read_output(const std::string& name) const {
    const Tag&                  t = tag_checked(name, Area::Output);
    std::lock_guard<std::mutex> lk(mtx_);
    return out_.get_scaled(t);
}

std::int64_t MGateDriver::read_output_int(const std::string& name) const {
    const Tag&                  t = tag_checked(name, Area::Output);
    std::lock_guard<std::mutex> lk(mtx_);
    return out_.get_int(t);
}

bool MGateDriver::read_output_bool(const std::string& name) const {
    const Tag&                  t = tag_checked(name, Area::Output);
    std::lock_guard<std::mutex> lk(mtx_);
    return out_.get_bool(t);
}

void MGateDriver::mark_outputs_dirty() {
    std::lock_guard<std::mutex> lk(mtx_);
    dirty_lo_ = 0;
    dirty_hi_ = out_.size();
}

std::vector<std::uint8_t> MGateDriver::input_snapshot() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return in_.bytes();
}

std::vector<std::uint8_t> MGateDriver::output_snapshot() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return out_.bytes();
}

std::vector<std::uint16_t> MGateDriver::status_snapshot() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return status_;
}

DriverStats MGateDriver::stats() const {
    std::lock_guard<std::mutex> lk(stats_mtx_);
    DriverStats s = stats_;
    s.online      = online_.load(std::memory_order_relaxed);
    return s;
}

void MGateDriver::note_error(const Result& r, const char* what) {
    std::lock_guard<std::mutex> lk(stats_mtx_);
    stats_.last_error = std::string(what) + ": " + r.message();
}

// --- poll thread ----------------------------------------------------------

Result MGateDriver::ensure_connected() {
    if (client_.is_connected()) return {};

    Result r = client_.connect(cfg_.host, cfg_.port, cfg_.timeout_ms);
    if (!r) {
        online_.store(false, std::memory_order_relaxed);
        note_error(r, "connect");
        return r;
    }

    client_.set_unit_id(cfg_.unit_id);
    client_.set_timeout(cfg_.timeout_ms);
    {
        std::lock_guard<std::mutex> lk(stats_mtx_);
        ++stats_.reconnects;
    }
    // The gateway's output image may be stale or zeroed after a reconnect;
    // push everything we believe the outputs should be.
    mark_outputs_dirty();
    return {};
}

Result MGateDriver::flush_outputs() {
    if (out_.empty()) return {};

    std::size_t lo = 0, hi = 0;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (cfg_.write_only_dirty) {
            if (dirty_lo_ >= dirty_hi_) return {};  // nothing changed
            lo = dirty_lo_;
            hi = dirty_hi_;
        } else {
            lo = 0;
            hi = out_.size();
        }
    }

    // Grow the span to whole registers.
    const std::size_t first_word = lo / 2;
    const std::size_t last_word  = (hi + 1) / 2;  // exclusive
    const std::size_t word_count = last_word - first_word;

    std::vector<std::uint16_t> regs(word_count);
    {
        std::lock_guard<std::mutex> lk(mtx_);
        out_.store_to_registers(regs.data(), word_count, first_word * 2);
    }

    std::size_t sent = 0;
    while (sent < word_count) {
        const std::uint16_t chunk =
            static_cast<std::uint16_t>((std::min)(static_cast<std::size_t>(kMaxWriteRegs), word_count - sent));
        const std::uint16_t addr =
            static_cast<std::uint16_t>(cfg_.output_base_reg + first_word + sent);

        Result r = (chunk == 1) ? client_.write_single_register(addr, regs[sent])
                                : client_.write_multiple_registers(addr, chunk, regs.data() + sent);
        if (!r) {
            {
                std::lock_guard<std::mutex> lk(stats_mtx_);
                ++stats_.write_errors;
            }
            note_error(r, "write outputs");
            if (r.status == Status::SocketError || r.status == Status::ProtocolError) client_.disconnect();
            return r;
        }
        sent += chunk;
    }

    {
        std::lock_guard<std::mutex> lk(mtx_);
        // Only clear what we actually transmitted; a concurrent write may have
        // extended the dirty span while we were on the wire.
        if (dirty_lo_ >= lo && dirty_hi_ <= hi) {
            dirty_lo_ = dirty_hi_ = 0;
        } else if (dirty_lo_ < hi && dirty_hi_ > hi) {
            dirty_lo_ = hi;
        }
    }
    return {};
}

Result MGateDriver::read_inputs() {
    if (in_.empty()) return {};

    const std::size_t          words = cfg_.input_word_count;
    std::vector<std::uint16_t> regs(words);

    std::size_t got = 0;
    while (got < words) {
        const std::uint16_t chunk =
            static_cast<std::uint16_t>((std::min)(static_cast<std::size_t>(kMaxReadRegs), words - got));
        const std::uint16_t addr = static_cast<std::uint16_t>(cfg_.input_base_reg + got);

        Result r = cfg_.input_uses_fc4 ? client_.read_input_registers(addr, chunk, regs.data() + got)
                                       : client_.read_holding_registers(addr, chunk, regs.data() + got);
        if (!r) {
            {
                std::lock_guard<std::mutex> lk(stats_mtx_);
                ++stats_.read_errors;
            }
            note_error(r, "read inputs");
            if (r.status == Status::SocketError || r.status == Status::ProtocolError) client_.disconnect();
            return r;
        }
        got += chunk;
    }

    {
        std::lock_guard<std::mutex> lk(mtx_);
        in_.load_from_registers(regs.data(), words, 0);
    }
    return {};
}

Result MGateDriver::read_status() {
    if (cfg_.status_word_count == 0) return {};

    const std::size_t          words = cfg_.status_word_count;
    std::vector<std::uint16_t> regs(words);

    std::size_t got = 0;
    while (got < words) {
        const std::uint16_t chunk =
            static_cast<std::uint16_t>((std::min)(static_cast<std::size_t>(kMaxReadRegs), words - got));
        const std::uint16_t addr = static_cast<std::uint16_t>(cfg_.status_base_reg + got);

        Result r = cfg_.status_uses_fc4 ? client_.read_input_registers(addr, chunk, regs.data() + got)
                                        : client_.read_holding_registers(addr, chunk, regs.data() + got);
        if (!r) {
            note_error(r, "read status");
            if (r.status == Status::SocketError || r.status == Status::ProtocolError) client_.disconnect();
            return r;
        }
        got += chunk;
    }

    std::lock_guard<std::mutex> lk(mtx_);
    status_.swap(regs);
    return {};
}

Result MGateDriver::exchange() {
    // Times each request, so a slow cycle shows which one stalled (DriverStats::slow_*_us).
    phase_us_= {};
    const auto timed = [](long long& us, auto&& request){
        const auto start = std::chrono::steady_clock::now();
        Result r = request();
        us = std::chrono::duration_cast<std::chrono::microseconds>(
                 std::chrono::steady_clock::now() - start)
                 .count();
        return r;
    };
    if (Result r = timed(phase_us_[0], [this] { return flush_outputs(); }); !r) return r;
    if (Result r = timed(phase_us_[1], [this] { return read_inputs(); }); !r) return r;
    if (Result r = timed(phase_us_[2], [this] { return read_status(); }); !r) return r;
    return {};
}

void MGateDriver::poll_loop() {
    auto next = std::chrono::steady_clock::now();

    while (running_.load(std::memory_order_relaxed)) {
        const auto cycle_start = std::chrono::steady_clock::now();

        Result last = ensure_connected();
        bool   ok   = false;
        if (last) {
            last = exchange();
            ok   = static_cast<bool>(last);
            online_.store(ok, std::memory_order_relaxed);
            // Only after a whole cycle (outputs, inputs and status) and online_: wait_for_data()
            // callers may check online() and the live list at once. The release also publishes
            // the online_ store above.
            if (ok) have_data_.store(true, std::memory_order_release);
        } else {
            have_data_.store(false, std::memory_order_release);
        }

        {
            std::lock_guard<std::mutex> lk(stats_mtx_);
            ++stats_.cycles;
            stats_.last_cycle_us = std::chrono::duration_cast<std::chrono::microseconds>(
                                       std::chrono::steady_clock::now() - cycle_start)
                                       .count();
            if (ok) stats_.last_error.clear();
            stats_.max_cycle_us = std::max(stats_.max_cycle_us,stats_.last_cycle_us);
            if (stats_.last_cycle_us > 1000LL * cfg_.slow_cycle_ms) {
                ++stats_.slow_cycles;
                stats_.slow_cycle_us  = stats_.last_cycle_us;
                stats_.slow_write_us  = phase_us_[0];
                stats_.slow_input_us  = phase_us_[1];
                stats_.slow_status_us = phase_us_[2];
            }
        }
        {
            std::function<void(bool, const Result&)> cb;
            {
                std::lock_guard<std::mutex> lk(stats_mtx_);
                cb = cycle_cb_;
            }
            if (cb) cb(ok, last);
        }

        const int wait_ms = ok ? cfg_.poll_interval_ms : cfg_.reconnect_ms;
        next += std::chrono::milliseconds(wait_ms);
        const auto now = std::chrono::steady_clock::now();
        if (next < now) next = now;  // we overran; do not spin trying to catch up

        std::unique_lock<std::mutex> lk(wake_mtx_);
        wake_cv_.wait_until(lk, next, [this] { return !running_.load(std::memory_order_relaxed); });
    }

    online_.store(false, std::memory_order_relaxed);
    have_data_.store(false, std::memory_order_release);
}

}  // namespace mgate
