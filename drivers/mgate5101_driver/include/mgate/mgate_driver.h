// mgate_driver.h - cyclic Modbus/TCP driver for a Moxa MGate 5101-PBM-MN.
//
// Roles:
//   PC (this code) .... Modbus TCP CLIENT  (master)
//   MGate 5101 ........ Modbus TCP SERVER  (slave)  +  PROFIBUS DP CLASS-1 MASTER
//   Field devices ..... PROFIBUS DP slaves (their GSD files are loaded into the MGate)
//
// A background thread runs a fixed-period exchange:
//   1. flush any changed output bytes  -> FC16 (or FC6 for a single register)
//   2. read the whole input area       -> FC4 (or FC3)
// Application threads only ever touch cached process images under a mutex, so
// read()/write() never block on the network.

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <array>

#include "mgate/modbus_tcp.h"
#include "mgate/process_image.h"

namespace mgate {

struct GatewayConfig {
    std::string   host    = "192.168.127.254";  // MGate factory default
    std::uint16_t port    = 502;
    std::uint8_t  unit_id = 1;  // Modbus slave ID configured on the gateway

    // ---------------------------------------------------------------------
    // Read these four numbers off the gateway, do not guess them.
    // Web console -> Protocol Settings -> Modbus TCP -> "Modbus address map"
    // (MGate Manager shows the same table). The page lists, per PROFIBUS slave
    // and per GSD module, the internal byte offset and the Modbus address the
    // gateway assigned. See MAPPING.md.
    //
    // NOTE: the manual's 4xxxxx / 3xxxxx style addresses are 1-based.
    // Register 40001 is address 0x0000 on the wire. Subtract one.
    // ---------------------------------------------------------------------
    std::uint16_t input_base_reg    = 0x0000;  // first register of the PROFIBUS input area
    std::uint16_t input_word_count  = 0;       // area size in 16-bit registers
    std::uint16_t output_base_reg   = 0x0800;  // first register of the PROFIBUS output area
    std::uint16_t output_word_count = 0;

    // Inputs are usually exposed as input registers (FC4) and also mirrored
    // into holding registers (FC3). Set false to use FC3 for the input area.
    bool input_uses_fc4 = true;

    // Optional status/diagnostic block (slave live list etc.). Leave count at 0
    // to skip. Address is model- and firmware-specific: take it from the same
    // address-map page.
    std::uint16_t status_base_reg   = 0x0000;
    std::uint16_t status_word_count = 0;
    bool          status_uses_fc4   = true;

    int poll_interval_ms = 50;    // cycle period
    int timeout_ms       = 1000;  // per-transaction socket timeout
    int reconnect_ms     = 2000;  // wait before retrying a dead connection
    int slow_cycle_ms    = 100; // a longer cycle counts in DriverStats::slow_cycles

    // If true only the changed span of the output area is written each cycle.
    // If false the whole output area is written every cycle.
    bool write_only_dirty = true;
};

struct DriverStats {
    std::uint64_t cycles          = 0;
    std::uint64_t read_errors     = 0;
    std::uint64_t write_errors    = 0;
    std::uint64_t reconnects      = 0;
    long long     last_cycle_us   = 0;
    long long     max_cycle_us    = 0;
    std::uint64_t slow_cycles     = 0; // cycles longer than GatewayConfig::slow_cycle_ms
    // The last slow cycle, and its three requests, in microseconds
    long long    slow_cycle_us    = 0;
    long long    slow_write_us    = 0;
    long long    slow_input_us    = 0;
    long long    slow_status_us   = 0;
    std::string   last_error;
    bool          online          = false;
};

class MGateDriver {
public:
    explicit MGateDriver(GatewayConfig cfg);
    ~MGateDriver();

    MGateDriver(const MGateDriver&)            = delete;
    MGateDriver& operator=(const MGateDriver&) = delete;

    // --- tag registry (populate before start(), it is not locked) ----------
    void       add_tag(const Tag& t);
    void       add_tags(const std::vector<Tag>& tags);
    const Tag& tag(const std::string& name) const;
    bool       has_tag(const std::string& name) const;

    // --- lifecycle ---------------------------------------------------------
    void start();
    void stop();
    bool online() const noexcept { return online_.load(std::memory_order_relaxed); }

    // Blocks until at least one full successful cycle (outputs, inputs, status) has happened,
    // so online() is true and the status area is filled when it returns true.
    bool wait_for_data(int timeout_ms);

    // Called from the poll thread after every cycle. Keep it short.
    void set_cycle_callback(std::function<void(bool /*ok*/, const Result&)> cb);

    // --- reading inputs ----------------------------------------------------
    double       read(const std::string& name) const;       // scaled engineering value
    std::int64_t read_int(const std::string& name) const;   // raw integer
    bool         read_bool(const std::string& name) const;
    std::vector<std::uint8_t> read_raw(const std::string& name) const;

    // --- writing outputs (staged, sent on the next cycle) ------------------
    void write(const std::string& name, double engineering_value);
    void write_int(const std::string& name, std::int64_t raw);
    void write_bool(const std::string& name, bool v);
    void write_raw(const std::string& name, const std::vector<std::uint8_t>& bytes);

    // Reads back what is currently staged in the output image.
    double       read_output(const std::string& name) const;
    std::int64_t read_output_int(const std::string& name) const;
    bool         read_output_bool(const std::string& name) const;

    // Force a full re-transmit of the output area on the next cycle.
    void mark_outputs_dirty();

    // --- whole-area access -------------------------------------------------
    std::vector<std::uint8_t> input_snapshot() const;
    std::vector<std::uint8_t> output_snapshot() const;
    std::vector<std::uint16_t> status_snapshot() const;

    DriverStats stats() const;

    // The configuration this driver was built with (block sizes for IndraDrive's check).
    const GatewayConfig& config() const { return cfg_; }

private:
    void   poll_loop();
    Result ensure_connected();
    Result exchange();  // one full cycle
    Result flush_outputs();
    Result read_inputs();
    Result read_status();
    void   note_error(const Result& r, const char* what);
    void touch(std::size_t byte_offset, std::size_t len);

    const Tag& tag_checked(const std::string& name, Area expected) const;

    GatewayConfig   cfg_;
    ModbusTcpClient client_;

    std::map<std::string, Tag> tags_;

    mutable std::mutex         mtx_;
    ProcessImage               in_;
    ProcessImage               out_;
    std::vector<std::uint16_t> status_;
    std::size_t                dirty_lo_ = 0;  // byte range, half-open
    std::size_t                dirty_hi_ = 0;

    std::thread             thread_;
    std::atomic<bool>       running_{false};
    std::atomic<bool>       online_{false};
    std::atomic<bool>       have_data_{false};
    std::mutex              wake_mtx_;
    std::condition_variable wake_cv_;
    std::array<long long, 3> phase_us_{};  // write, inputs, status of this cycle (poll thread)

    mutable std::mutex                                 stats_mtx_;
    DriverStats                                        stats_;
    std::function<void(bool, const Result&)>           cycle_cb_;
};

}  // namespace mgate
