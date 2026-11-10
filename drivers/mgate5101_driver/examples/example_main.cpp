// example_main.cpp - Rexroth IndraDrive (HCS03 / CSB01.1C-PB, MPB-04) behind
// an MGate 5101-PBM-MN at 192.168.1.70.
//
//   mgate_example            monitor slave 2 (the wavemaker) at registers 0 / 0
//   mgate_example --input-base 7 --output-base 6 --profibus-address 3
//                            monitor slave 3 (the vertical oscillator, daisy chain)
//   mgate_example --move --profibus-address N [--input-base I --output-base O]
//                            clears errors, enables, moves +5 units and back at
//                            10 units, disables. Ctrl+C halts and disables.
//
// The monitor does not command a move, but it is not read only: it writes the
// whole output block (control word 0) every cycle and at exit. Never run it on
// a block whose rig is running. --move needs --profibus-address so it cannot
// pick a drive by default.
//
// Before --move: set a fault timeout (200-500 ms, fault value 00) on the
// MGate's "Output 6 Words" module, and keep the E-stop within reach.
//
// Modbus (MGate manual p.37, register = byte offset / 2): reads at 0x0000 hit
// the input area, writes at 0x0000 hit the output area. Never write register
// 0x0300 - that is the gateway's PROFIBUS master control word.

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include "mgate/indradrive.h"
#include "mgate/mgate_driver.h"

using namespace mgate;

namespace {

std::atomic<IndraDrive*> g_drive{nullptr};

void on_sigint(int) {
    if (IndraDrive* d = g_drive.load()) d->request_abort();
}

void sleep_ms(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

void usage() {
    std::fprintf(stderr,
                 "usage: mgate_example [--move] [--input-base REG] [--output-base REG]\n"
                 "                     [--profibus-address N]\n"
                 "  defaults: registers 0 / 0, address 2 (the wavemaker); the vertical\n"
                 "  oscillator is --input-base 7 --output-base 6 --profibus-address 3.\n"
                 "  --move requires --profibus-address.\n");
}

// Parses a whole decimal number in [lo, hi]; false on anything else.
bool parse_int(const char* text, long lo, long hi, long& out) {
    char* end = nullptr;
    const long v = std::strtol(text, &end, 10);
    if (end == text || *end != '\0' || v < lo || v > hi) return false;
    out = v;
    return true;
}

int monitor(MGateDriver& drv, IndraDrive& drive) {
    for (int i = 0; i < 50; ++i) {
        if (drv.online())
            std::printf("%s\n", drive.describe().c_str());
        else
            std::printf("offline: %s\n", drv.stats().last_error.c_str());
        sleep_ms(200);
    }
    return 0;
}

// Hardware test checklist for --move:
// - Verify the gateway address map, unit ID, input/output sizes, and byte order.
// - Confirm the drive is mechanically clear, the E-stop is reachable, and the
//   MGate output fault timeout is configured before enabling the axis.
// - Run monitor mode first and confirm the status word, position, velocity,
//   diagnostic number, and PROFIBUS live bit are sensible.
// - Start with the configured low speed and a small step; confirm the axis
//   stops at the target, returns to its starting position, and disables cleanly.
// - Interrupt the test or disconnect the gateway and verify the control word
//   goes to zero and the drive does not restart without enable().
int test_move(IndraDrive& drive) {
    constexpr double kStepDeg = 5.0;
    constexpr double kRpm     = 10.0;

    std::printf("before:  %s\n", drive.describe().c_str());

    if (drive.status_word() & indra_sw::kClass1Error) {
        std::printf("drive has an error, clearing...\n");
        if (!drive.clear_errors()) return 1;
        std::printf("cleared: %s\n", drive.describe().c_str());
    }

    std::printf("enabling...\n");
    if (!drive.enable()) return 1;
    std::printf("enabled: %s\n", drive.describe().c_str());

    const double start = drive.position_deg();
    const double moves[] = {start + kStepDeg, start};
    for (double target : moves) {
        std::printf("move to %.4f deg at %.1f rpm\n", target, kRpm);
        if (!drive.move_to(target, kRpm) || !drive.wait_in_position(15000)) return 1;
        std::printf("reached: %s\n", drive.describe().c_str());
        sleep_ms(500);
    }
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    bool do_move = false;
    long input_base = 0, output_base = 0, profibus_address = -1;  // -1: not given
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        long* target = nullptr;
        long lo = 0, hi = 0;
        if (a == "--move") {
            do_move = true;
            continue;
        } else if (a == "--input-base") {
            target = &input_base; hi = 0x02FF;   // below the status area at 0x0300
        } else if (a == "--output-base") {
            target = &output_base; hi = 0x02FF;
        } else if (a == "--profibus-address") {
            target = &profibus_address; lo = 1; hi = 125;
        } else if (a == "--help" || a == "-h") {
            usage();
            return 0;
        } else {
            std::fprintf(stderr, "unknown option %s\n", argv[i]);
            usage();
            return 2;
        }
        if (i + 1 >= argc || !parse_int(argv[i + 1], lo, hi, *target)) {
            std::fprintf(stderr, "%s needs a number in %ld..%ld\n", a.c_str(), lo, hi);
            return 2;
        }
        ++i;
    }
    if (do_move && profibus_address < 0) {
        std::fprintf(stderr, "--move needs --profibus-address (2 wavemaker, 3 oscillator)\n");
        return 2;
    }
    if (profibus_address < 0) profibus_address = 2;
    std::printf("registers in %ld / out %ld, PROFIBUS address %ld\n",
                input_base, output_base, profibus_address);

    GatewayConfig cfg;
    cfg.host    = "192.168.1.70";
    cfg.port    = 502;
    cfg.unit_id = 1;

    cfg.input_base_reg    = static_cast<std::uint16_t>(input_base);
    cfg.input_word_count  = 7;
    // FC16 goes to the output area even where the address equals an input register.
    cfg.output_base_reg   = static_cast<std::uint16_t>(output_base);
    cfg.output_word_count = 6;
    cfg.input_uses_fc4    = true;

    // Gateway status word + PROFIBUS live list (used to detect a lost slave).
    cfg.status_base_reg   = 0x0300;
    cfg.status_word_count = 9;

    cfg.poll_interval_ms = 50;
    cfg.timeout_ms       = 1000;
    // Write the whole output area every cycle. The MGate's fault timeout only
    // counts Modbus accesses to the output block; writing only on change would
    // let it expire while the PC is healthy but idle.
    cfg.write_only_dirty = false;

    MGateDriver drv(cfg);

    IndraDriveConfig dcfg;
    dcfg.profibus_address = static_cast<int>(profibus_address);
    dcfg.max_velocity_rpm = 50.0;  // commissioning cap; move_to() rejects anything faster
    IndraDrive drive(drv, dcfg);

    g_drive.store(&drive);
    std::signal(SIGINT, on_sigint);

    drv.start();
    if (!drv.wait_for_data(5000)) {
        std::printf("no data from gateway: %s\n", drv.stats().last_error.c_str());
        drv.stop();
        return 1;
    }

    int rc = 0;
    if (do_move) {
        rc = test_move(drive);
        if (rc != 0) std::printf("test move failed: %s\n", drive.last_error().c_str());
        std::printf("disabling...\n");
        drive.disable();
        std::printf("after:   %s\n", drive.describe().c_str());
    } else {
        rc = monitor(drv, drive);
    }

    // Control word 0 (drive off, parameter mode) and give it a few cycles to go out.
    drive.shutdown();
    sleep_ms(300);
    g_drive.store(nullptr);

    const DriverStats s = drv.stats();
    std::printf("cycles=%llu read_err=%llu write_err=%llu reconnects=%llu last_cycle=%lld us\n",
                static_cast<unsigned long long>(s.cycles),
                static_cast<unsigned long long>(s.read_errors),
                static_cast<unsigned long long>(s.write_errors),
                static_cast<unsigned long long>(s.reconnects), s.last_cycle_us);

    drv.stop();
    return rc;
}
