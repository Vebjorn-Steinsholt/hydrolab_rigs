// linmot_probe - checks the link to the LinMot bridge without moving anything.
//
// Opens the port, waits for READY, sends NMT operational and then only status requests for a
// few seconds; prints the drive's position and status word and how many feedbacks arrived.
// Never switches the drive on, homes or moves it. A switched-off drive is left in NMT
// pre-operational, so its sync watchdog does not raise an error after the probe.
//
//   ros2 run linmot_serial_driver linmot_probe [/dev/ttyACM0] [seconds]

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

#include "linmot/linmot_drive.h"
#include "linmot/serial_bridge.h"

using namespace linmot;
using Clock = std::chrono::steady_clock;

int main(int argc, char** argv) {
    BridgeConfig cfg;
    if (argc > 1) cfg.device = argv[1];
    const double seconds = argc > 2 ? std::atof(argv[2]) : 2.0;

    SerialBridge bridge(cfg);
    if (!bridge.start()) {
        std::printf("cannot open %s: %s\n", cfg.device.c_str(), bridge.stats().last_error.c_str());
        return 2;
    }
    const auto opened = Clock::now();
    if (!bridge.wait_ready(3000)) {
        std::printf("no READY within 3 s: is the bridge firmware on the Arduino?\n");
        return 1;
    }
    const auto ready_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - opened).count();
    std::printf("READY %lld ms after opening the port\n", static_cast<long long>(ready_ms));

    const std::uint64_t seq = bridge.send(Command::Operational);
    if (!bridge.wait_feedback(seq, [](const Feedback&) { return true; }, 2000)) {
        std::printf("no feedback from the drive: check its power, CAN cable, node ID and TPDO1\n");
        return 1;
    }
    const BridgeStats before = bridge.stats();
    std::uint64_t feedbacks = 0;
    const std::uint64_t from = bridge.latest().sequence;
    const auto until = Clock::now() + std::chrono::milliseconds(static_cast<int>(seconds * 1000));
    std::uint64_t last = from;
    while (Clock::now() < until) {
        if (bridge.wait_feedback(last + 1, [](const Feedback&) { return true; }, 200)) {
            last = bridge.latest().sequence;
            ++feedbacks;
        }
    }
    const BridgeStats after = bridge.stats();
    const Feedback fb = bridge.latest();
    std::printf("position %.3f mm, status %s\n", fb.position / kPositionPerMetre * 1000.0,
                describe_status(fb.status_word).c_str());
    std::printf("%llu feedbacks for %llu frames in %.1f s; bad CRC %llu, bytes skipped %llu "
                "(start-up text included), Arduino bad frames %u\n",
                static_cast<unsigned long long>(feedbacks),
                static_cast<unsigned long long>(after.frames_sent - before.frames_sent), seconds,
                static_cast<unsigned long long>(after.bad_crc),
                static_cast<unsigned long long>(after.dropped_bytes), fb.bad_frames);
    if (!(fb.status_word & linmot_sw::kOperationEnabled)) {
        bridge.wait_sent(bridge.send(Command::PreOperational), 1000);
    }
    bridge.stop();
    return 0;
}
