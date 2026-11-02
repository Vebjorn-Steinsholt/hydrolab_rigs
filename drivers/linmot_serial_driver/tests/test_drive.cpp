// SerialBridge and LinMotDrive against the simulated Arduino and drive (bridge_sim.h).
#include <chrono>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "bridge_sim.h"
#include "linmot/linmot_drive.h"

using namespace linmot;
using linmot_test::BridgeSim;

int failures = 0;
#define CHECK(cond) do { if (!(cond)) { std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); ++failures; } } while (0)

namespace {

LinMotDriveConfig fast_config() {
    LinMotDriveConfig c;
    c.ready_timeout_ms = 1000;
    c.step_timeout_ms = 300;
    c.home_timeout_ms = 1000;
    c.feedback_timeout_ms = 100;
    return c;
}

void sleep_ms(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

bool contains(const std::string& s, const std::string& part) {
    return s.find(part) != std::string::npos;
}

// Runs bring_up() on a fresh bridge and drive; the simulator's drive starts as given.
// The fault list is declared first: the bridge thread writes it until the drive is destroyed.
struct Rig {
    std::mutex faults_mtx;
    std::vector<std::string> faults;
    std::vector<std::string> log;
    BridgeSim sim;
    SerialBridge bridge;
    LinMotDrive drive;

    explicit Rig(BridgeSim::Options o = {}, LinMotDriveConfig c = fast_config())
        : sim(o), bridge(BridgeConfig{sim.device()}), drive(bridge, c) {
        drive.set_fault_callback([this](const std::string& why) {
            std::lock_guard<std::mutex> lk(faults_mtx);
            faults.push_back(why);
        });
    }
    std::size_t fault_count() {
        std::lock_guard<std::mutex> lk(faults_mtx);
        return faults.size();
    }
    bool start() { return bridge.start(); }
    bool bring_up() {
        return drive.bring_up([this](const std::string& s) { log.push_back(s); });
    }
};

void test_fresh_drive_switches_on_and_homes() {
    Rig rig;
    CHECK(rig.start());
    CHECK(rig.bring_up());
    CHECK(rig.drive.enabled());
    CHECK(rig.drive.status_word() & linmot_sw::kHomed);
    CHECK(rig.sim.count(Command::Operational) == 1);
    CHECK(rig.sim.count(Command::Home) == 1);
    CHECK(rig.log.size() == 2 && contains(rig.log[0], "homing"));
    CHECK(std::abs(rig.drive.position_m() - 0.1) < 1e-9);
    // The Arduino's start-up text was skipped, not taken for frames.
    CHECK(rig.bridge.stats().dropped_bytes >= 40);
    CHECK(rig.bridge.stats().bad_crc == 0);
}

void test_homed_drive_does_not_move() {
    Rig rig;
    rig.sim.set_state(true, true);
    CHECK(rig.start());
    CHECK(rig.bring_up());
    CHECK(rig.sim.count(Command::Home) == 0);
    CHECK(rig.sim.count(Command::SwitchOn) == 0);
    CHECK(rig.log.empty());
}

void test_error_is_acknowledged() {
    Rig rig;
    rig.sim.set_state(false, true, true);
    CHECK(rig.start());
    CHECK(rig.bring_up());
    CHECK(rig.sim.count(Command::AckError) == 1);
    CHECK(rig.drive.enabled());
}

void test_fatal_error_is_not_acknowledged() {
    Rig rig;
    rig.sim.set_state(false, true, true, true);
    CHECK(rig.start());
    CHECK(!rig.bring_up());
    CHECK(rig.sim.count(Command::AckError) == 0);
    CHECK(contains(rig.drive.last_error(), "fatal"));
}

void test_no_homing_when_not_allowed() {
    LinMotDriveConfig c = fast_config();
    c.home_if_needed = false;
    Rig rig({}, c);
    CHECK(rig.start());
    CHECK(!rig.bring_up());
    CHECK(rig.sim.count(Command::Home) == 0);
    CHECK(contains(rig.drive.last_error(), "not homed"));
}

void test_switch_on_ignored() {
    BridgeSim::Options o;
    o.ignore_switch_on = true;
    Rig rig(o);
    CHECK(rig.start());
    CHECK(!rig.bring_up());
    CHECK(contains(rig.drive.last_error(), "LinMot-Talk"));
}

void test_drive_silent_on_can() {
    BridgeSim::Options o;
    o.drive_on_can = false;
    Rig rig(o);
    CHECK(rig.start());
    CHECK(!rig.bring_up());
    CHECK(contains(rig.drive.last_error(), "CAN"));
}

void test_arduino_silent() {
    BridgeSim::Options o;
    o.send_ready = false;
    LinMotDriveConfig c = fast_config();
    c.ready_timeout_ms = 300;
    Rig rig(o, c);
    CHECK(rig.start());
    CHECK(!rig.bring_up());
    CHECK(contains(rig.drive.last_error(), "READY"));
}

void test_move_streams_and_halt_stops() {
    Rig rig;
    rig.sim.set_state(true, true);
    CHECK(rig.start());
    CHECK(rig.bring_up());

    CHECK(!rig.drive.move_to(0.001, 0.1, 1.0));   // below 5 mm
    CHECK(!rig.drive.move_to(0.1, 2.5, 1.0));     // above 2 m/s
    CHECK(!rig.drive.move_to(0.1, 0.1, 25.0));    // above 20 m/s^2
    CHECK(rig.drive.move_to(0.105, 0.0314, 0.0987));
    sleep_ms(100);
    CHECK(rig.sim.position() == 1050000);
    std::vector<CommandFrame> sent = rig.sim.commands();
    CHECK(!sent.empty() && sent.back().command == Command::GoToPosition);
    if (!sent.empty()) {
        CHECK(sent.back().velocity == 31400);
        CHECK(sent.back().acceleration == 9870 && sent.back().deceleration == 9870);
    }
    const int gotos = rig.sim.count(Command::GoToPosition);
    CHECK(gotos >= 3);  // one per 20 ms period

    rig.drive.halt();
    sleep_ms(100);
    CHECK(rig.sim.count(Command::GoToPosition) <= gotos + 1);
    sent = rig.sim.commands();
    CHECK(!sent.empty() && sent.back().command == Command::RequestStatus);
    CHECK(rig.drive.enabled() && !rig.drive.faulted());

    CHECK(rig.drive.disable());
    CHECK(!(rig.drive.status_word() & linmot_sw::kOperationEnabled));
    // Switched off, then pre-operational: the drive's sync watchdog is off while idle.
    // disable() returns once the frame is written; the simulator handles it a moment later.
    sleep_ms(50);
    CHECK(rig.sim.count(Command::PreOperational) == 1);
    CHECK(!rig.sim.operational());
    // The next bring_up() starts it again.
    CHECK(rig.bring_up());
    CHECK(rig.sim.operational());
    CHECK(rig.fault_count() == 0);
}

void test_faults_latch() {
    {
        Rig rig;
        rig.sim.set_state(true, true);
        CHECK(rig.start() && rig.bring_up());
        rig.sim.raise_error();
        sleep_ms(100);
        CHECK(rig.drive.faulted() && contains(rig.drive.fault_reason(), "drive error"));
        CHECK(rig.fault_count() == 1);
        CHECK(!rig.drive.move_to(0.1, 0.1, 1.0));
        // bring_up() clears the fault and acknowledges the error.
        CHECK(rig.bring_up());
        CHECK(!rig.drive.faulted());
    }
    {
        Rig rig;
        rig.sim.set_state(true, true);
        CHECK(rig.start() && rig.bring_up());
        rig.sim.switch_off_externally();
        sleep_ms(100);
        CHECK(rig.drive.faulted() && contains(rig.drive.fault_reason(), "switched off"));
    }
    {
        Rig rig;
        rig.sim.set_state(true, true);
        CHECK(rig.start() && rig.bring_up());
        CHECK(rig.drive.move_to(0.1, 0.1, 1.0));
        rig.sim.set_flags_once(flag::kWatchdogTripped);
        sleep_ms(100);
        CHECK(rig.drive.faulted() && contains(rig.drive.fault_reason(), "watchdog"));
        CHECK(!rig.bridge.streaming());
    }
    {
        Rig rig;
        rig.sim.set_state(true, true);
        CHECK(rig.start() && rig.bring_up());
        rig.sim.set_flags_once(flag::kRejected);
        sleep_ms(100);
        CHECK(rig.drive.faulted() && contains(rig.drive.fault_reason(), "rejected"));
    }
    {
        Rig rig;
        rig.sim.set_state(true, true);
        CHECK(rig.start() && rig.bring_up());
        rig.sim.restart();
        sleep_ms(100);
        CHECK(rig.drive.faulted() && contains(rig.drive.fault_reason(), "restarted"));
    }
    {
        Rig rig;
        rig.sim.set_state(true, true);
        CHECK(rig.start() && rig.bring_up());
        rig.sim.set_silent(true);
        sleep_ms(250);
        CHECK(rig.drive.faulted() && contains(rig.drive.fault_reason(), "no feedback"));
    }
}

void test_missing_port() {
    SerialBridge bridge(BridgeConfig{"/dev/does-not-exist"});
    CHECK(!bridge.start());
    CHECK(contains(bridge.stats().last_error, "cannot open"));
}

}  // namespace

int main() {
    test_fresh_drive_switches_on_and_homes();
    test_homed_drive_does_not_move();
    test_error_is_acknowledged();
    test_fatal_error_is_not_acknowledged();
    test_no_homing_when_not_allowed();
    test_switch_on_ignored();
    test_drive_silent_on_can();
    test_arduino_silent();
    test_move_streams_and_halt_stops();
    test_faults_latch();
    test_missing_port();
    if (failures == 0) std::printf("test_drive: all checks passed\n");
    return failures == 0 ? 0 : 1;
}
