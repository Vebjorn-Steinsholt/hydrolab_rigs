#!/usr/bin/env python3
"""
Link test for the LinMot serial-to-CAN bridge (linmot_serial_bridge.ino). No ROS.

Without --move nothing moves: the test checks the serial link and the CAN path (ping, a frame with
a bad CRC, the drive's status after NMT operational). With --move it also switches the drive on,
moves the slide slowly to the centre, streams a small slow sine, checks that an out-of-range
command is rejected, and checks that the watchdog aborts a move when the computer stops sending.
At the end the drive is switched off and put into NMT pre-operational, unless --stay-on.

Apart from the watchdog check, the test never pauses longer than 20 ms between frames, so it also
passes with the drive's own sync watchdog (Legacy Sync Watchdog in LinMot-Talk) switched on. With
it on, the watchdog check also shows the drive stopping itself, and --stay-on leaves the drive to
do the same once the test has ended.

    python3 link_test.py                          # link only, no motion
    python3 link_test.py --move                   # also motion, drive must be homed
    python3 link_test.py --move --home            # home first (the slide moves to find home)
    python3 link_test.py --move --rate 100        # stream at 100 Hz instead of 50

For --move keep LinMot-Talk open to switch the drive off (the rig has no e-stop). Ctrl-C stops
sending; the watchdog then aborts the motion.
"""

import argparse
import binascii
import math
import struct
import sys
import time

import serial

SYNC = b'\xa5\x5a'
COMMAND = struct.Struct('<2sBBiIII')   # sync, command, sequence, position, velocity, acc, dec
REPLY = struct.Struct('<2sBBiHBBH')    # sync, type, sequence, position, status, flags, bad, crc

CMD_PING, CMD_OPERATIONAL, CMD_HOME, CMD_GO_TO_POS = 0, 1, 2, 3
CMD_SWITCH_ON, CMD_SWITCH_OFF, CMD_REQUEST_STATUS, CMD_ACK_ERROR = 4, 5, 6, 7
CMD_PRE_OPERATIONAL = 8         # NMT pre-operational: no PDOs, sync watchdog off; no reply
KEEPALIVE_S = 0.02              # longest pause between frames outside the watchdog check
REPLY_READY, REPLY_FEEDBACK = 0x10, 0x01
FLAG_WATCHDOG, FLAG_CAN_ERROR, FLAG_REJECTED = 0x01, 0x02, 0x04

# Status word bits (LinMot Motion Control SW manual, section 3.23).
STATUS_BITS = {0: 'operation enabled', 3: 'error', 6: 'switch on locked', 7: 'warning',
               9: 'special motion', 10: 'in target', 11: 'homed', 12: 'fatal error',
               13: 'motion active'}

POS_PER_MM = 10000              # position unit 0.1 um
VEL_PER_MPS = 1_000_000         # velocity unit 1e-6 m/s
ACC_PER_MPS2 = 100_000          # acceleration unit 1e-5 m/s^2

CENTRE_MM = 100.0
ARRIVED_MM = 0.5                # "at the target"; the rig's friction leaves a few 0.1 mm
APPROACH_MPS = 0.02             # speed for moves to the centre
APPROACH_MPS2 = 0.2
SINE_AMPLITUDE_MM = 5.0
SINE_PERIOD_S = 2.0
SINE_DURATION_S = 6.0
WATCHDOG_MS = 200               # as in the firmware


def crc(data):
    return binascii.crc_hqx(data, 0xFFFF)


def describe_status(word):
    names = [name for bit, name in STATUS_BITS.items() if word & (1 << bit)]
    return f'0x{word:04X} ({", ".join(names) or "-"})'


class Link:
    """Frames commands and parses replies, resynchronising on the sync bytes."""

    def __init__(self, port):
        self.serial = serial.Serial(port, 115200, timeout=0)
        # Opening the port resets the Uno; anything still buffered is from before that reset.
        self.serial.reset_input_buffer()
        self.buffer = b''
        self.sequence = 0
        self.sent = {}          # sequence -> send time, for the round trip
        self.round_trips = []
        self.dropped_bytes = 0
        self.bad_replies = 0

    def send(self, command, position=0, velocity=0, acceleration=0, deceleration=0):
        self.sequence = (self.sequence + 1) % 256
        body = COMMAND.pack(SYNC, command, self.sequence, int(position), int(velocity),
                            int(acceleration), int(deceleration))
        self.serial.write(body + struct.pack('<H', crc(body[2:])))
        self.sent[self.sequence] = time.monotonic()
        return self.sequence

    def send_raw(self, data):
        self.serial.write(data)

    def replies(self):
        """Return the complete, valid replies received so far."""
        self.buffer += self.serial.read(4096)
        out = []
        while True:
            start = self.buffer.find(SYNC)
            if start < 0:
                self.dropped_bytes += max(0, len(self.buffer) - 1)
                self.buffer = self.buffer[-1:]
                return out
            self.dropped_bytes += start
            self.buffer = self.buffer[start:]
            if len(self.buffer) < REPLY.size:
                return out
            frame, self.buffer = self.buffer[:REPLY.size], self.buffer[REPLY.size:]
            _, kind, seq, position, status, flags, bad, received = REPLY.unpack(frame)
            if crc(frame[2:-2]) != received:
                self.bad_replies += 1
                self.buffer = frame[1:] + self.buffer   # look for the next sync inside it
                continue
            if seq in self.sent:
                self.round_trips.append(time.monotonic() - self.sent.pop(seq))
            out.append({'type': kind, 'seq': seq, 'position_mm': position / POS_PER_MM,
                        'status': status, 'flags': flags, 'bad_frames': bad})

    def idle(self, seconds):
        """Wait without a gap in the frames: status requests every KEEPALIVE_S."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.send(CMD_REQUEST_STATUS)
            self.replies()
            time.sleep(KEEPALIVE_S)

    def wait_for(self, predicate, timeout_s, resend=None, period_s=KEEPALIVE_S):
        """Read replies until one satisfies predicate; call resend every period_s if given."""
        deadline = time.monotonic() + timeout_s
        next_send = 0.0
        while time.monotonic() < deadline:
            if resend and time.monotonic() >= next_send:
                resend()
                next_send = time.monotonic() + period_s
            for reply in self.replies():
                if predicate(reply):
                    return reply
            time.sleep(0.002)
        return None


class Test:
    def __init__(self):
        self.failures = []

    def check(self, ok, what, detail=''):
        print(f'  [{"PASS" if ok else "FAIL"}] {what}{": " + detail if detail else ""}')
        if not ok:
            self.failures.append(what)
        return ok


def status_reply(link, timeout_s=1.0):
    """Feedback to one of our status requests; replies to earlier commands are skipped."""
    requests = set()
    return link.wait_for(lambda r: r['type'] == REPLY_FEEDBACK and r['seq'] in requests,
                         timeout_s, resend=lambda: requests.add(link.send(CMD_REQUEST_STATUS)))


def go_to(link, mm, velocity_mps, acceleration_mps2):
    return link.send(CMD_GO_TO_POS, mm * POS_PER_MM, velocity_mps * VEL_PER_MPS,
                     acceleration_mps2 * ACC_PER_MPS2, acceleration_mps2 * ACC_PER_MPS2)


def stream_to(link, mm, rate_hz, timeout_s):
    """Streams one target at rate_hz until the slide is within ARRIVED_MM of it."""
    period = 1.0 / rate_hz
    deadline = time.monotonic() + timeout_s
    last = None
    while time.monotonic() < deadline:
        go_to(link, mm, APPROACH_MPS, APPROACH_MPS2)
        for reply in link.replies():
            if reply['type'] == REPLY_FEEDBACK:
                last = reply
        if last and abs(last['position_mm'] - mm) < ARRIVED_MM:
            return last
        time.sleep(period)
    return last


def wait_until_still(link, timeout_s=3.0, window_s=0.2, tolerance_mm=0.02):
    """Status replies until the position has changed less than tolerance_mm over window_s."""
    deadline = time.monotonic() + timeout_s
    window = []
    while time.monotonic() < deadline:
        reply = status_reply(link, 0.5)
        if reply is None:
            continue
        now = time.monotonic()
        window = [(t, mm) for t, mm in window if now - t <= window_s]
        window.append((now, reply['position_mm']))
        positions = [mm for _, mm in window]
        spread = max(positions) - min(positions)
        if now - window[0][0] >= 0.75 * window_s and spread < tolerance_mm:
            return reply
        time.sleep(0.02)
    return None


def link_checks(link, test):
    print('Link')
    # Only a READY that answers one of our pings: the Uno's own READY at start-up carries
    # sequence 0 and may come from a start before the reset at port open (right after an upload).
    pings = set()
    ready = link.wait_for(lambda r: r['type'] == REPLY_READY and r['seq'] in pings, 5.0,
                          resend=lambda: pings.add(link.send(CMD_PING)), period_s=0.2)
    if not test.check(ready is not None, 'ping answered with READY'):
        return False

    bad_before = ready['bad_frames']
    body = COMMAND.pack(SYNC, CMD_PING, 0, 0, 0, 0, 0)
    link.send_raw(body + struct.pack('<H', crc(body[2:]) ^ 0xFFFF))   # wrong CRC
    reply = link.wait_for(lambda r: r['type'] == REPLY_READY, 1.0,
                          resend=lambda: link.send(CMD_PING), period_s=0.2)
    test.check(reply is not None and (reply['bad_frames'] - bad_before) % 256 == 1,
               'frame with a bad CRC dropped and counted',
               f"bad frames {bad_before} -> {reply['bad_frames'] if reply else '?'}")

    print('CAN path')
    link.send(CMD_OPERATIONAL)
    status = status_reply(link, 2.0)
    if not test.check(status is not None, 'drive answers a status request (CAN, TPDO1 on SYNC)'):
        print('  No feedback: check CAN wiring, 500 kbit/s, node 12, INT on pin D2 and that TPDO1 '
              'is synchronous.')
        return False
    print(f"  position {status['position_mm']:.3f} mm, status {describe_status(status['status'])}")
    test.check(not status['flags'] & FLAG_CAN_ERROR, 'no CAN send error')
    return True


def motion_checks(link, test, args):
    period = 1.0 / args.rate

    status = status_reply(link)
    if status and status['status'] & (1 << 3):
        print(f"Drive error, status {describe_status(status['status'])}: acknowledging")
        link.send(CMD_ACK_ERROR)
        status = status_reply(link)
        if not test.check(status is not None and not status['status'] & (1 << 3),
                          'drive error acknowledged'):
            print('  A fatal error cannot be acknowledged; check the drive in LinMot-Talk.')
            return

    print('Switch on')
    # The firmware clears control word bit 0 first if needed (manual section 3.1).
    link.send(CMD_SWITCH_ON)
    link.idle(0.5)   # the drive's hardware test before power on takes about 300 ms
    status = status_reply(link)
    test.check(status is not None and status['status'] & 1, 'operation enabled',
               describe_status(status['status']) if status else 'no reply')

    if args.home:
        print('Home (the slide moves)')
        link.send(CMD_HOME)
        homed = link.wait_for(lambda r: r['type'] == REPLY_FEEDBACK and r['status'] & (1 << 11)
                              and not r['status'] & (1 << 13), 60.0,
                              resend=lambda: link.send(CMD_REQUEST_STATUS))
        if not test.check(homed is not None, 'homed'):
            return
        link.send(CMD_SWITCH_ON)   # clears the home bit in the control word
    status = status_reply(link)
    if not test.check(status is not None and status['status'] & (1 << 11), 'drive is homed',
                      describe_status(status['status']) if status else 'no reply'):
        print('  Positions mean nothing before homing; run again with --home.')
        return

    print(f'Move to the centre ({CENTRE_MM:.0f} mm) at {APPROACH_MPS * 1000:.0f} mm/s')
    distance = abs(status['position_mm'] - CENTRE_MM) / 1000
    reached = stream_to(link, CENTRE_MM, args.rate, distance / APPROACH_MPS + 5.0)
    if not test.check(reached is not None and abs(reached['position_mm'] - CENTRE_MM) < ARRIVED_MM,
                      'at the centre',
                      f"{reached['position_mm']:.3f} mm" if reached else 'no feedback'):
        return

    print(f'Sine: {SINE_AMPLITUDE_MM:.0f} mm, {SINE_PERIOD_S:.0f} s, {SINE_DURATION_S:.0f} s '
          f'at {args.rate:.0f} Hz')
    omega = 2 * math.pi / SINE_PERIOD_S
    peak_v = SINE_AMPLITUDE_MM / 1000 * omega
    peak_a = SINE_AMPLITUDE_MM / 1000 * omega ** 2
    link.round_trips.clear()
    sent = received = 0
    errors = []
    positions = []
    last = reached
    bad_before = reached['bad_frames']
    start = time.monotonic()
    next_send = start
    while True:
        now = time.monotonic()
        if now - start > SINE_DURATION_S:
            break
        if now >= next_send:
            target = CENTRE_MM + SINE_AMPLITUDE_MM * math.sin(omega * (now - start))
            go_to(link, target, 1.5 * peak_v, 1.5 * peak_a)
            sent += 1
            next_send += period
        for reply in link.replies():
            if reply['type'] == REPLY_FEEDBACK:
                received += 1
                t = time.monotonic() - start
                errors.append(reply['position_mm'] - (
                    CENTRE_MM + SINE_AMPLITUDE_MM * math.sin(omega * t)))
                positions.append(reply['position_mm'])
                last = reply
        time.sleep(0.0005)
    rtt = sorted(link.round_trips)
    stroke = max(positions) - min(positions) if positions else 0.0
    test.check(stroke >= 0.8 * 2 * SINE_AMPLITUDE_MM, 'slide followed the sine',
               f'{stroke:.2f} mm peak to peak, commanded {2 * SINE_AMPLITUDE_MM:.0f} mm')
    test.check(received >= 0.95 * sent, 'one feedback per command', f'{received} of {sent}')
    if rtt:
        print(f'  round trip median {1000 * rtt[len(rtt) // 2]:.1f} ms, '
              f'max {1000 * rtt[-1]:.1f} ms')
        test.check(rtt[-1] < period, 'round trip shorter than one cycle')
    if errors:
        print(f'  feedback minus command: max {max(abs(e) for e in errors):.2f} mm '
              "(includes the drive's lag; for information)")
    test.check((last['bad_frames'] - bad_before) % 256 == 0, 'no bad frames while streaming')
    test.check(not last['flags'] & FLAG_CAN_ERROR, 'no CAN send error while streaming')

    reached = stream_to(link, CENTRE_MM, args.rate, 5.0)
    if not test.check(reached is not None, 'back at the centre after the sine'):
        return

    print('Out-of-range command (to the current position, too fast; nothing should move)')
    still = wait_until_still(link)
    if not test.check(still is not None, 'slide at rest before the check'):
        return
    here = still['position_mm']
    seq = link.send(CMD_GO_TO_POS, here * POS_PER_MM, 3_100_000, APPROACH_MPS2 * ACC_PER_MPS2,
                    APPROACH_MPS2 * ACC_PER_MPS2)
    reply = link.wait_for(lambda r: r['type'] == REPLY_FEEDBACK and r['seq'] == seq, 1.0,
                          resend=lambda: link.send(CMD_REQUEST_STATUS))
    test.check(reply is not None and reply['flags'] & FLAG_REJECTED, 'rejected')
    after = status_reply(link)
    if reply and after:
        test.check(abs(after['position_mm'] - here) < 0.05, 'did not move',
                   f"{after['position_mm'] - here:+.3f} mm")

    print('Watchdog: start a 5 mm move at 5 mm/s, stop sending after 0.3 s')
    target = CENTRE_MM + 5.0
    stop_at = time.monotonic() + 0.3
    while time.monotonic() < stop_at:
        link.send(CMD_GO_TO_POS, target * POS_PER_MM, 0.005 * VEL_PER_MPS,
                  APPROACH_MPS2 * ACC_PER_MPS2, APPROACH_MPS2 * ACC_PER_MPS2)
        link.replies()
        time.sleep(period)
    stopped_sending = time.monotonic()
    # A second without sending, listening only. The bridge's watchdog aborts the move after
    # 200 ms. With the drive's sync watchdog on, the drive stops itself after 1.5 x its cycle
    # time (error CDh "Error Control Timeout") and, as CANopen does on a communication error,
    # leaves NMT operational: it sends nothing more until the next NMT start.
    tripped_at = None
    while time.monotonic() < stopped_sending + 1.0:
        for reply in link.replies():
            if tripped_at is None and reply['flags'] & FLAG_WATCHDOG:
                tripped_at = time.monotonic()
        time.sleep(0.002)
    link.send(CMD_OPERATIONAL)   # back to NMT operational in case the drive left it
    first = status_reply(link)
    if not test.check(first is not None, 'drive answers after the pause',
                      '' if first else 'no reply'):
        return
    test.check(first['flags'] & FLAG_WATCHDOG, 'watchdog tripped')
    if tripped_at is not None:
        print(f'  trip reported about {(tripped_at - stopped_sending) * 1000:.0f} ms after the '
              f'last command (watchdog {WATCHDOG_MS} ms)')
    sync_watchdog = bool(first['status'] & (1 << 3))
    if sync_watchdog:
        print(f"  the drive stopped itself too (sync watchdog on): "
              f"{describe_status(first['status'])}")
    else:
        print('  the drive kept its power stage on (sync watchdog off)')
    link.idle(0.5)
    second = status_reply(link)
    if second:
        moved = abs(second['position_mm'] - first['position_mm'])
        test.check(moved < 0.05, 'slide stopped after the trip', f'{moved:.3f} mm in 0.5 s')
        test.check(second['position_mm'] < target - 1.0, 'move aborted short of its target',
                   f"{second['position_mm']:.2f} mm, target {target:.1f} mm")

    if sync_watchdog:
        link.send(CMD_ACK_ERROR)
        link.send(CMD_SWITCH_ON)
        link.idle(0.5)
        status = status_reply(link)
        if not test.check(status is not None and status['status'] & 1,
                          'operation enabled again after the sync watchdog',
                          describe_status(status['status']) if status else 'no reply'):
            return
        # Some errors lose the homing; then 100 mm is not the centre any more.
        if not test.check(status['status'] & (1 << 11), 'still homed after the sync watchdog',
                          describe_status(status['status'])):
            print('  Not moving back; home again with --home.')
            return

    print('Back to the centre')
    back = stream_to(link, CENTRE_MM, args.rate, 5.0)
    test.check(back is not None and abs(back['position_mm'] - CENTRE_MM) < ARRIVED_MM,
               'back at the centre after the watchdog',
               f"{back['position_mm']:.2f} mm, status {describe_status(back['status'])}"
               if back else 'no feedback')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', default='/dev/ttyACM0')
    parser.add_argument('--move', action='store_true', help='also run the motion tests')
    parser.add_argument('--home', action='store_true', help='home the drive first (with --move)')
    parser.add_argument('--rate', type=float, default=50.0, help='command rate [Hz]')
    parser.add_argument('--stay-on', action='store_true', help='leave the drive switched on')
    args = parser.parse_args()

    try:
        link = Link(args.port)
    except serial.SerialException as error:
        print(f'Cannot open {args.port}: {error}. Is the Arduino connected? Try ls /dev/ttyACM*.')
        return 2
    test = Test()
    switched_on = False
    try:
        if link_checks(link, test) and args.move:
            switched_on = True
            motion_checks(link, test, args)
    except KeyboardInterrupt:
        print('\nInterrupted: stopped sending; the watchdog aborts any motion.')
        time.sleep(2 * WATCHDOG_MS / 1000)
    finally:
        if switched_on:
            # After a stop by the drive's sync watchdog it is pre-operational and ignores PDOs.
            link.send(CMD_OPERATIONAL)
        if switched_on and not args.stay_on:
            link.send(CMD_SWITCH_OFF)
            print('Drive switched off.')
        if switched_on:
            # A drive error during the motion tests would otherwise only show up in the next run.
            final = status_reply(link)
            if final:
                print(f"Final status: {final['position_mm']:.3f} mm, "
                      f"{describe_status(final['status'])}")
            test.check(final is not None and not final['status'] & ((1 << 3) | (1 << 12)),
                       'drive ends without an error',
                       describe_status(final['status']) if final else 'no reply')
        if not args.stay_on:
            # Sync watchdog off until the next NMT start, so the idle drive does not go into error.
            link.send(CMD_PRE_OPERATIONAL)
            link.serial.flush()
        if link.bad_replies or link.dropped_bytes:
            print(f'Replies with a bad CRC: {link.bad_replies}, bytes skipped while '
                  f'resynchronising: {link.dropped_bytes}')
        print('All checks passed.' if not test.failures else
              f'{len(test.failures)} check(s) failed: ' + '; '.join(test.failures))
    return 1 if test.failures else 0


if __name__ == '__main__':
    sys.exit(main())
