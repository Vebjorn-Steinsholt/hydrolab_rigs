#!/usr/bin/env python3

from collections import deque
import math
import threading
import time

import numpy as np
from probe_serial import ClockFit, parse_line, range_m, Sequence
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Range
import serial
from wave_probe_interfaces.msg import SurfaceElevation
from wave_probe_interfaces.srv import ZeroMeasurement

READ_TIMEOUT_S = 0.5
CLOCK_LOG_S = 300.0
DROPOUT_LINES = 1200
OPEN_RESET_S = 3.0          # a restart this soon after opening is the Uno's reset
AIR_TEMP_RANGE = (-10.0, 50.0)  # Celsius
ZERO_DURATION = (1.0, 120.0)


class WaveProbeNode(Node):
    def __init__(self):
        super().__init__('wave_probe_node')

        def p(name, default):
            return self.declare_parameter(name, default).get_parameter_value()

        self.port_name = p('port', '/dev/wave_probe').string_value
        self.baud = p('baud', 115200).integer_value
        self.frame_id = p('frame_id', 'wp_us1').string_value
        self.min_range = p('min_range_m', 0.02).double_value
        self.max_range = p('max_range_m', 4.0).double_value
        self.fov = p('field_of_view_rad', 0.26).double_value
        self.reopen_s = p('reopen_s', 2.0).double_value
        self.dropout_warn = p('dropout_warn', 0.10).double_value
        # dynamic_typing: `-p air_temp_c:=21` arrives as an int, so not double_value
        air = self.declare_parameter(
            'air_temp_c', 20.0, ParameterDescriptor(dynamic_typing=True)).value
        if isinstance(air, bool) or not isinstance(air, (int, float)):
            raise ValueError(f'air_temp_c must be a number, got {air!r}')
        self.air_temp_c = float(air)
        self.clock = ClockFit(p('fit_window_s', 300.0).double_value,
                              p('fit_block_s', 10.0).double_value,
                              p('latency_floor_s', 0.002).double_value)
        self.zero_max_std = p('zero_max_std', 0.002).double_value
        self.sequence = Sequence()
        self.add_on_set_parameters_callback(self.on_set)

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=100,
                         durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(Range, 'probe_range', qos)
        self.elev_pub = self.create_publisher(SurfaceElevation, 'surface_elevation', qos)
        self.zero = 0.0
        self.zero_lock = threading.Lock()  # protects self.zero and self.collect
        self.collect = None         # list of ranges (None = rejected) while a zero runs

        self.create_service(ZeroMeasurement, 'zero_measurement', self.on_zero_measurement,
                            callback_group=ReentrantCallbackGroup())

        self.started = time.monotonic()
        self.opened = time.monotonic()  # when the port was last opened
        self.lost_total = 0       # lines lost to seq gaps, since start
        self.bad_total = 0        # lines that didn't parse or didn't fit
        self.inf = deque(maxlen=DROPOUT_LINES)
        self.n_inf = 0            # True values in self.inf
        self.fit_logged = False
        self.last_clock_log = time.monotonic()
        self.create_timer(1.0, self.log_status)

        self.stopping = threading.Event()
        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()

    def read_loop(self):
        announce = True
        while not self.stopping.is_set():
            port = self.open_or_wait(announce)
            if port is None:
                return
            self.opened = time.monotonic()
            self.sequence.reset()
            self.clock.reset()
            self.fit_logged = False
            skip = True
            try:
                port.reset_input_buffer()
                while not self.stopping.is_set():
                    raw = port.readline()
                    arrival = self.get_clock().now().nanoseconds * 1e-9
                    if not raw.endswith(b'\n'):
                        skip = skip or bool(raw)
                        continue
                    if skip:
                        skip = False
                        continue
                    line = parse_line(raw)
                    if line is None:
                        self.bad_total += 1
                        continue
                    self.on_line(line, arrival)
            except (serial.SerialException, OSError) as e:
                self.get_logger().warning(
                    f'{self.port_name}: {e}; retrying every {self.reopen_s:g} s')
                announce = False
            finally:
                port.close()

    def open_or_wait(self, announce: bool) -> serial.Serial | None:
        while not self.stopping.is_set():
            try:
                # exclusive: a miniterm left open can't steal lines from the node
                return serial.Serial(self.port_name, self.baud,
                                     timeout=READ_TIMEOUT_S, exclusive=True)
            except serial.SerialException as e:
                if announce:
                    self.get_logger().warning(
                        f'{self.port_name}: {e}; retrying every {self.reopen_s:g} s')
                    announce = False
            self.stopping.wait(self.reopen_s)
        return None

    def on_line(self, line, arrival):
        step = self.sequence.push(line)
        log = self.get_logger()
        if step.kind == 'bad':
            self.bad_total += 1
            return
        if step.kind == 'first':
            log.info(f'{self.port_name} open; first line at seq {line.seq}')
        elif step.kind == 'reset':
            self.clock.reset()
            self.fit_logged = False
            if time.monotonic() - self.opened < OPEN_RESET_S:
                # opening the port resets an Uno; lines from before it can still arrive first
                log.info(f'board restarted on open, at seq {line.seq}; clock fit restarted')
            else:
                log.warning(f'restart at seq {line.seq} (board reset, or '
                            f'{self.sequence.max_bad} lines that did not fit); '
                            f'clock fit restarted')
        elif step.kind == 'gap':
            self.lost_total += step.lost
            minutes = (time.monotonic() - self.started) / 60
            log.warning(f'seq {line.seq - step.lost - 1} -> {line.seq}: {step.lost} lines '
                        f'lost ({self.lost_total} in {minutes:.0f} min)')
        if line.echo_us > 0:
            jump = self.clock.add(step.t_s + line.echo_us * 1e-6, arrival)
            if jump is not None:
                log.warning(f'clock refit moved the stamps by {jump * 1e3:+.1f} ms')
        if self.clock.d_ref is None:        # no echo yet since the (re)start: no clock
            return
        is_inf = line.echo_us == 0
        if len(self.inf) == self.inf.maxlen:
            self.n_inf -= self.inf[0]
        self.inf.append(is_inf)
        self.n_inf += is_inf
        self.publish(line, step.t_s)

    def publish(self, line, t_trigger):
        msg = Range()
        t_water = t_trigger + line.echo_us * 1e-6 / 2          # decision 3
        msg.header.stamp = Time(nanoseconds=round(self.clock.to_ros(t_water) * 1e9)).to_msg()
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self.fov
        msg.min_range, msg.max_range = self.min_range, self.max_range
        msg.range = range_m(line.echo_us, self.air_temp_c)
        r = msg.range
        valid = line.echo_us > 0 and self.min_range <= r <= self.max_range
        try:
            self.pub.publish(msg)
            with self.zero_lock:
                if self.collect is not None:
                    self.collect.append(r if valid else None)
                if self.zero > 0.0:
                    elev = SurfaceElevation(header=msg.header)
                    elev.surface_elevation = self.zero - r if valid else math.nan
                    self.elev_pub.publish(elev)
        except Exception:
            if self.context.ok():
                raise
            self.stopping.set()  # Ctrl-C shut rclpy down under the thread: stop quietly

    def log_status(self):
        log = self.get_logger()
        n = len(self.inf)
        if n == DROPOUT_LINES and self.n_inf / n > self.dropout_warn:
            log.warning(f'{100 * self.n_inf / n:.0f} % no echo in the last 60 s',
                        throttle_duration_sec=60.0)
        s = self.clock.status()
        eps_ppm = s['eps_ppm']
        if s['ready'] and not self.fit_logged:
            self.fit_logged = True
            log.info(f'clock: first fit, {eps_ppm:+.0f} ppm')
        if time.monotonic() - self.last_clock_log >= CLOCK_LOG_S:
            self.last_clock_log = time.monotonic()
            rms_ms, floor_ms = s['rms_s'] * 1e3, s['floor_s'] * 1e3
            log.info(f'clock: {eps_ppm:+.0f} ppm, rms {rms_ms:.2f} ms over '
                     f'{s["blocks"]} blocks ({s["span_s"]:.0f} s), '
                     f'floor {floor_ms:.1f} ms (set); '
                     f'{self.lost_total} lost, {self.bad_total} bad')

    def on_zero_measurement(self, request, response):
        lo, hi = ZERO_DURATION
        if not lo <= request.duration <= hi:
            return self.zero_failed(response, f'duration must be between {lo:g} and {hi:g} s')
        with self.zero_lock:
            if self.collect is not None:
                return self.zero_failed(response, 'a zero measurement is already running')
            self.collect = []
        time.sleep(request.duration)
        with self.zero_lock:
            samples, self.collect = self.collect, None
        good = [s for s in samples if s is not None]
        response.num_samples = len(good)
        response.num_rejected_samples = len(samples) - len(good)
        if not samples:
            return self.zero_failed(response, 'no samples: is the port open?')
        if response.num_rejected_samples / len(samples) > self.dropout_warn:
            return self.zero_failed(
                response, f'{response.num_rejected_samples} of {len(samples)} samples rejected')
        response.zero_measurement, response.std = float(np.mean(good)), float(np.std(good))
        if response.std > self.zero_max_std:
            return self.zero_failed(
                response, f'std {response.std * 1e3:.1f} mm > '
                          f'{self.zero_max_std * 1e3:.1f} mm; water not still?')
        self.zero = response.zero_measurement
        response.success = True
        response.message = (f'zero: {response.zero_measurement:.4f} m, '
                            f'std {response.std * 1e3:.1f} mm, '
                            f'{response.num_samples} used, '
                            f'{response.num_rejected_samples} rejected '
                            f'({request.duration:g} s)')
        self.get_logger().info(response.message)
        return response

    def zero_failed(self, response, reason):
        response.success = False
        response.message = f'zero refused: {reason}; zero stays {self.zero:.4f} m'
        self.get_logger().warning(response.message)
        return response

    def on_set(self, params):
        for param in params:
            if param.name != 'air_temp_c':
                return SetParametersResult(successful=False,
                                           reason=f'{param.name} is read at start only')
            value = param.value
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not AIR_TEMP_RANGE[0] <= value <= AIR_TEMP_RANGE[1]):
                return SetParametersResult(successful=False,
                                           reason=f'air_temp_c must be {AIR_TEMP_RANGE} °C')
        for param in params:
            old, self.air_temp_c = self.air_temp_c, float(param.value)
            change = range_m(1, self.air_temp_c) / range_m(1, old) - 1
            self.get_logger().info(f'air_temp_c {old:g} -> {self.air_temp_c:g} °C '
                                   f'(ranges {100 * change:+.2f} %)')
        return SetParametersResult(successful=True)

    def stop(self):
        self.stopping.set()
        self.reader.join(timeout=2 * READ_TIMEOUT_S)


def main():
    rclpy.init()
    node = WaveProbeNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
