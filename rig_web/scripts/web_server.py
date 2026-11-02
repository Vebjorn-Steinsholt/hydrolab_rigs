#!/usr/bin/env python3
"""
Web page for running a wavemaker or the sloshing oscillator, in place of LabVIEW.

It talks to bridge_node; the rig parameter selects the texts and the topic names.

Serves the page in html/ and a small JSON API on one port. The page plays LabVIEW's role: the
server publishes height, period, start and stop to the bridge, and the bridge heartbeat only
while an open page keeps acknowledging the server's events. When the page closes, the browser
freezes or the network drops, the heartbeat stops and the bridge stops the wave.

    ros2 run rig_web web_server --ros-args -r __ns:=/wavemakers/ladertanken -p port:=8080

Every open page counts: the wave keeps running while any one of them is alive.

The page also shows the log of every node in the wavemaker's namespace (controller, bridge,
lifecycle manager, this server), read from /rosout.

After idle_timeout_s (default 2 h) without motion, the server shuts the managed nodes down
through the lifecycle manager and exits; the launch files end the whole launch when it does.
Motion is a running wave from the bridge or any setpoint the controller publishes, so goals from
play_dat and returns to upright count too.
For the last idle_warning_s the page shows a warning with a Keep running button, which restarts
the count. Set idle_timeout_s to 0 to turn this off.

The rig parameter ('wavemaker' or 'oscillator') picks the page's texts and the server's messages;
the controls are the same for both.
"""

from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import re
import socket
import threading
import time

from ament_index_python.packages import get_package_share_directory
from nav2_msgs.srv import ManageLifecycleNodes
from rcl_interfaces.msg import Log
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rig_interfaces.srv import ReturnToUpright
from std_msgs.msg import Bool, Float64, String

ACK_TIMEOUT_S = 0.5      # a page is alive while it acknowledged an event this recently
EVENT_PERIOD_S = 0.1     # how often the page gets an update
PLOT_WINDOW_S = 30.0     # setpoint and position history kept for the plot
START_DELAY_S = 0.3      # between sending height/period and start
START_CHECK_S = 2.0      # how long to wait for the bridge's "Starting wave" message
RETURN_TIMEOUT_S = 60.0
SHUTDOWN_TIMEOUT_S = 15.0  # for the lifecycle manager's SHUTDOWN after the idle timeout
EXIT_DELAY_S = 2.0       # lets open pages show the auto-stop notice before the server exits
ACTIVE_STATES = ('starting', 'running', 'stopping')  # bridge states that count as a running wave
# Per rig: 'machine', 'motion' and 'a_motion' are used in the server's messages and on the page,
# the rest only on the page.
RIG_TEXTS = {
    'wavemaker': {
        'machine': 'wavemaker',
        'motion': 'wave',
        'a_motion': 'a wave',
        'title': 'Wavemaker Control',
        'wave_card': 'Regular wave',
        'height_label': 'Height H [m]',
        'motion_card': 'Paddle',
        'plot_note': 'metres at the flap top, last 20 s',
        'return_button': 'Return to upright',
        'returning': 'Returning to upright…',
    },
    'oscillator': {
        'machine': 'oscillator',
        'motion': 'oscillation',
        'a_motion': 'an oscillation',
        'title': 'Sloshing Oscillator',
        'wave_card': 'Regular oscillation',
        # The controller uses the height as the slide's stroke, peak to peak.
        'height_label': 'Stroke H [m], peak to peak',
        'motion_card': 'Slide',
        'plot_note': 'slide position in metres, last 20 s',
        'return_button': 'Go to centre',
        'returning': 'Going to centre…',
    },
}
LOG_HISTORY = 300        # log lines kept for pages that connect later
STARTING_RE = re.compile(r'height ([-+0-9.eE]+) m, period ([-+0-9.eE]+) s')
RETURN_STATUS = {
    0: 'SUCCESS', 1: 'CONTROLLER_INACTIVE', 2: 'BUSY', 3: 'ACTUATOR_NOT_READY',
    4: 'TIMEOUT', 5: 'ACTUATOR_FAULT', 6: 'POSITION_INVALID',
}
LOG_LEVELS = {
    Log.DEBUG: 'debug', Log.INFO: 'info', Log.WARN: 'warn', Log.ERROR: 'error', Log.FATAL: 'fatal',
}
STATIC_FILES = {
    '/': ('index.html', 'text/html; charset=utf-8'),
    '/index.html': ('index.html', 'text/html; charset=utf-8'),
    '/app.js': ('app.js', 'text/javascript; charset=utf-8'),
    '/styles.css': ('styles.css', 'text/css; charset=utf-8'),
}


class WebNode(Node):

    def __init__(self):
        super().__init__('web')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('rig', 'wavemaker')
        rig = self.get_parameter('rig').value
        if rig not in RIG_TEXTS:
            raise ValueError(f"rig must be one of {', '.join(RIG_TEXTS)}, not '{rig}'")
        self.texts = RIG_TEXTS[rig]
        self.declare_parameter('idle_timeout_s', 7200.0)
        self.declare_parameter('idle_warning_s', 600.0)
        self._idle_timeout = float(self.get_parameter('idle_timeout_s').value)
        self._idle_warning = float(self.get_parameter('idle_warning_s').value)

        self._lock = threading.Lock()
        self._state = 'unknown'
        self._message = ''
        self._message_count = 0
        self._started_with = None  # (message count, height, period) of the last start
        self._notice = ''
        self._samples = deque()   # (index, time, kind, value)
        self._sample_count = 0
        self._logs = deque(maxlen=LOG_HISTORY)  # (index, entry)
        self._log_count = 0
        # Logger names use dots: /wavemakers/ladertanken/controller logs as
        # wavemakers.ladertanken.controller.
        self._log_prefix = self.get_namespace().strip('/').replace('/', '.')
        self._last_ack = -math.inf
        self._t0 = time.monotonic()
        self._last_activity = time.monotonic()  # last motion (wave or setpoint), or Keep running
        self._idle_warned = False
        self._auto_stopping = False

        command_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        status_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._height_pub = self.create_publisher(Float64, 'height', command_qos)
        self._period_pub = self.create_publisher(Float64, 'period', command_qos)
        self._start_pub = self.create_publisher(Bool, 'start', command_qos)
        self._stop_pub = self.create_publisher(Bool, 'stop', command_qos)
        self._heartbeat_pub = self.create_publisher(Bool, 'heartbeat', qos_profile_sensor_data)

        # The bridge and controller name these after the rig: wavemaker_state, oscillator_state...
        self.create_subscription(String, f'{rig}_state', self._on_state, status_qos)
        self.create_subscription(String, f'{rig}_message', self._on_message, status_qos)
        self.create_subscription(
            Float64, f'{rig}_setpoint', lambda msg: self._on_sample('setpoint', msg), 10)
        self.create_subscription(
            Float64, f'{rig}_position', lambda msg: self._on_sample('position', msg), 10)
        # Matches the /rosout publishers (reliable, transient local), so a page opened just after
        # the controller starts still gets its configure and activate lines.
        rosout_qos = QoSProfile(
            depth=100, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Log, '/rosout', self._on_log, rosout_qos)
        self._return_client = self.create_client(ReturnToUpright, 'return_to_upright')
        self._manager_client = self.create_client(
            ManageLifecycleNodes, 'lifecycle_manager/manage_nodes')
        self.create_timer(0.1, self._send_heartbeat)
        if self._idle_timeout > 0.0:
            self.create_timer(1.0, self._check_idle)

    # ROS callbacks

    def _on_state(self, msg):
        with self._lock:
            self._state = msg.data

    def _on_message(self, msg):
        match = STARTING_RE.search(msg.data)
        with self._lock:
            self._message = msg.data
            self._message_count += 1
            if match:
                self._started_with = (
                    self._message_count, float(match.group(1)), float(match.group(2)))

    def _on_sample(self, kind, msg):
        now = self._now()
        with self._lock:
            if kind == 'setpoint':
                # The controller publishes setpoints only while it moves the paddle, for any
                # goal (bridge or play_dat) or a return to upright: that is activity.
                self._last_activity = time.monotonic()
                self._idle_warned = False
            self._sample_count += 1
            self._samples.append((self._sample_count, now, kind, msg.data))
            while self._samples and self._samples[0][1] < now - PLOT_WINDOW_S:
                self._samples.popleft()

    def _on_log(self, msg):
        prefix = self._log_prefix
        if prefix and msg.name != prefix and not msg.name.startswith(prefix + '.'):
            return
        entry = {
            'stamp': msg.stamp.sec + msg.stamp.nanosec * 1e-9,
            'level': LOG_LEVELS.get(msg.level, str(msg.level)),
            'node': msg.name[len(prefix):].lstrip('.') or msg.name,
            'text': msg.msg,
        }
        with self._lock:
            self._log_count += 1
            self._logs.append((self._log_count, entry))

    def _send_heartbeat(self):
        if self.page_alive():
            self._heartbeat_pub.publish(Bool(data=True))

    def _check_idle(self):
        now = time.monotonic()
        with self._lock:
            if self._auto_stopping:
                return
            if self._state in ACTIVE_STATES:
                self._last_activity = now
                self._idle_warned = False
                return
            idle = now - self._last_activity
            remaining = self._idle_timeout - idle
            warn = remaining <= self._idle_warning and not self._idle_warned
            if warn:
                self._idle_warned = True
            stop = remaining <= 0.0
            if stop:
                self._auto_stopping = True
        if stop:
            threading.Thread(target=self._auto_stop, daemon=True).start()
        elif warn:
            self.get_logger().warn(
                f"No {self.texts['motion']} for {format_duration(idle)}: the "
                f"{self.texts['machine']} stops in {format_duration(remaining)}. Press Keep "
                'running on the page to stay on.')

    def _auto_stop(self):
        machine, motion = self.texts['machine'], self.texts['motion']
        self.get_logger().warn(
            f'No {motion} for {format_duration(self._idle_timeout)}: stopping the {machine}')
        with self._lock:
            self._notice = (
                f'Stopped after {format_duration(self._idle_timeout)} without '
                f"{self.texts['a_motion']}. Start the {machine} again to continue.")
        self._shutdown_managed_nodes()
        time.sleep(EXIT_DELAY_S)
        # Ends spin() in main(); the launch file then shuts the whole launch down.
        rclpy.try_shutdown()

    def _shutdown_managed_nodes(self):
        # The ordered path: the lifecycle manager deactivates, cleans up and shuts down the
        # bridge and the controller. If it fails, the launch shutdown that follows this server's
        # exit still stops the controller.
        if not self._manager_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(
                'Lifecycle manager not available; exiting, the launch stops the nodes')
            return
        request = ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.SHUTDOWN)
        done = threading.Event()
        future = self._manager_client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(SHUTDOWN_TIMEOUT_S):
            self.get_logger().error('No reply from the lifecycle manager; exiting anyway')
        elif not future.result().success:
            self.get_logger().error('The lifecycle manager reported a failed shutdown')
        else:
            self.get_logger().info('Managed nodes shut down')

    # Used by the HTTP handler

    def _now(self):
        return time.monotonic() - self._t0

    def acknowledge(self):
        with self._lock:
            self._last_ack = time.monotonic()

    def page_alive(self):
        with self._lock:
            return time.monotonic() - self._last_ack < ACK_TIMEOUT_S

    def snapshot(self, after_sample, after_log):
        """Return the page update and the indices of the last sample and log line it contains."""
        with self._lock:
            idle_stop_in = None
            if (self._idle_timeout > 0.0 and not self._auto_stopping and
                    self._state not in ACTIVE_STATES):
                remaining = self._idle_timeout - (time.monotonic() - self._last_activity)
                if remaining <= self._idle_warning:
                    idle_stop_in = max(remaining, 0.0)
            samples = [[t, kind, value] for index, t, kind, value in self._samples
                       if index > after_sample]
            logs = [entry for index, entry in self._logs if index > after_log]
            update = {
                'namespace': self.get_namespace(),
                'state': self._state,
                'message': self._message,
                'notice': self._notice,
                'time': self._now(),
                'idle_timeout': self._idle_timeout,
                'idle_stop_in': idle_stop_in,
                'samples': samples,
                'logs': logs,
            }
            return update, self._sample_count, self._log_count

    def start(self, height, period):
        with self._lock:
            if self._auto_stopping:
                return False, f"The {self.texts['machine']} is stopping after the idle timeout."
            state = self._state
            first_message = self._message_count
            self._notice = ''
        if state != 'idle':
            return False, (
                f"The bridge is {state}; it must be idle to start {self.texts['a_motion']}.")
        self._height_pub.publish(Float64(data=height))
        self._period_pub.publish(Float64(data=period))
        time.sleep(START_DELAY_S)
        self._start_pub.publish(Bool(data=True))
        threading.Thread(
            target=self._check_start, args=(height, period, first_message), daemon=True).start()
        return True, 'Start sent.'

    def _check_start(self, height, period, first_message):
        # height, period and start are separate topics and DDS does not order them, so check
        # that the bridge started with the values sent, and stop the wave if it did not.
        deadline = time.monotonic() + START_CHECK_S
        while time.monotonic() < deadline:
            with self._lock:
                started_with = self._started_with
            if started_with and started_with[0] > first_message:
                _, used_height, used_period = started_with
                # The bridge prints the height to 4 and the period to 3 decimals.
                if abs(used_height - height) > 6e-5 or abs(used_period - period) > 6e-4:
                    self._stop_pub.publish(Bool(data=True))
                    with self._lock:
                        self._notice = (
                            f'Stopped: the bridge started with height {used_height} m, period '
                            f'{used_period} s instead of {height} m, {period} s. Start again.')
                return
            time.sleep(0.05)

    def stop(self):
        self._stop_pub.publish(Bool(data=True))
        return True, 'Stop sent.'

    def keep_running(self):
        with self._lock:
            if self._auto_stopping:
                return False, (
                    f"The {self.texts['machine']} is already stopping after the idle timeout.")
            self._last_activity = time.monotonic()
            self._idle_warned = False
        machine, a_motion = self.texts['machine'], self.texts['a_motion']
        timeout = format_duration(self._idle_timeout)
        self.get_logger().info(
            f'Keep running pressed: the {machine} stops after {timeout} without {a_motion} '
            'from now')
        return True, f'Keeping the {machine} on for another {timeout} without {a_motion}.'

    def return_to_upright(self):
        with self._lock:
            if self._auto_stopping:
                return False, f"The {self.texts['machine']} is stopping after the idle timeout."
        if not self._return_client.wait_for_service(timeout_sec=1.0):
            return False, 'The return_to_upright service is not available.'
        request = ReturnToUpright.Request(requester='web', tolerance=0.0)
        done = threading.Event()
        future = self._return_client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(RETURN_TIMEOUT_S):
            return False, 'No reply from return_to_upright.'
        response = future.result()
        status = RETURN_STATUS.get(response.status, str(response.status))
        text = f'{status}: {response.message}' if response.message else status
        return response.status == ReturnToUpright.Response.SUCCESS, text


def make_handler(node, html_dir):

    class Handler(BaseHTTPRequestHandler):

        def log_message(self, *args):
            pass  # the page acknowledges every event; logging each request would flood

        def do_GET(self):
            if self.path == '/api/events':
                self._events()
            elif self.path == '/api/config':
                body = json.dumps(node.texts).encode()
                self._send(HTTPStatus.OK, body, 'application/json')
            elif self.path in STATIC_FILES:
                name, content_type = STATIC_FILES[self.path]
                self._send(HTTPStatus.OK, (html_dir / name).read_bytes(), content_type)
            else:
                self._send(HTTPStatus.NOT_FOUND, b'Not found', 'text/plain')

        def do_POST(self):
            if self.path == '/api/ack':
                node.acknowledge()
                self._send(HTTPStatus.NO_CONTENT, b'', 'text/plain')
                return
            if self.path == '/api/start':
                try:
                    body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
                    height, period = float(body['height']), float(body['period'])
                except (KeyError, TypeError, ValueError):
                    self._json(False, 'Send height and period as numbers.')
                    return
                if not (math.isfinite(height) and math.isfinite(period) and
                        height > 0.0 and period > 0.0):
                    self._json(False, 'Height and period must be positive.')
                    return
                self._json(*node.start(height, period))
            elif self.path == '/api/stop':
                self._json(*node.stop())
            elif self.path == '/api/keep_running':
                self._json(*node.keep_running())
            elif self.path == '/api/return':
                self._json(*node.return_to_upright())
            else:
                self._send(HTTPStatus.NOT_FOUND, b'Not found', 'text/plain')

        def _events(self):
            self.send_response(HTTPStatus.OK)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            last_sample = 0
            last_log = 0
            try:
                while True:
                    update, last_sample, last_log = node.snapshot(last_sample, last_log)
                    self.wfile.write(f'data: {json.dumps(update)}\n\n'.encode())
                    self.wfile.flush()
                    time.sleep(EVENT_PERIOD_S)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def _json(self, ok, message):
            body = json.dumps({'ok': ok, 'message': message}).encode()
            self._send(HTTPStatus.OK, body, 'application/json')

        def _send(self, status, body, content_type):
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    return Handler


def format_duration(seconds):
    """Return seconds as '2 h', '1 h 50 min' or '10 min', rounded to whole minutes."""
    hours, minutes = divmod(max(round(seconds / 60.0), 0), 60)
    if hours and minutes:
        return f'{hours} h {minutes} min'
    return f'{hours} h' if hours else f'{minutes} min'


def page_url(host, port):
    """Return the page's address by hostname, as the lab Pis announce it (<hostname>.local)."""
    if host not in ('0.0.0.0', '::', ''):
        return f'http://{host}:{port}/'
    name = socket.gethostname()
    return f'http://{name if "." in name else name + ".local"}:{port}/'


def main():
    rclpy.init()
    node = WebNode()
    html_dir = Path(get_package_share_directory('rig_web')) / 'html'
    host = node.get_parameter('host').value
    port = node.get_parameter('port').value
    server = ThreadingHTTPServer((host, port), make_handler(node, html_dir))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    node.get_logger().info(f"Open the {node.texts['machine']} page at {page_url(host, port)}")

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
