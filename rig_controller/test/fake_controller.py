#!/usr/bin/env python3
"""
Fake controller for testing bridge_node and the web page without hardware.

Serves the move_<rig> action like the real controller, without a drive (--rig wavemaker, the
default, or oscillator; the topics are named after the rig the same way):
  - accepts one goal at a time and rejects goals while one is running ("busy"),
  - runs a regular wave until it is cancelled, sending feedback at 10 Hz and the setpoint and
    a slightly lagging "measured" position on <rig>_setpoint / <rig>_position at 50 Hz,
  - plays a goal with positions and sample_interval instead (each goal picks its type, as in
    the real controller): blends from the held position to the first sample, interpolates
    linearly and succeeds at the last sample; a goal with both is rejected,
  - holds its position after a cancel, and serves return_to_upright (back to 0 in 1.5 s),
  - optionally rejects every goal (--reject) or aborts a goal after N s (--abort-after),
  - declares its rig's yaml block as parameters when the namespace is /<prefix>/<name>, so
    play_dat's cross-check finds the limits; -p changes one, to see the warning.

Do not run it next to the real controller: both would serve the same action.

    source ~/ros_ws/install/setup.bash
    ros2 run rig_controller fake_controller --ros-args -r __ns:=/wavemakers/ladertanken
    ros2 run rig_controller fake_controller --abort-after 5 \
      --ros-args -r __ns:=/wavemakers/ladertanken
    ros2 run rig_controller fake_controller --rig oscillator \
      --ros-args -r __ns:=/oscillators/sloshing
    ros2 run rig_controller fake_controller \
      --ros-args -r __ns:=/wavemakers/ladertanken -p max_velocity_rpm:=200.0
"""

import argparse
import math
import sys
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from rig_interfaces.action import MoveRig
from rig_interfaces.srv import ReturnToUpright
import rig_limits  # installed next to this program
from std_msgs.msg import Float64

LOOP_S = 0.02            # setpoint rate, as the real controller at poll_interval_ms 20
LAG_S = 0.04             # the fake "measured" position trails the setpoint by this much
RETURN_DURATION_S = 1.5
# Blend into a pregenerated goal as the real controller does with its defaults
# (return_to_upright_max_velocity_mps, return_min_duration_s, return_default_tolerance_m).
BLEND_MAX_VELOCITY_MPS = 0.1
BLEND_MIN_DURATION_S = 1.0
BLEND_TOLERANCE_M = 0.002
QUINTIC_PEAK_VELOCITY_FACTOR = 1.875


def quintic(u):
    u = min(1.0, max(0.0, u))
    return 10 * u**3 - 15 * u**4 + 6 * u**5


class FakeController(Node):

    def __init__(self, reject, abort_after, rig='wavemaker'):
        super().__init__('controller')
        self._reject = reject
        self._abort_after = abort_after
        self._busy = False
        self._position = 0.0     # last setpoint, held after a stop
        self._lock = threading.Lock()
        self._declare_rig_parameters(rig)
        # Named after the rig, as in the real controller (oscillator_setpoint, ...).
        self._setpoint_pub = self.create_publisher(Float64, f'{rig}_setpoint', 10)
        self._measured_pub = self.create_publisher(Float64, f'{rig}_position', 10)
        self.create_service(
            ReturnToUpright, 'return_to_upright', self._return_to_upright,
            callback_group=ReentrantCallbackGroup())
        self._server = ActionServer(
            self, MoveRig, f'move_{rig}',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=ReentrantCallbackGroup())
        self.get_logger().info(
            f'Fake controller ready (rig={rig}, reject={reject}, abort_after={abort_after})')

    def _declare_rig_parameters(self, rig):
        """
        Declare the rig's yaml block as parameters, as the real controller has them.

        So play_dat's cross-check has something to compare. Without a /<prefix>/<name> namespace
        or a yaml block, none: the fake still serves goals. Lists are left out (an empty one has
        no type).
        """
        prefix = rig_limits.RIGS[rig][0]
        namespace = self.get_namespace()
        parts = namespace.strip('/').split('/')
        if len(parts) != 2 or parts[0] != prefix:
            self.get_logger().info(
                f'No rig parameters: namespace {namespace} is not /{prefix}/<name>')
            return
        config_path = rig_limits.default_config_path(rig)
        try:
            params = rig_limits.read_block(rig, parts[1], config_path)
        except (OSError, ValueError) as e:
            self.get_logger().warn(f'No rig parameters: {e}')
            return
        for key, value in params.items():
            if isinstance(value, (bool, int, float, str)):
                self.declare_parameter(key, value)  # a -p override wins
        self.get_logger().info(f'Parameters of {parts[1]} from {config_path}')

    def _on_goal(self, goal):
        with self._lock:
            if self._reject:
                self.get_logger().warn('Rejecting goal (--reject)')
                return GoalResponse.REJECT
            if self._busy:
                self.get_logger().warn('Rejecting goal: another goal is running')
                return GoalResponse.REJECT
            if len(goal.positions) > 0:
                if goal.height != 0.0 or goal.period != 0.0:
                    self.get_logger().warn(
                        'Rejecting goal: both positions and height/period; send one or the other')
                    return GoalResponse.REJECT
                if not math.isfinite(goal.sample_interval) or goal.sample_interval <= 0.0:
                    self.get_logger().warn(
                        'Rejecting goal: pregenerated goal needs a positive sample_interval')
                    return GoalResponse.REJECT
                if not all(math.isfinite(p) for p in goal.positions):
                    self.get_logger().warn('Rejecting goal: positions must be finite')
                    return GoalResponse.REJECT
            elif goal.height <= 0.0 or goal.period <= 0.0:
                self.get_logger().warn('Rejecting goal: height and period must be > 0')
                return GoalResponse.REJECT
            self._busy = True
        if len(goal.positions) > 0:
            self.get_logger().info(
                f'Accepted pregenerated goal: {len(goal.positions)} samples at '
                f'{goal.sample_interval} s ({self._samples_duration(goal):.1f} s)')
        else:
            self.get_logger().info(
                f'Accepted goal: height {goal.height} m, period {goal.period} s')
        return GoalResponse.ACCEPT

    @staticmethod
    def _samples_duration(goal):
        return goal.sample_interval * (len(goal.positions) - 1)

    def _on_cancel(self, goal_handle):
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    def _publish(self, setpoint, measured):
        self._position = setpoint
        self._setpoint_pub.publish(Float64(data=setpoint))
        self._measured_pub.publish(Float64(data=measured))

    def _return_to_upright(self, request, response):
        with self._lock:
            if self._busy:
                response.status = ReturnToUpright.Response.BUSY
                response.message = 'A goal is running (fake)'
                return response
            self._busy = True
        try:
            origin = self._position
            start = time.monotonic()
            while (elapsed := time.monotonic() - start) < RETURN_DURATION_S:
                u = elapsed / RETURN_DURATION_S
                blend = 10 * u**3 - 15 * u**4 + 6 * u**5
                u_lag = max(0.0, elapsed - LAG_S) / RETURN_DURATION_S
                blend_lag = 10 * u_lag**3 - 15 * u_lag**4 + 6 * u_lag**5
                self._publish(origin * (1.0 - blend), origin * (1.0 - blend_lag))
                time.sleep(LOOP_S)
            self._publish(0.0, 0.0)
        finally:
            with self._lock:
                self._busy = False
        self.get_logger().info(f'Returned to upright from {origin:.4f} m (requester '
                               f'{request.requester!r})')
        response.status = ReturnToUpright.Response.SUCCESS
        response.final_position = 0.0
        response.message = 'At upright (fake)'
        return response

    @staticmethod
    def _regular_motion(goal, origin):
        """Position at a time: fades from the held position into the wave over one period."""
        amplitude = goal.height / 2.0
        omega = 2.0 * math.pi / goal.period

        def position_at(t):
            fade = min(1.0, t / goal.period)
            return (1.0 - fade) * origin + fade * amplitude * math.sin(omega * t), False
        return position_at

    @staticmethod
    def _pregenerated_motion(goal, origin):
        """Position at a time: blend to the first sample, then interpolate; True at the end."""
        positions = list(goal.positions)
        dt = goal.sample_interval
        distance = abs(positions[0] - origin)
        blend = 0.0 if distance <= BLEND_TOLERANCE_M else max(
            QUINTIC_PEAK_VELOCITY_FACTOR * distance / BLEND_MAX_VELOCITY_MPS,
            BLEND_MIN_DURATION_S)
        samples_duration = dt * (len(positions) - 1)

        def position_at(t):
            if t < blend:
                return origin + quintic(t / blend) * (positions[0] - origin), False
            t -= blend
            if t >= samples_duration:
                return positions[-1], True
            index = min(int(t / dt), len(positions) - 2)
            fraction = t / dt - index
            return positions[index] + fraction * (positions[index + 1] - positions[index]), False
        return position_at

    def _execute(self, goal_handle):
        goal = goal_handle.request
        result = MoveRig.Result()
        feedback = MoveRig.Feedback()
        origin = self._position
        pregenerated = len(goal.positions) > 0
        motion = (self._pregenerated_motion if pregenerated else self._regular_motion)(
            goal, origin)
        start = time.monotonic()
        cycle = 0
        try:
            while rclpy.ok():
                elapsed = time.monotonic() - start
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Goal canceled; holding position (fake)'
                    self.get_logger().info('Goal canceled')
                    return result
                if self._abort_after is not None and elapsed >= self._abort_after:
                    goal_handle.abort()
                    result.success = False
                    result.message = 'Simulated drive fault (fake)'
                    self.get_logger().warn('Goal aborted (--abort-after)')
                    return result
                position, done = motion(elapsed)
                measured, _ = motion(max(0.0, elapsed - LAG_S))
                if done:
                    self._publish(position, position)
                    goal_handle.succeed()
                    result.success = True
                    result.message = 'Pregenerated trajectory completed (fake)'
                    self.get_logger().info('Pregenerated goal completed')
                    return result
                self._publish(position, measured)
                cycle += 1
                if cycle % 5 == 0:
                    feedback.desired_position = position
                    feedback.actual_position = measured
                    feedback.elapsed_time = elapsed
                    goal_handle.publish_feedback(feedback)
                time.sleep(LOOP_S)
            goal_handle.abort()
            result.message = 'Shutting down'
            return result
        finally:
            with self._lock:
                self._busy = False


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--reject', action='store_true', help='reject every goal')
    parser.add_argument(
        '--abort-after', type=float, default=None, metavar='SECONDS',
        help='abort each goal after this many seconds (simulates a fault)')
    parser.add_argument(
        '--rig', choices=['wavemaker', 'oscillator'], default='wavemaker',
        help='names the action and topics (move_<rig>, <rig>_setpoint, <rig>_position)')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init(args=sys.argv)
    node = FakeController(args.reject, args.abort_after, args.rig)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
