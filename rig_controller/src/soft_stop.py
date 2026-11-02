#!/usr/bin/env python3
import time

from gpiozero import Button
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

TICK_S = 0.02
TRIP_FILTR_S = 0.03
RELEASE_HOLD_S = 0.5


class SoftStop(Node):
    def __init__(self):
        super().__init__('soft_stop')
        self.pin = self.declare_parameter('pin', 17).value  # Example GPIO pin number
        self.internal_pull_up = self.declare_parameter('internal_pull_up', True).value
        # True: internal pull-up (default, button between pin and GND)
        # False: external pull-up (more noise-resistant on long cables)

        self.stopped = True
        self.high_since = None
        self.low_since = None

        try:
            if self.internal_pull_up:
                self.button = Button(self.pin, pull_up=True)
            else:
                self.button = Button(self.pin, pull_up=None, active_state=True)
        except Exception as error:
            self.button = None
            self.get_logger().error(
                f'Cannot open GPIO{self.pin}: {error}. Publishing stopped until restarted')
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.publisher = self.create_publisher(Bool, 'soft_stop', qos)
        self.timer = self.create_timer(TICK_S, self.tick)
        self.get_logger().info(f'Soft stop on GPIO{self.pin}, starting stopped')

    def pin_high(self):
        if self.button is None:
            return True
        try:
            pressed = self.button.is_pressed

        except Exception:
            return True
        return not pressed if self.internal_pull_up else pressed

    def tick(self):
        now = time.monotonic()
        if self.pin_high():
            self.low_since = None
            if self.high_since is None:
                self.high_since = now
            if not self.stopped and now - self.high_since >= TRIP_FILTR_S:
                self.stopped = True
                self.get_logger().warn('Soft stop triggered: stopping the motion')
        else:
            if self.high_since is not None and not self.stopped:
                self.get_logger().info(
                    f'Ignored {1000 * (now - self.high_since):.0f} ms dropout on GPIO{self.pin}')
            self.high_since = None
            if self.low_since is None:
                self.low_since = now
            if self.stopped and now - self.low_since >= RELEASE_HOLD_S:
                self.stopped = False
                self.get_logger().info('Soft stop released: motion can start again')

        self.publisher.publish(Bool(data=self.stopped))


def main(args=None):
    rclpy.init(args=args)
    node = SoftStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
