#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time


class ExampleVel(Node):
    """
    Demo node that continuously oscillates Joint 6 and 7
    between +0.1 and -0.1 rad/s every 3 seconds.
    """

    def __init__(self):
        super().__init__('example_vel')

        # ========================= CONFIG =========================
        self.command_topic = 'velocity_command_node/target_velocities'
        self.num_joints = 7
        self.velocity = 0.1      # rad/s
        self.period = 3.0        # seconds (3 sec per direction)
        # =========================================================

        # Publisher
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            self.command_topic,
            10
        )

        # Timer to publish at 100Hz
        self.timer = self.create_timer(0.01, self.publish_callback)  # 100 Hz

        self.start_time = time.time()
        self.get_logger().info('Example Velocity Node started')
        self.get_logger().info(f'Publishing to: {self.command_topic}')
        self.get_logger().info('Oscillating Joint 6 & 7 every 3 seconds')

    def publish_callback(self):
        """Called at 100Hz to publish current velocity command"""
        elapsed = time.time() - self.start_time
        phase = (elapsed % (2 * self.period))   # full cycle = 6 seconds

        if phase < self.period:
            vel = self.velocity      # +0.1 rad/s
            direction = "FORWARD"
        else:
            vel = -self.velocity     # -0.1 rad/s
            direction = "REVERSE"

        # Build message
        msg = Float64MultiArray()
        msg.data = [0.0] * self.num_joints
        msg.data[5] = vel   # Joint 6
        msg.data[6] = vel   # Joint 7

        self.publisher_.publish(msg)

        # Log every second
        if int(elapsed) % 1 == 0 and int(elapsed * 100) % 100 == 0:
            self.get_logger().info(
                f'{direction} → Joint6={vel:+.2f} rad/s, Joint7={vel:+.2f} rad/s'
            )

    def stop(self):
        """Send zero velocity"""
        msg = Float64MultiArray()
        msg.data = [0.0] * self.num_joints
        self.publisher_.publish(msg)
        self.get_logger().info('Sent zero velocity (stopped)')


def main(args=None):
    rclpy.init(args=args)
    node = ExampleVel()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user')
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()