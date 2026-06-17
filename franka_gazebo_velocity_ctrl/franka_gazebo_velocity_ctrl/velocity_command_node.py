#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

class VelocityCommandNode(Node):
    def __init__(self):
        super().__init__('velocity_command_node')
        self.publisher = self.create_publisher(Float64MultiArray, '/joint_velocity_controller/commands', 10)
        self.timer = self.create_timer(0.01, self.publish_velocities)  # 100 Hz
        self.get_logger().info("Velocity command node started!")

    def publish_velocities(self):
        msg = Float64MultiArray()
        # Example: sinusoidal motion on joint 4 and 5
        t = self.get_clock().now().seconds_nanoseconds()[0]
        vel = [
            0.0,
            0.0,
            0.0,
            0.3 * (abs((t % 4) - 2) - 1),   # oscillating joint 4
            0.2 * (abs(((t+1) % 4) - 2) - 1), # oscillating joint 5
            0.0,
            0.0
        ]
        msg.data = vel
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()