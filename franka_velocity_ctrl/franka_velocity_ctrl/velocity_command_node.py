#!/usr/bin/env python3
# Copyright (c) 2025
# Licensed under the Apache License, Version 2.0
#
# Franka Research 3 – Velocity Command Node
# ------------------------------------------
# Publishes joint velocity commands to the JointGroupVelocityController.
#
# Three operating modes (set via 'mode' parameter):
#
#   demo   – Sends a sine-wave profile on all 7 joints so you can verify
#             the controller is working without writing any external code.
#
#   topic  – Forwards commands received on ~/target_velocities to the
#             hardware controller.  Use this mode with keyboard_teleop_node
#             or any external planner.
#
#   zero   – Continuously publishes zero velocities (safe commissioning mode).
#
# Published topics:
#   /joint_velocity_controller/commands   (std_msgs/Float64MultiArray)
#   ~/target_velocities                   (std_msgs/Float64MultiArray) – echo
#
# Subscribed topics (topic mode only):
#   ~/target_velocities                   (std_msgs/Float64MultiArray)

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

NUM_JOINTS = 7

# FR3 hardware joint velocity limits [rad/s]  (from Franka documentation)
FR3_MAX_JOINT_VELOCITIES = [2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26]

# Phase offsets for the demo sine wave (spread joints so motion looks natural)
_DEMO_PHASE_OFFSETS = [i * (2.0 * math.pi / NUM_JOINTS) for i in range(NUM_JOINTS)]


class VelocityCommandNode(Node):
    """Velocity command node for the Franka FR3.

    ROS 2 Parameters
    ----------------
    mode : str
        Operating mode: ``demo`` | ``topic`` | ``zero``.  Default: ``demo``.
    max_velocity_scale : float
        Fraction of FR3 hardware joint-velocity limits used as the clamp
        boundary (0.0 – 1.0).  Default: ``0.1``.
    publish_rate : float
        Command publishing frequency [Hz].  Default: ``100.0``.
    demo_amplitude : float
        Amplitude of the sine-wave profile in demo mode [rad/s].
        Clamped to max_velocity_scale × hardware limit.  Default: ``0.1``.
    demo_frequency : float
        Frequency of the sine-wave profile in demo mode [Hz].  Default: ``0.2``.
    command_topic : str
        Topic to publish hardware commands to.
        Default: ``/joint_velocity_controller/commands``.
    """

    def __init__(self):
        super().__init__('velocity_command_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('mode', 'demo')
        self.declare_parameter('max_velocity_scale', 0.1)
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('demo_amplitude', 0.1)
        self.declare_parameter('demo_frequency', 0.2)
        self.declare_parameter(
            'command_topic', '/joint_velocity_controller/commands'
        )

        self._mode = (
            self.get_parameter('mode').get_parameter_value().string_value
        )
        self._scale = (
            self.get_parameter('max_velocity_scale').get_parameter_value().double_value
        )
        rate_hz = (
            self.get_parameter('publish_rate').get_parameter_value().double_value
        )
        self._demo_amp = (
            self.get_parameter('demo_amplitude').get_parameter_value().double_value
        )
        self._demo_freq = (
            self.get_parameter('demo_frequency').get_parameter_value().double_value
        )
        cmd_topic = (
            self.get_parameter('command_topic').get_parameter_value().string_value
        )

        # Validate mode
        valid_modes = ('demo', 'topic', 'zero')
        if self._mode not in valid_modes:
            self.get_logger().warn(
                f"Unknown mode '{self._mode}', falling back to 'zero'. "
                f"Valid modes: {valid_modes}"
            )
            self._mode = 'zero'

        # Compute per-joint velocity limits
        self._limits = [v * self._scale for v in FR3_MAX_JOINT_VELOCITIES]

        # Clamp demo amplitude to the scaled limits
        self._demo_amp = min(self._demo_amp, min(self._limits))

        # ── State ─────────────────────────────────────────────────────────────
        self._target_velocities = [0.0] * NUM_JOINTS  # used in topic mode
        self._start_time = time.monotonic()

        # ── Publishers ────────────────────────────────────────────────────────
        self._cmd_pub = self.create_publisher(
            Float64MultiArray, cmd_topic, 10
        )
        self._echo_pub = self.create_publisher(
            Float64MultiArray, '~/target_velocities', 10
        )

        # ── Subscriber (topic mode) ───────────────────────────────────────────
        self._sub = self.create_subscription(
            Float64MultiArray,
            '~/target_velocities',
            self._target_cb,
            10,
        )

        # ── Timer ─────────────────────────────────────────────────────────────
        self.create_timer(1.0 / rate_hz, self._publish_cb)

        self.get_logger().info(
            f'velocity_command_node started | mode={self._mode} | '
            f'max_scale={self._scale} | rate={rate_hz} Hz | '
            f'cmd_topic={cmd_topic}'
        )

        if self._mode == 'demo':
            self.get_logger().info(
                f'Demo mode: amplitude={self._demo_amp:.4f} rad/s | '
                f'frequency={self._demo_freq:.2f} Hz'
            )
        elif self._mode == 'topic':
            self.get_logger().info(
                'Topic mode: publish to ~/target_velocities to command joints'
            )

    # ── Subscriber callback ───────────────────────────────────────────────────

    def _target_cb(self, msg: Float64MultiArray):
        """Receive external velocity commands (topic mode)."""
        if self._mode != 'topic':
            return

        data = list(msg.data)
        if len(data) != NUM_JOINTS:
            self.get_logger().warn(
                f'Expected {NUM_JOINTS} velocity values, got {len(data)}. '
                'Message ignored.'
            )
            return

        # Clamp each joint to its scaled hardware limit
        self._target_velocities = [
            max(-lim, min(lim, v))
            for v, lim in zip(data, self._limits)
        ]

    # ── Timer callback ────────────────────────────────────────────────────────

    def _publish_cb(self):
        """Compute and publish the velocity command at the configured rate."""
        velocities = self._compute_velocities()

        cmd = Float64MultiArray()
        cmd.data = velocities
        self._cmd_pub.publish(cmd)

        echo = Float64MultiArray()
        echo.data = velocities
        self._echo_pub.publish(echo)

    # ── Velocity computation ──────────────────────────────────────────────────

    def _compute_velocities(self) -> list:
        """Return the velocity vector for the current mode."""
        if self._mode == 'zero':
            return [0.0] * NUM_JOINTS

        if self._mode == 'topic':
            return list(self._target_velocities)

        # demo – sine wave
        t = time.monotonic() - self._start_time
        omega = 2.0 * math.pi * self._demo_freq
        return [
            self._demo_amp * math.sin(omega * t + phase)
            for phase in _DEMO_PHASE_OFFSETS
        ]


def main(args=None):
    rclpy.init(args=args)
    node = VelocityCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send zero velocities before shutting down
        stop = Float64MultiArray()
        stop.data = [0.0] * NUM_JOINTS
        node._cmd_pub.publish(stop)
        node.get_logger().info('velocity_command_node stopped. Zero velocities sent.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
