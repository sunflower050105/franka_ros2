#!/usr/bin/env python3
# Copyright (c) 2025
# Licensed under the Apache License, Version 2.0
#
# Franka Research 3 – Keyboard Teleoperation Node
# ------------------------------------------------
# Reads single-character key presses from the terminal and publishes
# Float64MultiArray velocity commands to:
#   /velocity_command_node/target_velocities  (default)
#
# The velocity_command_node must be running in 'topic' mode to forward
# these commands to /joint_velocity_controller/commands.
#
# Alternatively, set target_topic to /joint_velocity_controller/commands
# to bypass the velocity_command_node entirely.
#
# Key bindings (printed on startup):
#   1-7        select active joint (J1 … J7)
#   + / =      increase velocity of selected joint by velocity_step
#   - / _      decrease velocity of selected joint by velocity_step
#   s          stop selected joint (set to 0)
#   a          stop ALL joints
#   q / ESC    quit – sends zero-velocity stop

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

NUM_JOINTS = 7
FR3_MAX_JOINT_VELOCITIES = [2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26]

BANNER = """
╔══════════════════════════════════════════════════════════╗
║   Franka FR3 – Keyboard Joint Velocity Teleoperation     ║
╠══════════════════════════════════════════════════════════╣
║  Keys:                                                   ║
║   1-7     select active joint (J1 … J7)                  ║
║   + / =   increase joint velocity by step                ║
║   - / _   decrease joint velocity by step                ║
║   s       stop selected joint (vel → 0)                  ║
║   a       stop ALL joints                                ║
║   q / ESC quit (sends zero-velocity stop)                ║
╚══════════════════════════════════════════════════════════╝
"""


def _get_key(settings, timeout: float = 0.05) -> str:
    """Non-blocking single-character read with timeout."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class KeyboardTeleopNode(Node):
    """Keyboard teleop node for Franka FR3 joint velocity control.

    ROS Parameters
    --------------
    velocity_step : float
        Amount added / subtracted per key press [rad/s]. Default: 0.05
    max_velocity_scale : float
        Fraction of FR3 hardware limits used as clamp boundary. Default: 0.1
    publish_rate : float
        Publishing frequency [Hz]. Default: 50
    target_topic : str
        Topic to publish commands on.
        Default: /velocity_command_node/target_velocities
    """

    def __init__(self):
        super().__init__('keyboard_teleop_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('velocity_step', 0.05)
        self.declare_parameter('max_velocity_scale', 0.1)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter(
            'target_topic', '/velocity_command_node/target_velocities'
        )

        self.step = (
            self.get_parameter('velocity_step').get_parameter_value().double_value
        )
        self.scale = (
            self.get_parameter('max_velocity_scale').get_parameter_value().double_value
        )
        rate_hz = (
            self.get_parameter('publish_rate').get_parameter_value().double_value
        )
        topic = (
            self.get_parameter('target_topic').get_parameter_value().string_value
        )

        # ── State ─────────────────────────────────────────────────────────────
        self.limits = [v * self.scale for v in FR3_MAX_JOINT_VELOCITIES]
        self.velocities = [0.0] * NUM_JOINTS
        self.active_joint = 0          # 0-indexed
        self._running = True

        # ── Publisher ─────────────────────────────────────────────────────────
        self.pub = self.create_publisher(Float64MultiArray, topic, 10)

        # ── Timer ─────────────────────────────────────────────────────────────
        self.create_timer(1.0 / rate_hz, self._publish_cb)

        # ── Save terminal settings ─────────────────────────────────────────────
        self._term_settings = termios.tcgetattr(sys.stdin)

        print(BANNER)
        self._print_state()

        self.get_logger().info(
            f'Keyboard teleop started | topic={topic} | '
            f'step={self.step} rad/s | max_scale={self.scale}'
        )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _clamp(self, value: float, idx: int) -> float:
        lim = self.limits[idx]
        return max(-lim, min(lim, value))

    def _print_state(self):
        parts = []
        for i, v in enumerate(self.velocities):
            if i == self.active_joint:
                parts.append(f'[J{i+1}:{v:+.3f}]')
            else:
                parts.append(f' J{i+1}:{v:+.3f} ')
        print('\r  ' + '  '.join(parts), end='', flush=True)

    # ── Timer callback (read key + publish) ───────────────────────────────────

    def _publish_cb(self):
        if not self._running:
            return

        key = _get_key(self._term_settings, timeout=0.0)
        if key:
            self._handle_key(key)

        msg = Float64MultiArray()
        msg.data = list(self.velocities)
        self.pub.publish(msg)

    # ── Key handling ──────────────────────────────────────────────────────────

    def _handle_key(self, key: str):
        if key in '1234567':
            self.active_joint = int(key) - 1
            self._print_state()

        elif key in '+=':
            self.velocities[self.active_joint] = self._clamp(
                self.velocities[self.active_joint] + self.step,
                self.active_joint,
            )
            self._print_state()

        elif key in '-_':
            self.velocities[self.active_joint] = self._clamp(
                self.velocities[self.active_joint] - self.step,
                self.active_joint,
            )
            self._print_state()

        elif key == 's':
            self.velocities[self.active_joint] = 0.0
            self._print_state()

        elif key == 'a':
            self.velocities = [0.0] * NUM_JOINTS
            self._print_state()

        elif key in ('q', '\x1b'):      # q or ESC
            print('\nQuitting keyboard teleop …')
            self._running = False
            self.velocities = [0.0] * NUM_JOINTS
            msg = Float64MultiArray()
            msg.data = list(self.velocities)
            self.pub.publish(msg)
            raise SystemExit

    # ── Restore terminal on shutdown ──────────────────────────────────────────

    def destroy_node(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._term_settings)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        # Restore terminal just in case
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node._term_settings)
        # Send final stop
        stop = Float64MultiArray()
        stop.data = [0.0] * NUM_JOINTS
        node.pub.publish(stop)
        node.get_logger().info('Keyboard teleop stopped. Zero velocities sent.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
