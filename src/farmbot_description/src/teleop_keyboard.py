#!/usr/bin/env python3
# Copyright 2026 Sathvik
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Keyboard teleoperation node for FarmBot Genesis gantry joints.

Publishes individual joint position commands to:
  /x_axis_joint/position
  /y_axis_joint/position
  /z_axis_joint/position

Controls:
  W/S        - Move X axis positive/negative (along tracks)
  A/D        - Move Y axis negative/positive (along gantry beam)
  Q/E        - Move Z axis positive/negative (down/up)
  R          - Reset all joints to home (0, 0, 0)
  ESC        - Quit
"""

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64


_STEP_SIZE = 0.05  # meters per keypress

# Joint limits from URDF spec (meters)
_LIMITS = {
    'x': (0.0, 2.7),
    'y': (0.0, 1.3),
    'z': (-0.4, 0.0),
}


def _getch() -> str:
    """Read a single character (terminal must already be in cbreak mode)."""
    return sys.stdin.read(1)


def _kbhit() -> bool:
    """Return True if a keypress is waiting on stdin."""
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    return bool(dr)


class TeleopKeyboardNode(Node):
    """Publish joint position commands for the FarmBot gantry via keyboard."""

    def __init__(self) -> None:
        """Initialise teleop keyboard node, publishers and parameters."""
        super().__init__('teleop_keyboard')

        self.declare_parameter('step_size', _STEP_SIZE)
        self._step = self.get_parameter('step_size').value

        self._x_pub = self.create_publisher(
            Float64, '/x_axis_joint/position', 10)
        self._y_pub = self.create_publisher(
            Float64, '/y_axis_joint/position', 10)
        self._z_pub = self.create_publisher(
            Float64, '/z_axis_joint/position', 10)

        self._positions = {'x': 0.0, 'y': 0.0, 'z': 0.0}

        self.get_logger().info(
            f'Teleop keyboard ready. Step size: {self._step:.3f}m'
        )

    def _publish_all(self) -> None:
        for axis, pub in (
            ('x', self._x_pub),
            ('y', self._y_pub),
            ('z', self._z_pub),
        ):
            msg = Float64()
            msg.data = self._positions[axis]
            pub.publish(msg)

    def _print_status(self) -> None:
        p = self._positions
        sys.stdout.write(
            f'\rX: {p["x"]:+.3f}  Y: {p["y"]:+.3f}  Z: {p["z"]:+.3f}  '
            f'[W/S X] [A/D Y] [Q/E Z] [R:home] [ESC:quit]   '
        )
        sys.stdout.flush()

    def run(self) -> None:
        """Blocking keyboard loop. Runs until ESC is pressed."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        # Enter cbreak mode once for the whole session: switching per-keypress
        # (the previous approach) uses TCSAFLUSH, which discards the very
        # keystroke that woke _kbhit(), so single presses never registered.
        tty.setcbreak(fd)
        try:
            self._print_status()
            self._publish_all()

            while rclpy.ok():
                if not _kbhit():
                    rclpy.spin_once(self, timeout_sec=0.05)
                    continue

                ch = _getch()

                if ch == '\x1b':  # ESC
                    self.get_logger().info('Quitting.')
                    break
                elif ch in ('w', 'W'):
                    self._positions['x'] += self._step
                elif ch in ('s', 'S'):
                    self._positions['x'] -= self._step
                elif ch in ('a', 'A'):
                    self._positions['y'] -= self._step
                elif ch in ('d', 'D'):
                    self._positions['y'] += self._step
                elif ch in ('q', 'Q'):
                    self._positions['z'] += self._step
                elif ch in ('e', 'E'):
                    self._positions['z'] -= self._step
                elif ch in ('r', 'R'):
                    self._positions = {'x': 0.0, 'y': 0.0, 'z': 0.0}

                # Clamp to joint limits
                for axis, (lo, hi) in _LIMITS.items():
                    self._positions[axis] = max(
                        lo, min(hi, self._positions[axis]))

                self._print_status()
                self._publish_all()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main(args=None) -> None:
    """Entry point: spin the teleop keyboard node."""
    rclpy.init(args=args)
    node = TeleopKeyboardNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
