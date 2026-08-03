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
Joint state publisher node for FarmBot Genesis.

Publishes all 3 gantry joint states on /joint_states as a fallback when
no hardware controller is running.  Joints start at home position
(0, 0, 0) and are updated live from the per-axis position topics
published by teleop_keyboard.py:
  /x_axis_joint/position
  /y_axis_joint/position
  /z_axis_joint/position
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

from std_msgs.msg import Float64


_JOINT_NAMES = ['x_axis_joint', 'y_axis_joint', 'z_axis_joint']
_COMMAND_TOPICS = [
    '/x_axis_joint/position',
    '/y_axis_joint/position',
    '/z_axis_joint/position',
]
_PUBLISH_RATE = 50.0  # Hz


class JointStatePublisherNode(Node):
    """
    Publishes live joint states for the gantry.

    This node is intended as a lightweight fallback so that the URDF
    can be displayed and TF frames published even when no real
    controller is connected. It subscribes to the same per-axis
    position topics teleop_keyboard.py publishes to, so keyboard
    teleop moves the model in RViz/Gazebo without ros2_control.
    """

    def __init__(self) -> None:
        """Initialise publisher, subscribers, timer and joint state msg."""
        super().__init__('farmbot_joint_state_publisher')

        self._pub = self.create_publisher(JointState, '/joint_states', 10)

        self._msg = JointState()
        self._msg.name = list(_JOINT_NAMES)
        self._msg.position = [0.0, 0.0, 0.0]
        self._msg.velocity = [0.0, 0.0, 0.0]
        self._msg.effort = [0.0, 0.0, 0.0]

        for i, topic in enumerate(_COMMAND_TOPICS):
            self.create_subscription(
                Float64, topic,
                self._make_position_cb(i), 10)

        period = 1.0 / _PUBLISH_RATE
        self._timer = self.create_timer(period, self._publish)

        self.get_logger().info(
            f'Joint state publisher running at {_PUBLISH_RATE:.0f} Hz '
            f'for joints: {_JOINT_NAMES}'
        )

    def _make_position_cb(self, index: int):
        def _cb(msg: Float64) -> None:
            self._msg.position[index] = msg.data
        return _cb

    def _publish(self) -> None:
        self._msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._msg)


def main(args=None) -> None:
    """Entry point: spin the joint state publisher node."""
    rclpy.init(args=args)
    node = JointStatePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
