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
Persistent motion logger.

Logs ALL joint states + TF to a timestamped file.
"""
from datetime import datetime
import os

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

from tf2_msgs.msg import TFMessage

LOG_DIR = os.path.expanduser('~/farmbot_logs')


class MotionLogger(Node):
    """Subscribe to /joint_states and /tf and log every sample to disk."""

    def __init__(self):
        """Open a fresh timestamped log file and subscribe to topics."""
        super().__init__('motion_logger')
        os.makedirs(LOG_DIR, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.logfile = os.path.join(LOG_DIR, f'motion_{ts}.log')
        self.f = open(self.logfile, 'w')
        self.f.write(f'# FarmBot Motion Log — {datetime.now()}\n')
        self.f.write('# Columns: TYPE timestamp joint_name position\n')
        self.f.write('#   JS = joint_state   TF = transform\n')
        self.f.write('# Move X(0→2.7), Y(0→1.3), Z(-0.4→0) fully\n\n')
        self.f.flush()
        self.get_logger().info(f'Logging to {self.logfile}')

        self.js_sub = self.create_subscription(
            JointState, '/joint_states', self.js_cb, 10)
        self.tf_sub = self.create_subscription(
            TFMessage, '/tf', self.tf_cb, 10)

    def js_cb(self, msg):
        """Append one JS line per joint in the incoming message."""
        t = self.get_clock().now().nanoseconds / 1e9
        for i, name in enumerate(msg.name):
            pos = msg.position[i] if i < len(msg.position) else float('nan')
            self.f.write(f'JS {t:.4f} {name} {pos:.6f}\n')
        self.f.flush()

    def tf_cb(self, msg):
        """Append one TF line per transform in the incoming message."""
        t = self.get_clock().now().nanoseconds / 1e9
        for tr in msg.transforms:
            self.f.write(
                f'TF {t:.4f} {tr.header.frame_id}->{tr.child_frame_id} '
                f'x={tr.transform.translation.x:.4f} '
                f'y={tr.transform.translation.y:.4f} '
                f'z={tr.transform.translation.z:.4f}\n'
            )
        self.f.flush()

    def destroy_node(self):
        """Write the closing footer line and close the log file."""
        self.f.write(f'\n# End — {datetime.now()}\n')
        self.f.close()
        self.get_logger().info('Log saved.')
        super().destroy_node()


def main():
    """Entry point: spin the motion logger node."""
    rclpy.init()
    node = MotionLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
