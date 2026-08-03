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
Launch RViz2 + robot_state_publisher + joint source + logger.

By default the joint source is the interactive joint_state_publisher_gui
slider panel. Pass `use_gui:=false` to launch the package's own
joint_state_publisher_node.py instead, which listens on the per-axis
position topics published by teleop_keyboard.py -- use that combination
when you want keyboard teleop to actually move the model.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Build the RViz visualisation launch description."""
    pkg_share = get_package_share_directory('farmbot_description')
    urdf_file = os.path.join(pkg_share, 'farmbot_genesis.urdf')
    rviz_config = os.path.join(pkg_share, 'config', 'farmbot.rviz')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description=(
            'true: joint_state_publisher_gui sliders. '
            'false: joint_state_publisher_node.py, driven by '
            'teleop_keyboard.py.'
        ),
    )
    use_gui = LaunchConfiguration('use_gui')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': False,
        }]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(use_gui),
    )

    joint_state_publisher_fallback_node = Node(
        package='farmbot_description',
        executable='joint_state_publisher_node.py',
        condition=UnlessCondition(use_gui),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
    )

    # This node creates the "world" frame
    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'world', '--child-frame-id', 'base_link']
    )

    # Motion logger (records joint states + TF to ~/farmbot_logs/)
    motion_logger_node = Node(
        package='farmbot_description',
        executable='motion_logger.py',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        use_gui_arg,
        static_transform_publisher_node,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        joint_state_publisher_fallback_node,
        rviz_node,
        motion_logger_node,
    ])
