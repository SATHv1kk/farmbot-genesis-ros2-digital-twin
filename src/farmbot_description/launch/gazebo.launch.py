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

"""Launch Gazebo Classic physics simulation + entity spawn + RViz2."""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Build the Gazebo simulation launch description."""
    pkg_share = get_package_share_directory('farmbot_description')
    urdf_file = os.path.join(pkg_share, 'farmbot_genesis.urdf')
    world_file = os.path.join(pkg_share, 'worlds', 'farmbot_empty.world')
    rviz_config = os.path.join(pkg_share, 'config', 'farmbot.rviz')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Path to the Gazebo world file'
    )

    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose', LaunchConfiguration('world'),
            '-s', 'libgazebo_ros_factory.so',
        ],
        output='screen'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': use_sim_time,
        }]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'farmbot_genesis',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
        ],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # farmbot.rviz uses "world" as the Fixed Frame; the URDF's own root is
    # base_link, so without this RViz reports "Fixed Frame [world] does
    # not exist" and nothing renders.
    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'world', '--child-frame-id', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        world_arg,
        gazebo,
        static_transform_publisher_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_entity,
        rviz_node,
    ])
