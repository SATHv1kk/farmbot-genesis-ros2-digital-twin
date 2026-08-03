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

"""Full move_group + RViz planning launch graph for the MoveIt demo."""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node

import yaml


def load_yaml(path):
    """Return the parsed contents of a YAML file."""
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    """Build the MoveIt planning-and-execution launch description."""
    description_pkg = get_package_share_directory('farmbot_description')
    moveit_config_pkg = get_package_share_directory(
        'farmbot_genesis_moveit_config')
    config_dir = os.path.join(moveit_config_pkg, 'config')

    urdf_path = os.path.join(description_pkg, 'farmbot_genesis.urdf')
    srdf_path = os.path.join(config_dir, 'farmbot_genesis.srdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()
    with open(srdf_path, 'r') as f:
        robot_description_semantic = f.read()

    robot_description_param = {
        'robot_description': robot_description,
        'robot_description_semantic': robot_description_semantic,
    }

    # MoveIt expects each config file under a specific parameter namespace.
    kinematics_params = {
        'robot_description_kinematics':
            load_yaml(os.path.join(config_dir, 'kinematics.yaml')),
    }
    joint_limits_params = {
        'robot_description_planning':
            load_yaml(os.path.join(config_dir, 'joint_limits.yaml')),
    }

    ompl_pipeline = load_yaml(
        os.path.join(config_dir, 'ompl_planning.yaml'))
    ompl_pipeline['planning_plugin'] = 'ompl_interface/OMPLPlanner'
    ompl_pipeline['request_adapters'] = (
        'default_planner_request_adapters/AddTimeOptimalParameterization '
        'default_planner_request_adapters/FixWorkspaceBounds '
        'default_planner_request_adapters/FixStartStateBounds '
        'default_planner_request_adapters/FixStartStateCollision '
        'default_planner_request_adapters/FixStartStatePathConstraints'
    )
    ompl_pipeline['start_state_max_bounds_error'] = 0.1

    pilz_yaml = load_yaml(
        os.path.join(config_dir, 'pilz_industrial_motion_planner.yaml'))
    pilz_pipeline = pilz_yaml['pilz_industrial_motion_planner']
    pilz_pipeline['planning_plugin'] = (
        'pilz_industrial_motion_planner/CommandPlanner')

    planning_params = {
        'planning_pipelines': ['ompl', 'pilz_industrial_motion_planner'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl_pipeline,
        'pilz_industrial_motion_planner': pilz_pipeline,
    }

    controller_manager = (
        'moveit_simple_controller_manager/MoveItSimpleControllerManager')
    trajectory_execution = {
        'allowed_execution_duration_scaling': 1.2,
        'allowed_goal_duration_margin': 0.5,
        'allowed_start_tolerance': 0.01,
    }
    controllers_params = {
        'moveit_controller_manager': controller_manager,
        'moveit_simple_controller_manager':
            load_yaml(os.path.join(config_dir, 'controllers.yaml')),
        'trajectory_execution': trajectory_execution,
    }

    scene_monitor_params = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'publish_robot_description': True,
        'publish_robot_description_semantic': True,
    }

    # The SRDF virtual joint attaches base_link to the "world" frame,
    # so something has to publish that transform.
    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'world', '--child-frame-id', 'base_link'],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description_param,
            kinematics_params,
            joint_limits_params,
            planning_params,
            controllers_params,
            scene_monitor_params,
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[
            robot_description_param,
            kinematics_params,
            joint_limits_params,
        ],
    )

    return LaunchDescription([
        static_transform_publisher_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
        move_group_node,
        rviz_node,
    ])
