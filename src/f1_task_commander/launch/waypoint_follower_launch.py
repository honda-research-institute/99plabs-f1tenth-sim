# Copyright (c) 2021 Samsung Research America
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    waypoints_file = LaunchConfiguration('waypoints_file')
    
    # Get the launch directory
    bringup_dir = get_package_share_directory('f1_task_commander')

    # File Path Declarations
    task_params_path = os.path.join(bringup_dir, 'params', 'task_commander_params.yaml')
    waypoints_file = os.path.join(bringup_dir, 'waypoints', 'waypoints.csv')

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='whether to use simulator simulation clock if true')
    
    declare_params_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=task_params_path,
        description='Default params File Path')

    # start the follow waypoints task
    nav_through_poses_cmd = Node(
        package='f1_task_commander',
        executable='waypoint_follower',
        name='waypoint_follower',
        parameters=[{'use_sim_time': use_sim_time, 'params_file':params_file, 'waypoints_file': waypoints_file}],
        output='screen')

    # Create Launch Description
    ld = LaunchDescription()

    # Add Launch Arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_cmd)

    # Nodes
    ld.add_action(nav_through_poses_cmd)
    
    return ld
