# Copyright (c) 2020 Samsung Research Russia
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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import HasNodeParams, RewrittenYaml


def generate_launch_description():
    # Input parameters declaration
    namespace = LaunchConfiguration('namespace')
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    use_composition = LaunchConfiguration('use_composition')
    log_level = LaunchConfiguration('log_level')
    model = LaunchConfiguration('model')

    # Variables
    lifecycle_nodes = ['map_saver']

    # Getting directories and launch-files
    bringup_dir = get_package_share_directory('car_navigation')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    slam_launch_file = os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')

    slam_lifecycle_nodes = ['map_saver']


    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=slam_params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)
    
    # If the provided param file doesn't have slam_toolbox params, we must remove the 'params_file'
    # LaunchConfiguration, or it will be passed automatically to slam_toolbox and will not load
    # the default file
    has_slam_toolbox_params = HasNodeParams(source_file=slam_params_file,
                                            node_name='slam_toolbox')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(bringup_dir, 'params', 'slam_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', 
        default_value='True',
        description='Automatically startup the nav2 stack')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', 
        default_value='True',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value = 'False',
        description='Whether to use composition or not')
    
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', 
        default_value='info',
        description='log level')
    
    node_group_cmd = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            Node(
                package='nav2_map_server',
                executable='map_saver_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[configured_params]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_slam',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'autostart': autostart},
                            {'node_names': slam_lifecycle_nodes}])
        ])
    
    # Slam Toolbox Conditional Launch Files Based on if Params are present or not
    start_slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=UnlessCondition(has_slam_toolbox_params))

    start_slam_toolbox_cmd_with_params = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={'use_sim_time': use_sim_time,
                          'slam_params_file': slam_params_file}.items(),
        condition=IfCondition(has_slam_toolbox_params))

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_use_composition_cmd)
    
    # Running Map Saver Server
    ld.add_action(node_group_cmd)

    # Running SLAM Toolbox (Only one of them will be run)
    ld.add_action(start_slam_toolbox_cmd)
    ld.add_action(start_slam_toolbox_cmd_with_params)

    return ld