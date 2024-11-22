# Copyright (c) 2018 Intel Corporation
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
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('car_navigation')

    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    slam_localization_params_file = LaunchConfiguration('slam_localization_params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    use_amcl = LaunchConfiguration('use_amcl')
    log_level = LaunchConfiguration('log_level')

    amcl_lifecycle_nodes = ['map_server', 'amcl']
    scan_match_lifecycle_nodes=['map_server']
    
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'maps', 'tape_map_no_lanes.yaml'),
        description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
    declare_slam_localization_params_file_cmd = DeclareLaunchArgument(
        'slam_localization_params_file',
        default_value=os.path.join(bringup_dir, 'params', 'slam_localization_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', 
        default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', 
        default_value='False',
        description='Use composed bringup if True')

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', 
        default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', 
        default_value='True',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')
    
    declare_use_amcl_cmd = DeclareLaunchArgument(
        'use_amcl', 
        default_value='False',
        description='Whether to use AMCL (nav2_amcl) or scan matching (slam_toolbox) for localization')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', 
        default_value='info',
        description='log level')
    
    # Non Composable Launch Conditions
    amcl_condition = IfCondition(PythonExpression(['not ', use_composition, ' and ', use_amcl]))
    scan_match_condition = IfCondition(PythonExpression(['not ', use_composition, ' and', ' not ', use_amcl]))
    
    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                condition=amcl_condition,
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            # AMCL Lifecycle Manager
            Node(
                condition=amcl_condition,
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': amcl_lifecycle_nodes}]),
            Node(
                condition=scan_match_condition,
                package='slam_toolbox',
                executable='localization_slam_toolbox_node',
                name='slam_toolbox',
                parameters=[slam_localization_params_file],
                arguments=['--ros-args', '--log-level', log_level],
                output='screen'),
            # Scan Match Lifecycle Manager
            Node(
                condition=scan_match_condition,
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': scan_match_lifecycle_nodes}]),
        ]
    )
    
    amcl_load_nodes = GroupAction(
        condition=amcl_condition,
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': amcl_lifecycle_nodes}])
        ]
    )

    scan_match_load_nodes = GroupAction(
        condition=scan_match_condition,
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                condition=scan_match_condition,
                package='slam_toolbox',
                executable='localization_slam_toolbox_node',
                name='slam_toolbox',
                parameters=[slam_localization_params_file,{'use_sim_time': use_sim_time}],
                arguments=['--ros-args', '--log-level', log_level],
                output='screen'),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': scan_match_lifecycle_nodes}])
            
        ]
    )

    # Conditions for loading composable nodes for AMCL and SLAM
    amcl_compose_condition = IfCondition(PythonExpression([use_composition, ' and ', use_amcl]))
    scan_match_compose_condition = IfCondition(PythonExpression([use_composition, ' and', ' not ', use_amcl]))
    
    # AMCL composable nodes
    load_composable_amcl_nodes = LoadComposableNodes(
        condition=amcl_compose_condition,
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='map_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_amcl',
                plugin='nav2_amcl::AmclNode',
                name='amcl',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_localization',
                parameters=[{'use_sim_time': use_sim_time,
                             'autostart': autostart,
                             'node_names': amcl_lifecycle_nodes}])]
    )

    # scan matching composable nodes
    load_composable_scan_match_nodes = LoadComposableNodes(
        condition=scan_match_compose_condition,
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='map_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='slam_toolbox',
                plugin='slam_toolbox::LocalizationSlamToolbox',
                name='slam_toolbox',
                parameters=[slam_localization_params_file, {'use_sim_time': use_sim_time}],
                extra_arguments=[{'ros_arguments': ['--ros-args', '--log-level', log_level]}],
                remappings=remappings),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_localization',
                parameters=[{'use_sim_time': use_sim_time,
                             'autostart': autostart,
                             'node_names': scan_match_lifecycle_nodes}])
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_slam_localization_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_use_amcl_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add the actions to launch all of the localization nodes
    
    # Non Composable Nodes
    #ld.add_action(load_nodes)
    ld.add_action(amcl_load_nodes)
    ld.add_action(scan_match_load_nodes)
    
    # Composable
    ld.add_action(load_composable_amcl_nodes)
    ld.add_action(load_composable_scan_match_nodes)

    return ld