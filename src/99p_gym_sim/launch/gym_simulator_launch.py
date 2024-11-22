# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
import yaml

def generate_launch_description():
    # Config Declarations
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    model = LaunchConfiguration('model')
    
    # Launch configuration variables specific to simulation
    nav_params = LaunchConfiguration('nav_params')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    map_yaml_file = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('use_rviz')
    
    use_amcl = LaunchConfiguration('use_amcl')
    use_task_cmd = LaunchConfiguration('use_task_cmd')
    use_nav = LaunchConfiguration('use_nav')
    
    # Package Directories
    main_pkg_dir = get_package_share_directory('99p_gym_sim')
    nav_pkg_dir = get_package_share_directory('car_navigation')
    task_cmd_pkg_dir = get_package_share_directory('f1_task_commander')
    ac_pkg_dir = get_package_share_directory('autonomous_control')

    # model description directory
    model_dir = get_package_share_directory('f110_description')
    default_model_path = os.path.join(model_dir, 'urdf', 'racecar.xacro')
    
    # RVIZ
    rviz_config_path = os.path.join(main_pkg_dir, 'rviz', 'nav2_view.rviz')

    # Simulator config
    sim_config = os.path.join(main_pkg_dir,'config','sim.yaml')
    
    # Navigation config
    nav_params_path = os.path.join(main_pkg_dir, 'params', 'nav2_params.yaml')
    slam_params_path = os.path.join(main_pkg_dir, 'params', 'slam_params.yaml')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    
    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
        )
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(main_pkg_dir, 'maps', 'tape_map_no_lanes.yaml'),
        description='Full path to map yaml file to load')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='True',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')
    
    declare_model_cmd = DeclareLaunchArgument(
        'model',
        default_value= default_model_path,
        description='model of the robot')
    
    declare_nav_params = DeclareLaunchArgument(
        name='nav_params',
        default_value=nav_params_path,
        description='Nav2 parameters'
        )
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
         'rviz_config_file',
         default_value=rviz_config_path,
         description='Full path to the RVIZ config file to use.')
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
         'use_rviz',
         default_value='True',
         description='Whether to start RVIZ')
    
    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether to start RVIZ')
    
    declare_slam_params_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=slam_params_path,
        description='Slam Parameters File')
    
    declare_use_amcl_cmd = DeclareLaunchArgument(
        'use_amcl', default_value='True',
        description='Whether to use AMCL or slam localization')
    
    declare_use_nav_cmd = DeclareLaunchArgument(
        'use_nav', default_value='True',
        description='Whether to use the Navigation Stack')
    
    declare_use_task_commander_cmd = DeclareLaunchArgument(
        'use_task_cmd', default_value='False',
        description='Whether to use the Automated Task Commander')
    
    # Node Commands
    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),
        # Simulator Launch
        Node(
            package='99p_gym_sim',
            executable='gym_simulator',
            name='gym_simulator',
            parameters=[sim_config]
        ),
        # URDF state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['xacro ', model])}],
            output='screen',
            remappings=remappings
            ),
        # RVIZ
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_pkg_dir, 'launch','rviz_launch.py')),
            condition=IfCondition(use_rviz),
            launch_arguments={'namespace': namespace,
                            'use_sim_time': use_sim_time,
                            'use_namespace': use_namespace,
                            'rviz_config': rviz_config_file}.items()),
        # Autonomous Control
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ac_pkg_dir, 'launch','bringup_launch.py'))),
        # Navigation
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg_dir, 'launch', 'bringup_launch.py')),
        condition=IfCondition(use_nav),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'slam': slam,
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'params_file': nav_params,
                          'slam_params': slam_params_file,
                          'autostart': autostart,
                          'use_composition': use_composition,
                          'use_respawn': use_respawn,
                          'log_level': log_level,
                          'use_amcl': use_amcl}.items()),
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(task_cmd_pkg_dir, 'launch', 'nav_through_poses_launch.py')),
        condition=IfCondition(use_task_cmd),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'autostart': autostart,
                          'use_composition': use_composition,
                          'use_respawn': use_respawn,
                          'log_level': log_level}.items())
        ])
        
    
    ld = LaunchDescription()
    # Add Config and Params Declarations
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_nav_params)
    ld.add_action(declare_slam_params_cmd)
    ld.add_action(declare_model_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_amcl_cmd)
    ld.add_action(declare_use_nav_cmd)
    ld.add_action(declare_use_task_commander_cmd)
    
    # Launch Gym Simulator
    ld.add_action(bringup_cmd_group)
    return ld