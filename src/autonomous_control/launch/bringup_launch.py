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
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    config_file = LaunchConfiguration('config_file')
    
    # Config Files
    autonomous_control_config = os.path.join(
        get_package_share_directory('autonomous_control'),
        'config',
        'autonomous_control.yaml'
    )
    
    # Declarations
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')
    
    autonomous_control_la = DeclareLaunchArgument(
        'config_file',
        default_value=autonomous_control_config,
        description='Descriptions for twist to ackermann configs')
    
    # config declaration
    ld = LaunchDescription()
    
    # Nodes
    autonomous_control_node = Node(
        package='autonomous_control',
        executable='autonomous_control',
        name='autonomous_control',
        parameters=[config_file],
        namespace=namespace
    )
    
    # Compose the launch description
    ld.add_action(autonomous_control_la)
    ld.add_action(declare_namespace_cmd)
    # Node
    ld.add_action(autonomous_control_node)
    return ld