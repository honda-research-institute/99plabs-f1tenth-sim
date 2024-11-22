#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
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

import csv
from geometry_msgs.msg import PoseStamped
from f1_task_commander.f1_robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
import yaml

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self.navigator = BasicNavigator()
        self.waypoints = []
        
        # Setup Waypoint Follower Configurations
        self.config_setup()
        
        
        # Execute navigation
        self.execute_navigation()

    def config_setup(self):
        """
        Set up the configuration for the WaypointNavigator.

        This function declares the necessary parameters for the initial pose position and file path.
        It then retrieves the values of these parameters and verifies their types.
        The initial pose position is stored as a tuple of (x, y) coordinates, and the orientation is stored as a tuple of (z, w) values.
        The waypoints file and config file paths are also stored as attributes.

        Returns:
            None: If there is an error in the parameter types.

        """
        try:
            # Declare parameters for initial pose position and file path
            #self.declare_parameter('initial_pose_x', rclpy.Parameter.Type.DOUBLE)
            #self.declare_parameter('initial_pose_y', rclpy.Parameter.Type.DOUBLE)
            #self.declare_parameter('initial_pose_orientation_z', rclpy.Parameter.Type.DOUBLE)
            #self.declare_parameter('initial_pose_orientation_w', rclpy.Parameter.Type.DOUBLE)
            self.declare_parameter('waypoints_file', rclpy.Parameter.Type.STRING)
            self.declare_parameter('params_file', rclpy.Parameter.Type.STRING)
            # Get parameters and verify types
            self.params_file = self.get_parameter('params_file').value
            params_file =yaml.safe_load(open(self.params_file))
            self.params = params_file['task_commander']['ros__parameters']
            self.localizer = params_file['task_commander']['ros__parameters']['localizer']
            self.waypoints_file = self.get_parameter('waypoints_file').value

            self.initial_pose_x = float(self.params['initial_pose_x'])
            self.initial_pose_y = float(self.params['initial_pose_y'])
            self.initial_pose_orientation_z = float(self.params['initial_pose_orientation_z'])
            self.initial_pose_orientation_w = float(self.params['initial_pose_orientation_w'])

            # Print initialized parameters
            initialized_parameters = {
                #'initial_pose_x': self.initial_pose_x,
                #'initial_pose_y': self.initial_pose_y,
                #'initial_pose_orientation_z': self.initial_pose_orientation_z,
                #'initial_pose_orientation_w': self.initial_pose_orientation_w,
                'localizer': self.localizer,
                'waypoints_file': self.waypoints_file,
                'params_file': self.params_file,
                'params': self.params
            }
            self.get_logger().info(f"Initialized parameters: {initialized_parameters}")

        except (ValueError, TypeError):
            self.get_logger().error("Error: Invalid parameter type. Check the types of initial_pose_x, initial_pose_y, initial_pose_orientation_z, and initial_pose_orientation_w.")
            return

    def read_waypoints_from_csv(self):
        """
        Parses a CSV file containing waypoints data and populates the self.waypoints list with PoseStamped objects.
        """
        # Parse CSV File
        with open(self.waypoints_file, 'r') as file:
            reader = csv.reader(file, delimiter=',')
            next(reader, None)  # skip the headers
            # Go through each row in csv file and parse poses to list
            for row in reader:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = float(row[0])
                pose.pose.position.y = float(row[1])
                pose.pose.orientation.z = float(row[2])
                pose.pose.orientation.w = float(row[3])
                self.waypoints.append(pose)
    
    def setInitialPose(self):
        """
        Set Initial Configured Pose
        """
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.position.x = self.initial_pose_x
        initial_pose.pose.position.y = self.initial_pose_y
        initial_pose.pose.orientation.z = self.initial_pose_orientation_z
        initial_pose.pose.orientation.w = self.initial_pose_orientation_w
        self.navigator.setInitialPose(initial_pose)

    def execute_navigation(self):
        """
        Executes the navigation by setting the initial pose, reading waypoints from a CSV file,
        waiting for navigation to fully activate, executing the navigation, and monitoring the
        feedback and result of the navigation task.
        """
        # Set initial pose
        self.setInitialPose()

        # Read waypoints from CSV file
        self.read_waypoints_from_csv()
        goal_poses = self.waypoints
        self.get_logger().info(f"Number of Waypoints: {len(goal_poses)}")

        # Wait for navigation to fully activate, localizer is configurable
        self.navigator.waitUntilNav2Active(localizer=self.localizer)
        
        # Execute navigation
        nav_start = self.get_clock().now()
        self.navigator.followWaypoints(goal_poses)

        i = 0
        while not self.navigator.isTaskComplete():
            # Provide Navigation progress feedback or anything else you need to monitor
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Executing current waypoint: ' +
                      str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
                now = self.get_clock().now()

                # navigation timeout
                if now - nav_start > Duration(seconds=600.0):
                    self.navigator.cancelTask()


        # Return Feedback Based on navigator status
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

        self.navigator.lifecycleShutdown()

def main(args=None):
    rclpy.init(args=args)
    waypoint_navigator = WaypointNavigator()
    rclpy.spin(waypoint_navigator)

if __name__ == '__main__':
    main()
