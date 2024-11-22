import rclpy
from rclpy.node import Node
from math import atan
from std_msgs.msg import Int32MultiArray, Bool
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

class AutonmousControl(Node):
    def __init__(self):
        super().__init__('autonomous_control')
        self.get_logger().info("Node Starting...")
        self.declare_parameters(
            namespace='',
            parameters=[
                ('ackermann_topic', rclpy.Parameter.Type.STRING),
                ('twist_topic', rclpy.Parameter.Type.STRING),
                ('wheelbase', rclpy.Parameter.Type.DOUBLE),
                ('frame_id', rclpy.Parameter.Type.STRING)
            ])
        # Topic Variables
        self.ackermann_topic = self.get_parameter('ackermann_topic').value
        self.twist_topic = self.get_parameter('twist_topic').value

        # Robot Params
        self.wheelbase = self.get_parameter('wheelbase').value
        self.frame_id = self.get_parameter('frame_id').value

        # Message Creation
        self.ackermann_msg = AckermannDriveStamped()
        self.system_deadman_msg = Bool()

        # Subscriber for Twist, Traffic Light
        self.subscription = self.create_subscription(Twist, self.twist_topic, self.cmd_callback, 1)
        
        # Publisher
        self.publisher = self.create_publisher(AckermannDriveStamped, self.ackermann_topic, 1)
        self.get_logger().info("Autonomous Driving Node Setup...")

    def convert_trans_rot_vel_to_steering_angle(self, v, omega):
        if omega == 0 or v == 0:
            return 0.0
        radius = v / omega
        steering_angle = float(atan(self.wheelbase / float(radius)))
        return steering_angle
    
    # Message Callbacks
    def create_drive_msg(self, data):
        v = data.linear.x
        steering_angle = self.convert_trans_rot_vel_to_steering_angle(v, data.angular.z)
        steering_angle = float(steering_angle)
        
        # Set message values
        self.ackermann_msg.header.stamp = self.get_clock().now().to_msg()
        self.ackermann_msg.header.frame_id = self.frame_id
        self.ackermann_msg.drive.steering_angle = steering_angle
        self.ackermann_msg.drive.speed = v

    def cmd_callback(self, data):
        self.create_drive_msg(data)
        # publish message
        self.publisher.publish(self.ackermann_msg)
        
def main(args=None):  
    rclpy.init(args=args)
    autonomous_control = AutonmousControl()
    rclpy.spin(autonomous_control)
    autonomous_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()


