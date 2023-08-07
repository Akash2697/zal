from typing import List
import rclpy
from rclpy.node import Node
from rclpy.context import Context
from rclpy.parameter import Parameter
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32
import time

class Robot(Node):

    def __init__(self, node_name: str = 'robot_node', *, context: Context = None, cli_args: List[str] = None, namespace: str = None, use_global_arguments: bool = True, enable_rosout: bool = True, start_parameter_services: bool = True, parameter_overrides: List[Parameter] = None, allow_undeclared_parameters: bool = False, automatically_declare_parameters_from_overrides: bool = False) -> None:
        super().__init__(node_name, context=context, cli_args=cli_args, namespace=namespace, use_global_arguments=use_global_arguments, enable_rosout=enable_rosout, start_parameter_services=start_parameter_services, parameter_overrides=parameter_overrides, allow_undeclared_parameters=allow_undeclared_parameters, automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides)
        
        #Message types:
        self.pose_msg = Odometry()
        self.goal_pose_msg = Float64MultiArray()
        self.angle_difference_msg = Float32()
        self.distance_difference_msg = Float32()

        #Topic names:
        self.pose_topic = 'odom'
        self.goal_pose_topic = 'goal_pose'
        self.angle_difference_topic = 'angle_diff'
        self.distance_difference_topic = 'distance_diff'

        #Publishers and Subscribers:
        self.pose_subscriber = self.create_subscription(Odometry, self.pose_topic,self.pose_subscription_callback,10)
        self.goal_pose_subscriber = self.create_subscription(Float64MultiArray, self.goal_pose_topic,self.goal_pose_subscription_callback,10)

        if self.pose_subscriber:
            self.get_logger().info('Pose subscriber created successfully.')
        else:
            self.get_logger().error('Pose subscriber creation failed.')

        if self.goal_pose_subscriber:
            self.get_logger().info('Goal pose subscriber created successfully.')
        else:
            self.get_logger().error('Goal pose subscriber creation failed.')
            
        self.angle_difference_publisher = self.create_publisher(Float32,self.angle_difference_topic,10)
        self.distance_difference_publisher = self.create_publisher(Float32,self.distance_difference_topic,10)

        #Timers:
        timer_period_angle_diff = 0.1  # seconds
        self.timer_pub_angle_diff = self.create_timer(timer_period_angle_diff, self.angle_difference_publisher_callback)
        timer_period_distance_diff = 0.1  # seconds
        self.timer_pub_distance_diff = self.create_timer(timer_period_distance_diff, self.distance_difference_publisher_callback)

        self.goal_position_x = 0.0
        self.goal_position_y = 0.0
        self.angle_difference = 0.0
        self.distance_difference = 0.0


    def pose_subscription_callback(self, msg):
        roll, pitch, yaw = self.euler_from_quaternion(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)

        obs_state_vector_x_y_yaw = [msg.pose.pose.position.x,msg.pose.pose.position.y,yaw]
        desired_angle_goal = math.atan2((self.goal_position_y-obs_state_vector_x_y_yaw[1]),(self.goal_position_x - obs_state_vector_x_y_yaw[0]))

        self.get_logger().info('X: "%s"' % msg.pose.pose.position.x)
        self.get_logger().info('Y: "%s"' % msg.pose.pose.position.y)
        self.get_logger().info('Yaw: "%s"' % yaw)


        self.angle_difference = (180/math.pi) * (desired_angle_goal - yaw)
        self.get_logger().info('Angle Difference: "%s"' % self.angle_difference)

        self.distance_difference = abs(math.sqrt(((self.goal_position_x - obs_state_vector_x_y_yaw[0]) ** 2) + ((self.goal_position_y - obs_state_vector_x_y_yaw[1]) ** 2)))
        self.get_logger().info('Goal Distance: "%s"' % self.distance_difference)


    def goal_pose_subscription_callback(self,msg):
        self.goal_position = msg.data
        self.goal_position_x = self.goal_position[0]
        self.goal_position_y = self.goal_position[1]
        self.get_logger().info('Goal Pose: "%s"' % self.goal_position)


    def angle_difference_publisher_callback(self):
        self.angle_difference_msg.data = self.angle_difference
        self.angle_difference_publisher.publish(self.angle_difference_msg)

    def distance_difference_publisher_callback(self):
        self.distance_difference_msg.data = self.distance_difference
        self.distance_difference_publisher.publish(self.distance_difference_msg)

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
    
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
    
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
    
        return roll_x, pitch_y, yaw_z # in radians
    
def main():
    
    rclpy.init()
    robot_node = Robot()
    rclpy.spin(robot_node)
    robot_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()