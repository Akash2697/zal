#!/usr/bin/env python3

from typing import List
import rclpy
from rclpy.node import Node
from rclpy.context import Context
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist

import rclpy
from rclpy.action import ActionServer
from custom_msgs.action import Task
from geometry_msgs.msg import Point
import math
import sys
import argparse
from std_msgs.msg import Float32
from std_msgs.msg import Float64MultiArray
import time
from rclpy.parameter import ParameterValue
from nav_msgs.msg import Odometry
import threading
from rclpy import executors

class TaskPubSub(Node):
    def __init__(self, node_name: str ='task_pub_sub', *, context: Context = None, cli_args: List[str] = None, namespace: str = None, use_global_arguments: bool = True, enable_rosout: bool = True, start_parameter_services: bool = True, parameter_overrides: List[Parameter] = None, allow_undeclared_parameters: bool = False, automatically_declare_parameters_from_overrides: bool = False) -> None:
        super().__init__(node_name, context=context, cli_args=cli_args, namespace=namespace, use_global_arguments=use_global_arguments, enable_rosout=enable_rosout, start_parameter_services=start_parameter_services, parameter_overrides=parameter_overrides, allow_undeclared_parameters=allow_undeclared_parameters, automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides)
        self.velocity_topic = 'cmd_vel'
        self.pose_topic = 'odom'
        
        self.velocity_publisher = self.create_publisher(Twist,self.velocity_topic,10)
        self.pose_subscriber = self.create_subscription(Odometry, self.pose_topic,self.pose_subscription_callback,10)

        if self.pose_subscriber:
            self.get_logger().info('Pose subscriber created successfully.')
        else:
            self.get_logger().error('Pose subscriber creation failed.')

    def pose_subscription_callback(self, msg):
        roll, pitch, yaw = self.euler_from_quaternion(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)

        self.obs_state_vector_x_y_yaw = [msg.pose.pose.position.x,msg.pose.pose.position.y,yaw]
        global curr_yaw
        curr_yaw = yaw
        global curr_x 
        curr_x = msg.pose.pose.position.x
        global curr_y
        curr_y = msg.pose.pose.position.y
        self.get_logger().info('X: "%s"' % msg.pose.pose.position.x)
        self.get_logger().info('Y: "%s"' % msg.pose.pose.position.y)
        self.get_logger().info('Yaw: "%s"' % yaw)

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


class TaskActionServer(Node):
    def __init__(self, velocity_publisher, pose_subscriber):
        super().__init__('action_server')
        self._action_server = ActionServer(
            node = self,
            action_type=Task,
            action_name='task',
            execute_callback = self.execute_callback
        )
        self.velocity_publisher = velocity_publisher
        self.pose_subscriber = pose_subscriber
        self.velocity_msg = Twist()

        self.not_aligned = True
        self.not_reached = True

        self.velocity_topic = 'cmd_vel'
        self.pose_topic = 'odom'
        self.feedback = Task.Feedback()
        
    
    def execute_callback(self, goal_handle):

        self.goal_handle = goal_handle
        goal = self.goal_handle.request
        target_position = goal.goal_pose
        target_velocity = goal.velocity_vector

        self.goal_position_x = target_position[0]
        self.goal_position_y = target_position[1]

        self.move_robot_to(self.goal_position_x,self.goal_position_y,target_velocity)
        
        self.goal_handle.succeed()
        return Task.Result()


    def move_robot_to(self, target_x, target_y, target_velocity):

        self.desired_angle_goal = math.atan2((target_y - curr_y),(target_x - curr_x))

        while(self.not_reached):
            self.angle_difference = self.desired_angle_goal - curr_yaw
            self.angle_difference_degree = (180/math.pi) * self.angle_difference

            self.distance_difference = abs(math.sqrt(((target_x - curr_x) ** 2) + ((target_y - curr_y) ** 2)))

            if (self.angle_difference > 0.1) and self.not_aligned:
                self.velocity_msg.angular.z = target_velocity[5] 

            elif (self.angle_difference < -0.1) and self.not_aligned:
                self.velocity_msg.angular.z = -target_velocity[5] 

            elif (self.angle_difference < 0.1) and (self.angle_difference > -0.1) and self.not_aligned:
                self.velocity_msg.angular.z = 0.0

            if (self.distance_difference > 0.1):
                self.velocity_msg.linear.x = target_velocity[0]
                
                if (self.distance_difference < 1.0):
                    self.velocity_msg.linear.x = 0.0
                    self.velocity_msg.angular.z = 0.0
                    self.not_aligned = False
                    self.not_reached=False
            
            
            self.feedback.distance_remaining = self.distance_difference
            self.goal_handle.publish_feedback(self.feedback)
            self.velocity_publisher.publish(self.velocity_msg)

        return True



        """ while(self.not_aligned):
            self.desired_angle_goal = math.atan2((target_y-curr_y),(target_x - curr_x))

            self.angle_difference = (180/math.pi) * (self.desired_angle_goal - curr_yaw)
            print("Angle Difference:", self.angle_difference)

            if (self.angle_difference > 5.0):
                self.velocity_msg.angular.z = target_velocity[5]
                self.velocity_publisher.publish(self.velocity_msg)

            if (self.angle_difference < -5.0):
                self.velocity_msg.angular.z = -target_velocity[5]
                self.velocity_publisher.publish(self.velocity_msg)

            if (self.angle_difference < 5.0):
                self.velocity_msg.angular.z = 0.0
                self.velocity_publisher.publish(self.velocity_msg)
                self.not_aligned = False
            

        while(self.not_reached):

            self.distance_difference = abs(math.sqrt(((target_x - self.obs_state_vector_x_y_yaw[0]) ** 2) + ((target_y - self.obs_state_vector_x_y_yaw[1]) ** 2)))
            print("Distance Difference:", self.distance_difference)
    
            if (self.distance_difference > 0.3):
                self.velocity_msg.linear.x = target_velocity[0]
                self.velocity_publisher.publish(self.velocity_msg)

            if (self.distance_difference < 0.3):
                self.velocity_msg.linear.x = 0.0
                self.velocity_publisher.publish(self.velocity_msg)
                self.not_reached = False """
        
        """ self.desired_angle_goal = math.atan2((target_y - curr_y),(target_x - curr_x))

        self.angle_difference = self.desired_angle_goal - curr_yaw
        self.angle_difference_degree = (180/math.pi) * self.angle_difference
        print("Angle Difference:", self.angle_difference)
        time_angle = self.angle_difference / target_velocity[5]

        if (self.angle_difference_degree > 1.0):
            self.velocity_msg.angular.z = target_velocity[5]
            self.velocity_publisher.publish(self.velocity_msg)
            time.sleep(time_angle)
            self.velocity_msg.angular.z = 0.0
            self.velocity_publisher.publish(self.velocity_msg)

        if (self.angle_difference_degree < -1.0):
            self.velocity_msg.angular.z = -target_velocity[5]
            self.velocity_publisher.publish(self.velocity_msg)
            time.sleep(time_angle)
            self.velocity_msg.angular.z = 0.0
            self.velocity_publisher.publish(self.velocity_msg)


        self.distance_difference = abs(math.sqrt(((target_x - curr_x) ** 2) + ((target_y - curr_y) ** 2)))
        
        time_distance = self.distance_difference / target_velocity[0]

        if (self.distance_difference > 0.3):
            self.velocity_msg.linear.x = target_velocity[0]
            self.velocity_publisher.publish(self.velocity_msg)
            time.sleep(time_distance)
            self.velocity_msg.linear.x = 0.0
            self.velocity_publisher.publish(self.velocity_msg)

            return True """
    
    

    

def main(args=None):
    rclpy.init(args=args)
    pub_sub_node = TaskPubSub()
    action_server = TaskActionServer(pub_sub_node.velocity_publisher,pub_sub_node.pose_subscriber)
    executor = executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(action_server)
    executor.add_node(pub_sub_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    thread.join()
    action_server.destroy_node()
    pub_sub_node.destroy_node()
    executor.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
