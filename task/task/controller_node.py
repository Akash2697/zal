#!/usr/bin/env python3

from typing import List
import rclpy
from rclpy.node import Node
from rclpy.context import Context
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
import math
import sys
import argparse
from std_msgs.msg import Float32
from std_msgs.msg import Float64MultiArray
import time
from rclpy.parameter import ParameterValue


class Controller(Node):

    def __init__(self, node_name: str = 'controller_node', *, context: Context = None, cli_args: List[str] = None, namespace: str = None, use_global_arguments: bool = True, enable_rosout: bool = True, start_parameter_services: bool = True, parameter_overrides: List[Parameter] = None, allow_undeclared_parameters: bool = False, automatically_declare_parameters_from_overrides: bool = False) -> None:
        
        super().__init__(node_name, context=context, cli_args=cli_args, namespace=namespace, use_global_arguments=use_global_arguments, enable_rosout=enable_rosout, start_parameter_services=start_parameter_services, parameter_overrides=parameter_overrides, allow_undeclared_parameters=allow_undeclared_parameters, automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides)

        # Set the loaded parameters
        
        self.declare_parameters(namespace='',
                                parameters=[('goals_list',[3.0, 4.0,5.0,8.0]),
                                            ('velocity_vector',[0.5,0.0,0.0,0.0,0.0,0.5])])
        
        self.goals_list_ = self.get_parameter('goals_list').value
        self.goals_list = []
        for i in range(0,len(self.goals_list_),2):
            """ if ((self.goals_list_[i] < 11.5) and (self.goals_list_[i] > -11.5)) and ((self.goals_list_[i+1] > -11.5) and (self.goals_list_[i+1] < 11.5)):
                self.goals_list.append(self.goals_list_[i])
                self.goals_list.append(self.goals_list_[i+1])
             """
                
            if(self.goals_list_[i] > 11.5):
                self.goals_list_[i] = 9.5
                self.goals_list.append(self.goals_list_[i])
            elif (self.goals_list_[i] < -11.5):
                self.goals_list_[i] = -9.5
                self.goals_list.append(self.goals_list_[i])
            else:
                self.goals_list.append(self.goals_list_[i])

            if (self.goals_list_[i+1] > 11.5):
                self.goals_list_[i+1] = 9.5
                self.goals_list.append(self.goals_list_[i+1])
            elif (self.goals_list_[i+1] < -11.5):
                self.goals_list_[i+1] = -9.5
                self.goals_list.append(self.goals_list_[i+1])
            else:
                self.goals_list.append(self.goals_list_[i+1])

            
                

        
        self.velocity_vector = self.get_parameter('velocity_vector').value
        self.get_logger().info(f"velocity_vector: {self.velocity_vector}")
        self.get_logger().info(f"goals_list: {self.goals_list}")

        self.num_goals = len(self.goals_list) / 2

        self.get_logger().info(f"Number of Goals: {self.num_goals}")

        self.velocity_msg = Twist()
        self.goal_pose_msg = Float64MultiArray()
        self.angle_difference_msg = Float32()
        self.distance_difference_msg = Float32()

        self.velocity_topic = 'cmd_vel'
        self.goal_pose_topic = 'goal_pose'
        self.angle_difference_topic = 'angle_diff'
        self.distance_difference_topic = 'distance_diff'
        
        self.velocity_publisher = self.create_publisher(Twist,self.velocity_topic,10)
        self.goal_pose_publisher = self.create_publisher(Float64MultiArray,self.goal_pose_topic,10)
        self.angle_difference_subscriber = self.create_subscription(Float32, self.angle_difference_topic,self.angle_difference_subscription_callback,10)
        self.distance_difference_subscriber = self.create_subscription(Float32, self.distance_difference_topic,self.distance_difference_subscription_callback,10)

        timer_period_velocity = 0.1  # seconds
        self.timer_pub_vel = self.create_timer(timer_period_velocity, self.velocity_publisher_callback)

        timer_period_goal_position = 0.1  # seconds
        self.timer_pub_goal_position = self.create_timer(timer_period_goal_position, self.goal_pose_publisher_callback)

        self.angle_difference = 0.0
        self.distance_difference = 0.0

        self.K_linear = 1
        self.K_angular = 1
        self.flag = False
        self.i = 0
        self.completed_goals = 0
        self.goal_pose_msg.data = [0.0,0.0]

    def velocity_publisher_callback(self):


        if (self.angle_difference > 0.01) and (self.flag == False):
            self.velocity_msg.angular.z = self.velocity_vector[5] * self.K_angular

        elif (self.angle_difference < -0.01) and (self.flag == False):
           self.velocity_msg.angular.z = -self.velocity_vector[5] * self.K_angular

        elif (self.angle_difference < 0.01) and (self.angle_difference > -0.01) and (self.flag == False):
            self.velocity_msg.angular.z = 0.0

        if (self.distance_difference > 0.1) and (self.flag == False):
            self.velocity_msg.linear.x = self.velocity_vector[0]
            
            if (self.distance_difference < 0.3):
                self.velocity_msg.linear.x = 0.0
                self.velocity_msg.angular.z = 0.0
                self.i += 2
                self.completed_goals += 1
                

        if self.completed_goals == self.num_goals:
            self.velocity_msg.linear.x = 0.0
            self.velocity_msg.angular.z = 0.0
            self.flag = True
        
        
        self.velocity_publisher.publish(self.velocity_msg)

    def goal_pose_publisher_callback(self):
        self.goal_pose_msg.data = [float(self.goals_list[self.i]),float(self.goals_list[self.i + 1])]
        self.goal_pose_publisher.publish(self.goal_pose_msg)


    def angle_difference_subscription_callback(self,msg):
        self.angle_difference = msg.data

    def distance_difference_subscription_callback(self,msg):
        self.distance_difference = msg.data

def main():
    
    rclpy.init()
    controller_node = Controller()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()