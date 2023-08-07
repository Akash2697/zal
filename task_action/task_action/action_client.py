#!/usr/bin/env python3
from typing import List
from custom_msgs.action import Task
import rclpy
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter



class TaskActionClient(Node):

    def __init__(self, node_name: str='action_client', *, context: Context = None, cli_args: List[str] = None, namespace: str = None, use_global_arguments: bool = True, enable_rosout: bool = True, start_parameter_services: bool = True, parameter_overrides: List[Parameter] = None, allow_undeclared_parameters: bool = False, automatically_declare_parameters_from_overrides: bool = False) -> None:
        super().__init__(node_name, context=context, cli_args=cli_args, namespace=namespace, use_global_arguments=use_global_arguments, enable_rosout=enable_rosout, start_parameter_services=start_parameter_services, parameter_overrides=parameter_overrides, allow_undeclared_parameters=allow_undeclared_parameters, automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides)
        self._action_client = ActionClient(self,Task,'task')

    def send_goal(self,goal_pose, velocity_vector):
        goal_msg = Task.Goal()
        goal_msg.goal_pose = goal_pose
        goal_msg.velocity_vector = velocity_vector

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback = self.feedback_callback(Task.Feedback()))
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        return self._send_goal_future

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.goal_reached))
        rclpy.shutdown()
    
    def feedback_callback(self,feedback):
        print("Feedback:", feedback.distance_remaining)


def main(args=None):
    rclpy.init(args=args)

    action_client = TaskActionClient()

    future = action_client.send_goal([2.0, 9.0],[0.5,0.0,0.0,0.0,0.0,0.5])

    rclpy.spin_until_future_complete(action_client, future)

    rclpy.shutdown()
if __name__ == '__main__':
    main()
