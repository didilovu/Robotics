import time
import math

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Twist
from action_turtle_commands.action import MessageTurtleCommands


class MessageTurtleActionServer(Node):

    def __init__(self):
        super().__init__('message_turtle_action_server')
        self._action_server = ActionServer(
            self,
            MessageTurtleCommands,
            'messageturtle',
            self.execute_callback)
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 1)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        command = goal_handle.request.command
        s = goal_handle.request.s
        angle = goal_handle.request.angle
        message = Twist()
        self.publisher_.publish(message)

        feedback_msg = MessageTurtleCommands.Feedback()
        feedback_msg.odom = 0

        if command == "forward":
            self.get_logger().info('Goal: forward, {0} metres '.format(goal_handle.request.s))
            for i in range(goal_handle.request.s):
                feedback_msg.odom += 1
                message.linear.x += 1.0
                self.publisher_.publish(message)
                self.get_logger().info('Feedback: {0} metres'.format(feedback_msg.odom))
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(1)
        elif command == "turn_right":
            self.get_logger().info('Goal: turn right, {0} degrees '.format(goal_handle.request.angle))
            message.angular.z = - angle * math.pi / 180.0 # 1.57 radians = 90 degrees
            self.publisher_.publish(message)
            self.get_logger().info('Feedback: {0} metres'.format(feedback_msg.odom))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)
        else:
            self.get_logger().info('Goal: turn left, {0} degrees '.format(goal_handle.request.angle))
            message.angular.z = angle * math.pi / 180.0 
            self.publisher_.publish(message)
            self.get_logger().info('Feedback: {0} metres'.format(feedback_msg.odom))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()
        result = MessageTurtleCommands.Result()
        result.result = True
        return result


def main(args=None):
    rclpy.init(args=args)
    message_turtle_action_server = MessageTurtleActionServer()
    rclpy.spin(message_turtle_action_server)


if __name__ == '__main__':
    main()
