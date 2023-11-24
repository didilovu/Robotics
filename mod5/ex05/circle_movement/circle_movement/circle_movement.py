import rclpy, sys
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
from math import pi


class CirclePublisher(Node):

    def __init__(self):
        super().__init__('publisher')
        self.i = 0
        
        self.publisher = self.create_publisher(Twist, '/kak/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.radius = self.declare_parameter('radius', 0.5).get_parameter_value().double_value

    def timer_callback(self):
        self.i = self.i + 1
        twist = Twist()
        if self.i < 10:
            twist.linear.x = -2.0
        elif self.i == 10:
            twist.angular.z = 2.5
        elif self.i < 20:
            twist.linear.y = -2.0
        elif self.i == 20:
            twist.angular.z = 2.5
        elif self.i <30:
            twist.linear.x = -2.0
        elif self.i == 30:
            twist.angular.z = 2.5
        elif self.i < 40:
            twist.linear.x = -2.0
        elif self.i == 40:
            self.i = 0.0
        self.publisher.publish(twist)
        
        
def main(args=None):
    rclpy.init(args=args)
    circling = CirclePublisher()
    rclpy.spin(circling)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
