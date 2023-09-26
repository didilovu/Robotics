import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 1)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        movie = Twist()
        msg = sys.argv[1]
        if msg == "turn_right":
            movie.angular.z = -2.0
        elif msg == "turn_left":
            movie.angular.z = 2.0
        elif msg == "move_forward":
            movie.linear.x = 2.0
        elif msg == "move_backward":
            movie.linear.x = -2.0
        else:
            msg = "Error message"

        self.get_logger().info('Publishing: "%s"' % (msg))
        self.publisher_.publish(movie)


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
