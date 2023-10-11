import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import sys
import math

class Turtle(Node):
    def __init__(self):
        super().__init__("move_to_goal_Node")
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.timer = self.create_timer(0.1, self.go_to_goal)
        self.pose = Pose()

    def pose_callback(self, data):
        self.pose = data

    def go_to_goal(self):
        goal = Pose()
        goal.x = float(sys.argv[1])
        goal.y = float(sys.argv[2])
        goal.theta = float(sys.argv[3])

        new_vel = Twist()

        distance_to_goal = math.sqrt((goal.x - self.pose.x)**2+(goal.y - self.pose.y)**2)
        angle_to_goal =math.atan2(goal.y - self.pose.y , goal.x - self.pose.x)

        angle_error = angle_to_goal - self.pose.theta
	
        if distance_to_goal >= 0.1:
            self.get_logger().info('Distance to the goal: {0}'.format(distance_to_goal))
            if abs(angle_error) > 0.1:
                new_vel.angular.z =  angle_error
            else :
                new_vel.angular.z = 0.0
            new_vel.linear.x =  0.2*distance_to_goal
        else :
            if abs(goal.theta - self.pose.theta) > 0.001:
                new_vel.angular.z = 2*(goal.theta - self.pose.theta)
            else:
                new_vel.linear.x= 0.0
                new_vel.angular.z= 0.0
                self.get_logger().info("Goal Reached ")
                quit()


        self.cmd_vel_pub.publish(new_vel)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Turtle()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
