import sys
from summfullname.srv import AddNames

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddNames, 'summ_full_name', self.summ_full_name_callback)

    def summ_full_name_callback(self, request, response):
        response.full_name = request.last_name + ' ' + request.name + ' ' + request.first_name
        self.get_logger().info('Incoming request\nlast name: %s name: %s  first name: %s' % (request.last_name, request.name, request.first_name))

        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
