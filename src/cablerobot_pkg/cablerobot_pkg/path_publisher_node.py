#!usr/bin/env python3

# ROS Imports
from numpy.typing import NDArray
import rclpy
from rclpy.node import Node

# External Imports
import numpy as np
from sshkeyboard import listen_keyboard
from time import sleep


# Interfaces
from cablerobot_interfaces.msg import ManualCommand


class PathPublisherNode(Node):
    def __init__(self):
        super().__init__("PathPublisherNode")

        self.position_publisher_ = self.create_publisher(ManualCommand, "motor_command", 10)

        self.get_logger().info("PathPublisherNode Initialized")

    def publishPosition(self, position: list):
        pos = ManualCommand()

        pos.command = position

        self.position_publisher_.publish(pos)


def main(args=None):
    rclpy.init(args=args)

    node = PathPublisherNode()

    path = []

    squareLimits = 10.0

    path += [[-squareLimits, _] for _ in np.linspace(squareLimits, -squareLimits, 50)]
    path += [[_, -squareLimits] for _ in np.linspace(-squareLimits, squareLimits, 50)]
    path += [[squareLimits, _] for _ in np.linspace(-squareLimits, squareLimits, 50)]
    path += [[_, squareLimits] for _ in np.linspace(squareLimits, -squareLimits, 50)]

    for pos in path:
        print(pos)
        node.publishPosition(pos)
        sleep(0.2)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
