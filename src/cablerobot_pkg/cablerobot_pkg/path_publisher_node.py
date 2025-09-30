#!usr/bin/env python3

# ROS Imports
from numpy.typing import NDArray
import rclpy
from rclpy.node import Node

# External Imports
import numpy as np
from sshkeyboard import listen_keyboard
from time import sleep
from numpy.typing import NDArray

# Interfaces
from cablerobot_interfaces.msg import MotorPositionCommand
from cablerobot_interfaces.msg import ManualCommand


class PathPublisherNode(Node):
    def __init__(self, pathToData):
        super().__init__("PathPublisherNode")

        self.motor_position_publisher_ = self.create_publisher(MotorPositionCommand, "motor_position_command", 10)

        self.end_effector_position_publisher_ = self.create_publisher(
            MotorPositionCommand, "end_effector_position_command", 10
        )

        self.get_logger().info("Path Publisher Node Initialized. Path beginning")

        self.table: NDArray = np.loadtxt(pathToData, delimiter=",")

        # Frequency of publisher (100 Hz)
        freq = 5
        pub_period = 1 / freq
        self.spin_ct = 0
        self.timer = self.create_timer(pub_period, self.publishAngles)

    def publishPosition(self, position: list):
        pos = MotorPositionCommand()

        pos.command = position

        self.end_effector_position_publisher_.publish(pos)

    def publishMotorPositions(self, motor_positions: list):
        pos = MotorPositionCommand()

        pos.command = motor_positions

        self.motor_position_publisher_.publish(pos)

    def publishAngles(self):
        if self.spin_ct >= len(self.table[0]):
            self.get_logger().warn("Path data completed.")
            self.spin_ct = 0

        m1_offset = 269.05
        m2_offset = 2.361

        # Send list of 4 motor angle values to [msg].command
        pos = MotorPositionCommand()
        pos.command = [
            float(self.table[0][self.spin_ct] - self.table[0][0]) / (4 * np.pi) + m1_offset,
            float(self.table[1][self.spin_ct] - self.table[1][0]) / (4 * np.pi) + m2_offset,
            float(self.table[2][self.spin_ct]) / (4 * np.pi),  # unused
            float(self.table[3][self.spin_ct]) / (4 * np.pi),  # unused
        ]

        self.motor_position_publisher_.publish(pos)

        # Increase spin count
        self.spin_ct += 20


def main(args=None):
    rclpy.init(args=args)

    node = PathPublisherNode(
        pathToData="/home/h-e-t/cablerobot_ws/src/cablerobot_pkg/cablerobot_pkg/data/motor_angles.csv"
    )

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
