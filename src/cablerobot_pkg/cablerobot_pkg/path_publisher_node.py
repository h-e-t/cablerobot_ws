#!usr/bin/env python3

# ROS Imports
from os import MFD_HUGE_512KB
from numpy.typing import NDArray
import rclpy
from rclpy.node import Node

# External Imports
import time
import numpy as np
from sshkeyboard import listen_keyboard
from time import sleep
from numpy.typing import NDArray

# Interfaces
from cablerobot_interfaces.msg import MotorPositionCommand
from cablerobot_interfaces.msg import ManualCommand
from std_msgs.msg import Float32MultiArray


class PathPublisherNode(Node):
    def __init__(self):
        super().__init__("PathPublisherNode")

        self.motor_position_publisher_ = self.create_publisher(MotorPositionCommand, "motor_position_command", 10)

        # self.end_effector_position_publisher_ = self.create_publisher(
        #     MotorPositionCommand, "end_effector_position_command", 10
        # )
        self.end_effector_position_publisher_ = self.create_publisher(
            ManualCommand, "end_effector_position_command", 10
        )

        self.get_logger().info("Path Publisher Node Initialized. Path beginning")

        self.motor_pos = [0] * 4

        self.motor_pos_sub = self.create_subscription(
            Float32MultiArray, "motor_position", self.callback_get_motor_positions, 10
        )

    def publishPosition(self, position: list):
        pos = ManualCommand()

        pos.command = position
        pos.state = 3

        self.end_effector_position_publisher_.publish(pos)

    def publishMotorPositions(self, motor_positions: list):
        pos = MotorPositionCommand()

        pos.command = motor_positions

        print("SOMETHING")
        self.motor_position_publisher_.publish(pos)

    def callback_get_motor_positions(self, msg: Float32MultiArray):
        self.motor_pos = msg.data

    def get_motor_pos(self):
        self.get_logger().info(f"Motor Positions {self.motor_pos}")
        return self.motor_pos


def main(args=None):
    rclpy.init(args=args)

    node = PathPublisherNode()
    rclpy.spin_once(node, timeout_sec=1)

    freq = 100.0
    pub_period = 1.0 / freq
    spin_ct = 0

    pathToData = "/home/h-e-t/cablerobot_ws/src/cablerobot_pkg/cablerobot_pkg/data/circle_new_cage.csv"

    table = np.loadtxt(pathToData, delimiter=",")

    [m1_offset, m2_offset, m3_offset, m4_offset] = node.get_motor_pos()

    offsetTuningParam = 2  # degrees

    m1_offset += np.deg2rad(-offsetTuningParam) / (2 * np.pi)
    m2_offset += np.deg2rad(offsetTuningParam) / (2 * np.pi)
    m3_offset += np.deg2rad(offsetTuningParam) / (2 * np.pi)
    m4_offset += np.deg2rad(-offsetTuningParam) / (2 * np.pi)

    node.publishMotorPositions([m1_offset, m2_offset, m3_offset, m4_offset])

    input("Waiting for user input for confirmation (PRESS ENTER) ")

    while spin_ct < len(table[0]):
        if spin_ct >= len(table[0]):
            node.get_logger().warn("Path data completed.")
            spin_ct = 0

        # Send list of 4 motor angle values to [msg].command
        command = [
            -(float(table[0][spin_ct] - table[0][0]) / (4 * np.pi)) + m1_offset,
            -(float(table[1][spin_ct] - table[1][0]) / (4 * np.pi)) + m2_offset,
            -(float(table[2][spin_ct] - table[2][0]) / (4 * np.pi)) + m3_offset,
            -(float(table[3][spin_ct] - table[3][0]) / (4 * np.pi)) + m4_offset,
        ]

        node.publishMotorPositions(command)
        time.sleep(pub_period)
        spin_ct += 1

    rclpy.shutdown()


if __name__ == "__main__":
    main()
