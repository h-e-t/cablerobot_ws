#!usr/bin/env python3

# ROS Imports
import rclpy
from rclpy.node import Node

# External Imports
import numpy as np
from sshkeyboard import listen_keyboard

# Odrive Imports
from odrive.enums import AxisState
from odrive.enums import ControlMode

# Internal Imports
from cablerobot_interfaces.msg import ManualCommand
from cablerobot_interfaces.srv import AcquireWorkspace

# Interfaces
from std_srvs.srv import Trigger


class TeleopNode(Node):
    def __init__(self):
        super().__init__("cable_teleop_node")
        self.motorCommandPub_ = self.create_publisher(ManualCommand, "motor_command", 10)
        self.command = np.array([0, 0], dtype=np.float64)
        self.commandMode = AxisState.IDLE
        self.calibrate_client_ = self.create_client(Trigger, "calibrate_motors")
        self.acquire_workspace_client = self.create_client(Trigger, "acquire_workspace")
        self.state = 0

    def call_calibrate(self):
        while not self.calibrate_client_.wait_for_service(1):
            self.get_logger().warn("Calibration service unavailable")

        request = Trigger.Request()

        self.calibrate_client_.call_async(request)

    def call_workspace(self):
        while not self.acquire_workspace_client.wait_for_service(1):
            self.get_logger().warn("Workspace service unavailable")

        request = Trigger.Request()

        self.acquire_workspace_client.call_async(request)

    def handleKeys(self, key: str):
        msg = ManualCommand()
        # TODO: CHANGE COMMANDS TO ALLOW TRUE POSITION CONTROL OF END EFFECTOR
        match key:
            case "t":  # torque control
                self.state = ControlMode.TORQUE_CONTROL
            case "p":  # position control
                self.state = ControlMode.POSITION_CONTROL
            # case "v":  # velocity control
            #     self.state = ControlMode.VELOCITY_CONTROL
            case "w":  # up
                self.command = self.command + np.array([0.0, 0.05])
            case "s":  # down
                self.command = self.command + np.array([0.0, -0.05])
            case "a":  # left
                self.command = self.command + np.array([-0.05, 0.0])
            case "d":  # right
                self.command = self.command + np.array([0.05, 0.0])
            case "q":  # stop
                msg.state = AxisState.IDLE
            case "c":  # calibrate
                self.call_calibrate()
            case "b":  # acquire workspace
                self.command = [0.0, 15.0]
                self.call_workspace()
            case "z":  # zero command
                self.command = [0.0 for _ in self.command]

        self.get_logger().info(f"Command (x,y) = ({self.command[0]:.2f}, {self.command[1]:.2f}) State={self.state}")

        msg.state = self.state
        msg.command = list(self.command)
        self.motorCommandPub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    listen_keyboard(
        on_press=node.handleKeys, sleep=0.001, delay_second_char=0.001, delay_other_chars=0.001, sequential=True
    )


if __name__ == "__main__":
    main()
