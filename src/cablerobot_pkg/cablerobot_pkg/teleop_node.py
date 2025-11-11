#!usr/bin/env python3

# ROS Imports
import rclpy
from rclpy.node import Node

# External Imports
import numpy as np
from sshkeyboard import listen_keyboard
import time

# Odrive Imports
from odrive.enums import AxisState
from odrive.enums import ControlMode

# Internal Imports
from cablerobot_interfaces.msg import ManualCommand
from cablerobot_interfaces.srv import AcquireWorkspace
from std_msgs.msg import Float32MultiArray

# Interfaces
from std_srvs.srv import Trigger


class TeleopNode(Node):
    def __init__(self):
        super().__init__("cable_teleop_node")
        self.end_effector_position_pub_ = self.create_publisher(ManualCommand, "end_effector_position_command", 10)
        self.position_command = np.array([0, 12.94])

        self.command = np.array([0, 0, 0, 0], dtype=np.float64)
        self.commandMode = AxisState.IDLE

        self.calibrate_client_ = self.create_client(Trigger, "calibrate_motors")

        self.acquire_workspace_client = self.create_client(Trigger, "acquire_workspace")

        self.state = 0
        self.spoolingDirection = 1

        self.reference = [0, 0, 0, 0]

        self.dif = 0.4

        self.motorPositions = []

        self.motorPositionsub_ = self.create_subscription(
            Float32MultiArray, "motor_position", self.callback_get_motor_positions, 10
        )

        self.get_logger().info("Teleop Node initialized")

    def callback_get_motor_positions(self, msg: Float32MultiArray):
        print("MOTOR RECEIVED")
        self.motorPositions = np.array(msg.data)

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
        match key:
            case "p":  # position control
                self.get_logger().info("TELEOP IN POSITION CONTROL")
                self.state = ControlMode.POSITION_CONTROL
                self.position_command = np.array([0, 12.94])
            case "r":  # spooling mode
                self.get_logger().info("TELEOP Spooling...")
                self.command = np.zeros(4)
            case "c":  # calibrate
                self.state = -1
                self.call_calibrate()
                return
            case "b":
                self.state = -1
                self.command = [0.0, 0.0, 0.0, 0.0]
                self.call_workspace()
                return

        if self.state == 0:  # Handle Spooling
            match key:
                case "m":
                    self.spoolingDirection = self.spoolingDirection * -1
                case "a":  # motor 1
                    self.command = self.command + self.spoolingDirection * np.array([0.05, 0, 0, 0])
                case "s":  # motor 2
                    self.command = self.command + self.spoolingDirection * np.array([0, 0.05, 0, 0])
                case "d":  # motor 3
                    self.command = self.command + self.spoolingDirection * np.array([0, 0, 0.05, 0])
                case "f":  # motor 4
                    self.command = self.command + self.spoolingDirection * np.array([0, 0, 0, 0.05])
            msg.state = self.state
            msg.command = list(self.command + self.motorPositions)
            self.end_effector_position_pub_.publish(msg)
            return
        else:
            match key:
                case "w":  # up
                    self.position_command = self.position_command + np.array([0.0, self.dif])
                case "s":  # down
                    self.position_command = self.position_command + np.array([0.0, -self.dif])
                case "a":  # left
                    self.position_command = self.position_command + np.array([-self.dif, 0.0])
                case "d":  # right
                    self.position_command = self.position_command + np.array([self.dif, 0.0])

            msg.state = self.state
            msg.command = list(self.position_command)

            print(f"publishing  {msg.command}")
            self.end_effector_position_pub_.publish(msg)
            # self.get_logger().info(f"Command (x,y) = ({self.command[0]:.2f}, {self.command[1]:.2f}) State={self.state}")
            return


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    time.sleep(0.1)
    rclpy.spin_once(node)

    listen_keyboard(
        on_press=node.handleKeys, sleep=0.001, delay_second_char=0.001, delay_other_chars=0.001, sequential=False
    )


if __name__ == "__main__":
    main()
