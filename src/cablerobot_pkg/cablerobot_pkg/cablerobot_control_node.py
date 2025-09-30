#!usr/bin/env python3

# Ros imports
from numpy.typing import NDArray
import rclpy
from rclpy.node import Node
from rclpy import Parameter

# External Imports
import time
import numpy as np
from collections import deque

# Odrive imports
import odrive
from odrive.enums import *
from odrive.enums import ControlMode
from odrive.enums import AxisState
from odrive.enums import InputMode
import odrive.utils as utils


# Internal Imports
from .utility.getStaticTorques import getStaticTorques
from .utility.calculateMotorPositions import get_motor_positions

# Interfaces
from std_srvs.srv import Trigger
from std_msgs.msg import Float32MultiArray
from cablerobot_interfaces.msg import ManualCommand
from cablerobot_interfaces.msg import MotorPositionCommand


class CableRobotControlNode(Node):
    def __init__(self):
        super().__init__("CableRobotControlNode")

        self.od1_SN = 261832858530  # Far Left
        self.od2_SN = 997655138845  # Middle left
        self.od3_SN = 126362882843  # Middle Right (TESTING DEVICE)
        self.od4_SN = 144157530696  # Far Right

        self.motors = [None] * 4

        connectedBoards = odrive.find_sync(count=2)
        for motor in connectedBoards:  # type: ignore[attr-defined]
            match motor.serial_number:
                case self.od1_SN:
                    self.od1 = motor
                    self.get_logger().info("Motor 1 Identified")
                    self.motors[0] = self.od1
                case self.od2_SN:
                    self.od2 = motor
                    self.get_logger().info("Motor 2 Identified")
                    self.motors[1] = self.od2
                case self.od3_SN:
                    self.od3 = motor
                    self.get_logger().info("Motor 3 Identified")
                    self.motors[2] = self.od3
                case self.od4_SN:
                    self.od4 = motor
                    self.get_logger().info("Motor 4 Identified")
                    self.motors[3] = self.od4

        self.motors = [m for m in self.motors if m is not None]
        self.declare_parameter("motor_position_offsets", [0.0] * len(self.motors))

        self.motors_mode = None
        self.motors_active = False

        # Calibration Declarations
        self.declare_parameter("calibrated", False)
        self.calibrated = self.get_parameter("calibrated").value
        self.calibrationService_ = self.create_service(Trigger, "calibrate_motors", self.callbackCalibrateMotors)

        # Workspace declarations
        self.declare_parameter("workspace_acquired", False)
        self.declare_parameter("workspace_timeout", 10)
        self.getWorkspaceService_ = self.create_service(Trigger, "acquire_workspace", self.callbackGetWorkspace)

        # Motor position declarations
        self.end_effector_command_ = self.create_subscription(
            ManualCommand, "end_effector_position_command", self.callbackManualCommand, 10
        )

        self.motor_position_pub_ = self.create_publisher(Float32MultiArray, "motor_position", 10)
        self.create_timer(0.1, self.callbackPublishMotorPosition)

        # Automatic control declarations
        self.automatic_motor_position_command_ = self.create_subscription(
            MotorPositionCommand, "motor_position_command", self.callbackAutoCommand, 10
        )

        # Motor Velocity Declarations
        self.create_timer(0.01, self.callbackCalculateVelocity)
        self.prev_pos = self.getMotorPositions()
        self.vel_buffer = deque(maxlen=10)
        self.vel_avg = np.zeros(4)

        self.get_logger().info("Cable Robot Control Node Initialized")

    def setMotorPositions(self, motorPositions):
        if self.od1.axis0.current_state != AxisState.CLOSED_LOOP_CONTROL:
            self.get_logger().warn("Motors not in closed loop control mode. Position not set.")
        else:
            # TODO: Update this function to handle all motors
            self.get_logger().warn(f"{motorPositions[0]}{motorPositions[1]}")
            self.od1.axis0.controller.input_pos = motorPositions[0]
            # self.od2.axis0.controller.input_pos = motorPositions[1]
            # self.od3.axis0.controller.input_pos = motorPositions[2]
            self.od4.axis0.controller.input_pos = motorPositions[1]

    def getMotorPositions(self) -> NDArray:
        positions = np.zeros(len(self.motors))

        for i, motor in enumerate(self.motors):
            positions[i] = motor.axis0.pos_estimate  # type: ignore[attr-defined]

        return np.array(positions)

    def stop_motors(self):
        for i, motor in enumerate(self.motors):
            motor.clear_errors()  # type: ignore[attr-defined]
            motor.axis0.controller.config.control_mode = ControlMode.POSITION_CONTROL  # type: ignore[attr-defined]
            self.motors_active = False
            self.motors_mode = ControlMode.POSITION_CONTROL

    def kill_motors(self):
        for i, motor in enumerate(self.motors):
            motor.clear_errors()  # type: ignore[attr-defined]
            motor.axis0.controller.input_torque = 0  # type: ignore[attr-defined]
            motor.axis0.controller.input_vel = 0  # type: ignore[attr-defined]
            motor.axis0.requested_state = AxisState.IDLE  # type: ignore[attr-defined]

    def setCagePosition(self, x, y):
        reference_motor_positions: NDArray = np.array(self.get_parameter("motor_position_offsets").value)

        positions_required = get_motor_positions(x, y, reference_motor_positions)

        print(f"{reference_motor_positions}  {positions_required}")

        self.setMotorPositions(positions_required)

    def callbackCalibrateMotors(self, request: Trigger.Request, response: Trigger.Response):
        response = Trigger.Response()

        response.success = True
        response.message = ""

        for motor in self.motors:
            motor.clear_errors()  # type: ignore[attr-defined]

        # Calibrate all motors sequentially to prevent cable lock ups
        for motor in self.motors:
            if motor.axis0.current_state == AxisState.IDLE:  # type: ignore[attr-defined]
                motor.axis0.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE  # type: ignore[attr-defined]
            else:
                response.success = False
                response.message = "Motors not idle. State incompatible with calibration"
                break

            while motor.axis0.current_state != AxisState.IDLE:  # type: ignore[attr-defined]
                time.sleep(0.1)

            utils.dump_errors(motor)  # type: ignore[attr-defined]

        if response.success:
            response.message = "Calibration Succeded"

        self.set_parameters([Parameter("calibrated", Parameter.Type.BOOL, True)])
        return response

    def callbackGetWorkspace(self, request: Trigger.Request, response: Trigger.Response):
        response = Trigger.Response()

        if not self.get_parameter("calibrated"):
            response.message = "Workspace acquisition failed, motors not calibrated"
            response.success = False
            return response

        # TODO: Make reference length location a parameter if desired
        # right now they are fixed at 15 inches above the center of the frame
        # and baked into the get_motor_positions func

        timeout = self.get_parameter("workspace_timeout").value

        self.get_logger().info("Starting workspace acquisition.")
        self.get_logger().info("Place carriage at calibration position.")

        # 4 Motor Case
        torques = [-0.1, 0.1, 0.1, -0.1]

        # 2 Motor Case
        torques = [0.2, -0.2]

        # Arbitrary torque that allows motors to maintain tension in their respective cables

        for i, motor in enumerate(self.motors):
            motor.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL  # type: ignore[attr-defined]
            motor.axis0.controller.config.control_mode = ControlMode.TORQUE_CONTROL  # type: ignore[attr-defined]
            motor.axis0.controller.config.input_mode = InputMode.PASSTHROUGH  # type: ignore[attr-defined]
            motor.axis0.controller.input_torque = torques[i]  # type: ignore[attr-defined]

        motorPositions: NDArray = self.getMotorPositions()

        self.get_logger().info("Counting down until motor offset is set.")
        countdown = timeout
        start = time.time()

        while True:
            time.sleep(1)
            motorPositions = self.getMotorPositions()

            if time.time() - start > timeout:  # type: ignore[attr-defined]
                msg = "Workspace acquisition complete, motor offsets determined."
                self.get_logger().info(msg)

                response.success = True
                response.message = msg

                self.set_parameters(
                    [rclpy.Parameter("motor_position_offsets", rclpy.Parameter.Type.DOUBLE_ARRAY, list(motorPositions))]
                )

                break

            self.get_logger().info(f"{countdown}...")
            countdown -= 1  # type: ignore[attr-defined]

        self.get_logger().info("Calibration positions acquired. Stopping motors...")

        self.get_logger().info("Motor offsets recorded")

        for i, offset in enumerate(self.get_parameter("motor_position_offsets").value):  # type: ignore[attr-defined]
            self.get_logger().info(f"Motor {i + 1} offset = {offset}")

        self.setCagePosition(0, 15)

        for motor in self.motors:
            utils.dump_errors(motor)  # type: ignore[attr-defined]

        response.success = True
        response.message = "Calibration positions acquired"

        return response

    def callbackAutoCommand(self, msg: MotorPositionCommand):
        command = msg.command

        if not self.motors_active:
            for motor in self.motors:
                motor.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL  # type: ignore[attr-defined]
                motor.axis0.controller.config.input_mode = InputMode.POS_FILTER  # type: ignore[attr-defined]
                motor.axis0.controller.config.control_mode = ControlMode.POSITION_CONTROL  # type: ignore[attr-defined]

            self.motors_active = True

        self.setMotorPositions(command)

    def callbackManualCommand(self, msg: ManualCommand):
        command = msg.command

        if not self.motors_active:
            match msg.state:
                case 1:  # Torque Control
                    for motor in self.motors:
                        motor.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL  # type: ignore[attr-defined]
                        motor.axis0.controller.config.input_mode = InputMode.TORQUE_RAMP  # type: ignore[attr-defined]
                        motor.axis0.controller.config.control_mode = ControlMode.TORQUE_CONTROL  # type: ignore[attr-defined]
                    self.motors_mode = ControlMode.TORQUE_CONTROL
                    self.motors_active = True

                case 3:  # Position Control
                    for motor in self.motors:
                        motor.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL  # type: ignore[attr-defined]
                        motor.axis0.controller.config.input_mode = InputMode.POS_FILTER  # type: ignore[attr-defined]
                        motor.axis0.controller.config.control_mode = ControlMode.POSITION_CONTROL  # type: ignore[attr-defined]

                    self.motors_mode = ControlMode.POSITION_CONTROL
                    self.motors_active = True

        elif self.motors_mode != msg.state:
            self.get_logger().info("Motors were active. Stopping motors. Choose mode again. ")
            self.motors_active = False
            self.stop_motors()

        match self.motors_mode:
            # case 1:
            #     self.motors[i].axis0.controller.input_torque = command  # type: ignore[attr-defined]
            #     self.get_logger().info(f"Commanded Torque = {command} nm")
            # case 2:
            #     self.motors[i].axis0.controller.input_vel = command  # type: ignore[attr-defined]
            #     self.get_logger().info(f"Commanded Vel = {command} rev/s")
            case 3:
                self.setCagePosition(command[0], command[1])
                # self.motors[i].axis0.controller.input_pos = command  # type: ignore[attr-defined]
                self.get_logger().info(f"Commanded Pos = {command[0]:.2f},{command[1]:.2f}")
            case _:
                self.get_logger().info("Invalid case...")

    def callbackPublishMotorPosition(self):
        positions = Float32MultiArray

        positions.data = self.getMotorPositions()

        self.motor_position_pub_.publish(positions)

    def callbackCalculateVelocity(self):
        window_size = 10
        dt = 0.01

        position: NDArray = self.getMotorPositions()
        vel = (position - self.prev_pos) / dt
        self.prev_pos = position

        self.vel_buffer.append(vel)

        self.vel_avg = sum(self.vel_buffer) / len(self.vel_buffer)


def main(args=None):
    rclpy.init(args=args)

    node = CableRobotControlNode()

    try:
        rclpy.spin(node)
    finally:
        node.kill_motors()  # disable motors

    rclpy.shutdown()


if __name__ == "__main__":
    main()
