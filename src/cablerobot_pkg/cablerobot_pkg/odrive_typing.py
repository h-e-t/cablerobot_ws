from typing import Protocol, Any


# -------------------------------
# Low-level ODrive objects
# -------------------------------
class Motor(Protocol):
    error: int
    is_calibrated: bool


class Encoder(Protocol):
    pos_estimate: float
    vel_estimate: float


class Controller(Protocol):
    input_pos: float
    input_vel: float
    input_torque: float


class Axis(Protocol):
    error: int
    current_state: int
    requested_state: int
    motor: Motor
    encoder: Encoder
    controller: Controller


# -------------------------------
# Top-level ODrive device
# -------------------------------
class ODriveHandle(Protocol):
    serial_number: int
    vbus_voltage: float

    axis0: Axis
    axis1: Axis

    def clear_errors(self) -> None: ...
    def reboot(self) -> None: ...
