import numpy as np
from numpy.typing import NDArray


# System Reference Dimensions


# Cable Frame Height/Width Inches
alpha = 32.25
beta = 31.5

# Cable Square Height/Width Inches
delta = 6.47
gamma = 6.47

# Cable Frame Corners
p1 = np.array([-beta / 2, alpha / 2])
p2 = np.array([beta / 2, alpha / 2])
p3 = np.array([-beta / 2, -alpha / 2])
p4 = np.array([beta / 2, -alpha / 2])

pulley_radius = 1.98 / 2  # inches


def find_lengths(x_desired, y_desired):
    p_a = np.array([x_desired - gamma / 2, y_desired + delta / 2])
    p_b = np.array([x_desired + gamma / 2, y_desired + delta / 2])
    p_c = np.array([x_desired - gamma / 2, y_desired - delta / 2])
    p_d = np.array([x_desired + gamma / 2, y_desired - delta / 2])

    A = np.linalg.norm(p_a - p1)
    B = np.linalg.norm(p_b - p2)
    C = np.linalg.norm(p_c - p3)
    D = np.linalg.norm(p_d - p4)

    # Remapping to motorspace from calculation space
    return np.array([C, D, B, A])


reference_lengths = find_lengths(0, 0)
circumference = 2 * np.pi * pulley_radius


def get_motor_positions(x, y, reference_motor_pos: NDArray, reference_length: NDArray = reference_lengths):
    new_lengths = find_lengths(x, y)

    # required rotation from reference position
    req_revs_from_reference = (new_lengths - reference_length) / circumference
    req_revs_from_reference *= np.array([-1, 1, 1, -1])

    return reference_motor_pos + req_revs_from_reference


class CableRobotCalculator:
    def __init__(
        self,
        frame_height=32.25,
        frame_width=31.5,
        carriage_height=6.47,
        carriage_width=6.47,
        reference_carriage_position: list = [0, 12.94],  # Defaults to top cables horizontal
        reference_motor_positions: NDArray = np.array([0, 0, 0, 0]),
    ):
        self.frame_height = frame_height
        self.frame_width = frame_width

        self.carriage_width = carriage_width
        self.carriage_height = carriage_height

        self.p1 = np.array([-self.frame_width / 2, self.frame_height / 2])
        self.p2 = np.array([self.frame_width / 2, self.frame_height / 2])
        self.p3 = np.array([-self.frame_width / 2, -self.frame_height / 2])
        self.p4 = np.array([self.frame_width / 2, -self.frame_height / 2])

        self.reference_lengths = self._find_lengths(reference_carriage_position[0], reference_carriage_position[1])

        # Values are given at reference position desired
        self.reference_motor_positions = reference_motor_positions

    def _find_lengths(self, x_desired, y_desired):
        p_a = np.array([x_desired - self.carriage_width / 2, y_desired + self.carriage_height / 2])
        p_b = np.array([x_desired + self.carriage_width / 2, y_desired + self.carriage_height / 2])
        p_c = np.array([x_desired - self.carriage_width / 2, y_desired - self.carriage_height / 2])
        p_d = np.array([x_desired + self.carriage_width / 2, y_desired - self.carriage_height / 2])

        A = np.linalg.norm(p_a - p1)
        B = np.linalg.norm(p_b - p2)
        C = np.linalg.norm(p_c - p3)
        D = np.linalg.norm(p_d - p4)

        # Remapping to motorspace from calculation space
        # return np.array([C, D, B, A]),np.array([p_a, p_b, p_c, p_d])
        return np.array([C, D, B, A])

    def get_motor_positions(self, x, y) -> NDArray:
        new_lengths = self._find_lengths(x, y)

        # required rotation from reference position
        req_revs_from_reference = (new_lengths - self.reference_lengths) / circumference
        req_revs_from_reference *= np.array([-1, 1, 1, -1])

        return self.reference_motor_positions + req_revs_from_reference

    def set_reference_motor_positions(self, ref_positions):
        self.reference_motor_positions = np.array(ref_positions)
