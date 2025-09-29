import numpy as np
from numpy.typing import NDArray


# System Reference Dimensions


# Cable Frame Height/Width Inches
alpha = 32.25
beta = 31.5

# Cable Square Height/Width Inches
delta = 1.9
gamma = 6.1

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
    LM1 = A
    LM2 = C
    LM3 = D
    LM4 = B

    return np.array([LM1, LM2, LM3, LM4])


reference_lengths = find_lengths(0, 15)
circumference = 2 * np.pi * pulley_radius


def get_motor_positions(x, y, reference_motor_pos: NDArray, reference_length: NDArray = reference_lengths):
    new_lengths = find_lengths(x, y)

    # For some reason, motors move 1 full rotation for each .5 (rev) commanded through odrive
    # That's where the .5 multiplier comes from
    # required rotation from reference position
    req_revs_from_reference = 0.5 * (new_lengths - reference_length) / circumference
    req_revs_from_reference *= np.array([-1, -1, 1, 1])

    # TODO: MAKE SURE THIS HANDLES ALL 4 MOTORS
    # req_revs_from_reference = np.array([req_revs_from_reference[0], req_revs_from_reference[3]]) # this is for motors controlling top cables
    req_revs_from_reference = np.array(
        [req_revs_from_reference[1], req_revs_from_reference[2]]
    )  # this is for motors controlling bottom cables
    return reference_motor_pos + req_revs_from_reference
