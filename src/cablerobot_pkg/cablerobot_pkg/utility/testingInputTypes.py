from collections import deque
import time
import numpy as np

import odrive
from odrive.enums import ControlMode, AxisState
from odrive import utils

import matplotlib.pyplot as plt

dt = 0.01
window_size = 10


def main():
    try:
        position = []
        velocity = []

        odr = odrive.find_sync()
        odr.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL  # type: ignore[attr-defined]
        start = time.time()

        print("Commanding velocity")

        pos_prev = odr.axis0.pos_estimate
        vel_buf = deque(maxlen=window_size)

        odr.axis0.controller.input_vel = 5
        while time.time() - start < 10:
            time.sleep(dt)
            pos = odr.axis0.pos_estimate
            vel = (pos - pos_prev) / dt
            pos_prev = pos

            vel_buf.append(vel)

            vel_avg = sum(vel_buf) / len(vel_buf)

            position += [odr.axis0.pos_estimate]  # type: ignore[attr-defined]
            velocity += [vel_avg]

        print("Going to sleep")
        odr.axis0.requested_state = AxisState.IDLE  # type: ignore[attr-defined]

        (fig, ax) = plt.subplots()

        ax.plot(position, label="position")
        ax.plot(velocity, label="velocity")
        ax.legend()

        plt.show()

    finally:
        odr.axis0.requested_state = AxisState.IDLE  # type: ignore[attr-defined]


if __name__ == "__main__":
    main()
