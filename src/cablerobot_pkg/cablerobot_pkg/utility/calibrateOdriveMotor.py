#!usr/bin/env python3

import odrive
import odrive.utils as utils
from odrive.enums import AxisState
import time
import sys


def main(argv: list[str]):
    if len(argv) != 0:
        motor = odrive.find_sync(argv)
    else:
        motor = odrive.find_any()

    print("Connected to " + str(motor.serial_number))

    if motor.axis0.current_state == AxisState.IDLE:  # type: ignore[attr-defined]
        motor.axis0.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE  # type: ignore[attr-defined]

    while motor.axis0.current_state != AxisState.IDLE:  # type: ignore[attr-defined]
        time.sleep(0.1)

    utils.dump_errors(motor)  # type: ignore[attr-defined]


if __name__ == "__main__":
    main(sys.argv[1:])
