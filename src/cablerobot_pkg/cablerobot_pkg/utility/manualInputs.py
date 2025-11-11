import odrive
from odrive.utils import dump_errors
from sshkeyboard import listen_keyboard
from odrive.enums import AxisState
import time
from functools import partial

global controlInput

controlType = AxisState.FULL_CALIBRATION_SEQUENCE  # P position T torque V velocity C calibrate


def on_press(key, motor, input):
    if key == "w":
        input += 0.1
    elif key == "s":
        input += -0.1
    elif key == "c":
        motor.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
    elif key == "d":
        dump_errors(motor)

        motor.clear_errors()
    else:
        return False
    print(input)
    motor.axis0.controller.input_pos = input
    return True


def main():
    # print("Connecting to motor")
    # od1 = odrive.find_sync()

    # print("Motor calibrating")
    # od1.axis0.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE

    # while od1.axis0.current_state != AxisState.IDLE:
    #     time.sleep(0.1)

    # print("Motor Ready")
    od1 = odrive.find_sync()
    controlInput = od1.axis0.pos_estimate
    print("Enter Input and press enter as desired: ")

    try:
        listen_keyboard(
            on_press=partial(on_press, motor=od1, input=controlInput),
            sleep=0.001,
            delay_second_char=0.001,
            delay_other_chars=0.001,
            sequential=True,
        )
    except KeyboardInterrupt:
        od1.axis0.requested_state = AxisState.IDLE
        print("Motors powering down")


if __name__ == "__main__":
    main()
