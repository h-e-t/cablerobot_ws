import odrive
from sshkeyboard import listen_keyboard
from odrive.enums import AxisState
import time
from functools import partial

controlInput = 0.0
controlType = AxisState.FULL_CALIBRATION_SEQUENCE  # P position T torque V velocity C calibrate


def on_press(key, motor):
    global controlInput

    if key == "w":
        controlInput += 0.1
    elif key == "s":
        controlInput -= 0.1
    elif key == "c":
        pass
    else:
        return False

    print(f"{controlInput:.3f}")
    return True


def main():
    # print("Connecting to motor")
    # od1 = odrive.find_sync()

    # print("Motor calibrating")
    # od1.axis0.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE

    # while od1.axis0.current_state != AxisState.IDLE:
    #     time.sleep(0.1)

    # print("Motor Ready")
    od1 = "blah"
    print("Enter Input and press enter as desired: ")

    try:
        listen_keyboard(
            on_press=partial(on_press, motor=od1),
            sleep=0.001,
            delay_second_char=0.001,
            delay_other_chars=0.001,
            sequential=True,
        )
    except KeyboardInterrupt:
        print("Motors powering down")


if __name__ == "__main__":
    main()
