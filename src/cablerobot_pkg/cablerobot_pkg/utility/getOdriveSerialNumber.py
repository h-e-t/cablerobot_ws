import odrive


def main():
    motor = odrive.find_sync()

    print("Connected Motor Serial Number = " + str(motor.serial_number))  # type: ignore[attr-defined]


if __name__ == "__main__":
    main()
