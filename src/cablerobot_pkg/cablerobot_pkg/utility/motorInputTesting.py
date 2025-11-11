from calculateMotorPositions import CableRobotCalculator
from calculateMotorPositions import get_motor_positions
import numpy as np
import matplotlib.pyplot as plt

positions = [[0, y] for y in np.linspace(0, 6, 50)]
positions += [[x, 6] for x in np.linspace(0, -6, 50)]
positions += [[-6, y] for y in np.linspace(6, -6, 50)]
positions += [[x, -6] for x in np.linspace(-6, 6, 50)]  # left bottom to right bottom
positions += [[6, y] for y in np.linspace(-6, 6, 50)]  # right bottom to right top
positions += [[x, 6] for x in np.linspace(6, 0, 50)]  # right bottom to right top
positions += [[0, y] for y in np.linspace(6, 0, 50)]  # right bottom to right top

calculator = CableRobotCalculator(reference_carriage_position=[0, 0])

motorPositions = []
for pos in positions:
    # motorPositions += [calculator.get_motor_positions(pos[0], pos[1])]
    motorPositions += [get_motor_positions(pos[0], pos[1], reference_motor_pos=np.array([0, 0, 0, 0]))]

motorPositions = np.array(motorPositions).T


# 12 inch side box

pathToData = "/home/h-e-t/cablerobot_ws/src/cablerobot_pkg/cablerobot_pkg/data/center_box.csv"
table = np.loadtxt(pathToData, delimiter=",")
table[0] = table[0] - table[0][0]
table[1] = table[1] - table[1][0]
table[2] = table[2] - table[2][0]
table[3] = table[3] - table[3][0]

table_time = np.linspace(0, 15, len(table[0]))
my_time = np.linspace(0, 15, len(positions))

fig, axes = plt.subplots(4, 1)

for i, ax in enumerate(axes):
    ax.plot(table_time, table[i] / (4 * np.pi), label="table")
    # ax.plot(my_time, motorPositions[i], label="my data")

axes[3].legend()
plt.show()
