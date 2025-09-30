import csv
import numpy as np



table = np.loadtxt('/home/h-e-t/cablerobot_ws/src/cablerobot_pkg/cablerobot_pkg/data/motor_angles.csv', delimiter=",")

spin_ct = 0
idx = 0


print([float(table[0][spin_ct]),
        float(table[1][spin_ct]),
        float(table[2][spin_ct]),
        float(table[3][spin_ct])])