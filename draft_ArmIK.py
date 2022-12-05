#!/usr/bin/env python3

import numpy as np

q_ArmL = np.array[0.0, 0.0, 0.0]

r_wa = np.array([0, 0, 0])

r_w1 = np.array([0, 0, 0])
r_12 = np.array([0, 0, 0])
r_23 = np.array([0, 0, 0])
r_3a = np.array([0, 0, 0])
r_1a = r_wa - r_w1
r_13 = r_1a - r_3a

A = np.sqrt(r_13[0]**2 + r_13[1]**2 + r_13[2]**2)
B = r_3a[2]
C = np.sqrt(r_1a[0]**2 + r_1a[1]**2 + r_1a[2]**2)
X = np.sin(theta3) * r_3a[2] + r_23[0]
Y = np.cos(theta2) * r_23[2] + r_12[2]

q_ArmL[0]

q_ArmL[1]

q_ArmL[2]

print(r_wa)
print(q_ArmL)