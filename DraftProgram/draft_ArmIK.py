#!/usr/bin/env python3

import numpy as np

# 各関節角度(初期値: 0.0)
q_ArmL = np.array([0.0, 0.0, 0.0])

# 目標リンク位置
r_wa = np.array([0.0, 0.0, 0.0])

# 各リンク位置
r_w1 = np.array([0.0, 0.0575, 0.0])
r_12 = np.array([0.0, 0.0245, -0.016])
r_23 = np.array([0.016, 0, -0.06])
r_3a = np.array([0, 0, -0.129])
r_1a = r_wa - r_w1
r_13 = r_1a - r_3a

# theta3
A = np.sqrt(r_13[0]**2 + r_13[1]**2 + r_13[2]**2)
B = r_3a[2]
C = np.sqrt(r_1a[0]**2 + r_1a[1]**2 + r_1a[2]**2)
q_ArmL[2] = -1 * np.arccos((A**2 + B**2 - C**2)/(2 * A * B)) + np.pi

# theta2
# print((r_wa[1] - r_12[1] - r_w1[1])/(np.cos(q_ArmL[2]) * r_3a[2] + r_23[2]))
a = r_12[1]
b = r_23[2]
print((a**2  + b**2 - A**2)/(2 * a * b))
# q_ArmL[1] = -1 * np.arccos((a**2  + b**2 - A**2)/(2 * a * b)) + np.pi
q_ArmL[1] = -1 * np.arcsin((r_wa[1] - r_12[1] - r_w1[1])/(np.cos(q_ArmL[2]) * r_3a[2] + r_23[2]))

# theta1
X = np.sin(q_ArmL[2]) * r_3a[2] + r_23[0]
Y = np.cos(q_ArmL[1]) * r_23[2] + r_12[2]
q_ArmL[0] = np.arcsin((r_wa[0] * Y - r_wa[1] * X)/(X**2 + Y**2))

print("target: ", r_wa)
print("joint angle: ", np.rad2deg(q_ArmL))
