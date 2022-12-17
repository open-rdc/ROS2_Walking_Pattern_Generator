#!/usr/bin/env python3

import numpy as np

def identityMatrix():
    return np.matrix([[1, 0, 0],
                      [0, 1, 0],
                      [0, 0, 1]])

def Rx(theta1):
    return np.matrix([[1, 0            , 0               ], 
                      [0, np.cos(theta1), -np.sin(theta1)], 
                      [0, np.sin(theta1), np.cos(theta1) ]]
    )
def Ry(theta2):
    return np.matrix([[np.cos(theta2) , 0, np.sin(theta2)],
                      [0              , 1, 0             ],
                      [-np.sin(theta2), 0, np.cos(theta2)]]
    )
def Rz(theta3):
    return np.matrix([[np.cos(theta3), -np.sin(theta3), 0],
                      [np.sin(theta3), np.cos(theta3) , 0],
                      [0             , 0              , 1]]
    )

Pw1 = np.array([-0.005, 0.037, -0.1222])
P12 = np.array([0, 0, 0])
P23 = np.array([0, 0, 0])
P34 = np.array([0, 0, -0.093])
P45 = np.array([0, 0, -0.093])
P56 = np.array([0, 0, 0])

# Target_Point
Pw6_target = Pw1 + P34
Rw6_target = identityMatrix()
# Pw6_target = np.array([0, 0, 0])


""" IK """
P16_target = Pw6_target - Pw1

a = P34[2]
b = P45[2]
c = np.sqrt(P16_target[0]**2 + P16_target[1]**2 + P16_target[2]**2)

Q = np.array([0, 0, 0, 0, 0, 0])

Q[3] = -np.arccos((a**2 + b**2 - c**2)/(2 * a * b)) + np.pi
alfa = np.sin((a * np.sin(np.pi - Q[2]))/c)
Q[4] = -np.arctan2(-P16_target[0], np.sign(-P16_target[2]) * np.sqrt(P16_target[1]**2 + P16_target[2]**2)) - alfa
Q[5] = np.arctan2(-P16_target[1], -P16_target[2])
# When general Rw6_target
R1_2_3 = Rw6_target @ Rx(Q[5]) @ Ry(Q[3] + Q[4])
Q[0] = np.arctan2(-R1_2_3[0, 1], R1_2_3[1, 1])
Q[1] = np.arctan2(R1_2_3[2, 1], -R1_2_3[0, 1] * np.sin(Q[0]) + R1_2_3[1, 1] * np.cos(Q[0]))
Q[2] = np.arctan2(-R1_2_3[2, 0], R1_2_3[2, 2])
# When identity-matrix Rw6_target
# Q[0] = np.arctan2(0, np.cos(Q[5]))
# Q[1] = np.arctan2(np.sin(Q[5]), np.cos(Q[5]) * np.cos(Q[0]))
# Q[2] = np.arctan2(np.cos(Q[5]) * np.sin(Q[3] + Q[4]), np.cos(Q[5]) * np.cos(Q[3] + Q[4]))

print("target[m]: ", Pw6_target)
print("Angles[rad]: ", Q, ",\n______[deg]: ", (Q*180/np.pi).astype(np.int16))


""" FK """
Rw1 = Rz(Q[0])
R12 = Rx(Q[1])
R23 = Ry(Q[2])
R34 = Ry(Q[3])
R45 = Ry(Q[4])
R56 = Rx(Q[5])

# FK_result = Rw1@R12@R23@R34@R45@P56 + Rw1@R12@R23@R34@P45 + Rw1@R12@R23@P34 + Rw1@R12@P23 + Rw1@P12 + Pw1
FK_result = Rw1@R12@R23@R34@P45 + Rw1@R12@R23@P34 + Pw1

print("FK_result: ", FK_result)
