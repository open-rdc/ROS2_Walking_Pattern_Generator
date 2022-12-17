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

Pw1 = np.matrix([-0.005, 0.037, -0.1222])
P12 = np.matrix([0, 0, 0])
P23 = np.matrix([0, 0, 0])
P34 = np.matrix([0, 0, -0.093])
P45 = np.matrix([0, 0, -0.093])
P56 = np.matrix([0, 0, 0])
# print(Pw1)

# Target_Point
# Pw6_target = Pw1 + P34 + P45/2
# Pw6_target = Pw1 + P34 
# Pw6_target = Pw1 + P34/2
Rw6_target = identityMatrix()
Pw6_target = np.array([0.05, 0.1, -0.194])


""" IK """
qqq = Pw1 - Pw6_target
# print(qqq.T)
P16_target = np.array((Rw6_target * qqq.T).T)[0]
# print(P16_target)

a = np.abs(P34[0,2])
b = np.abs(P45[0,2])
c = np.sqrt(P16_target[0]**2 + P16_target[1]**2 + P16_target[2]**2)
# print(a, b, c)

Q3 = -np.arccos((a**2 + b**2 - c**2)/(2 * a * b)) + np.pi
alfa = np.arcsin((a * np.sin(np.pi - Q3))/c)
Q4 = -np.arctan2(P16_target[0], np.sign(P16_target[2]) * np.sqrt(P16_target[1]**2 + P16_target[2]**2)) - alfa
Q5 = np.arctan2(P16_target[1], P16_target[2])
# print(P16_target[1], P16_target[2], np.arctan2(P16_target[1], P16_target[2]))
# When general Rw6_target
R1_2_3 = np.dot(Rw6_target, np.dot(Rx(Q5).T, np.dot(Ry(Q4).T, Ry(Q3).T)))
Q0 = np.arctan2(-1*R1_2_3[0, 1], R1_2_3[1, 1])
Q1 = np.arctan2(R1_2_3[2, 1], -1*R1_2_3[0, 1] * np.sin(Q0) + R1_2_3[1, 1] * np.cos(Q0))
Q2 = np.arctan2(-1*R1_2_3[2, 0], R1_2_3[2, 2])
# When identity-matrix Rw6_target
# Q[0] = np.arctan2(0, np.cos(Q[5]))
# Q[1] = np.arctan2(np.sin(Q[5]), np.cos(Q[5]) * np.cos(Q[0]))
# Q[2] = np.arctan2(np.cos(Q[5]) * np.sin(Q[3] + Q[4]), np.cos(Q[5]) * np.cos(Q[3] + Q[4]))

Q = np.array([Q0, Q1, Q2, Q3, Q4, Q5])

print("target[m]: ", Pw6_target)
print("Angles[rad]: ", Q, ",\n______[deg]: ", (Q*180/np.pi).astype(np.int16))


""" FK """
Rw1 = Rz(Q0)
R12 = Rx(Q1)
R23 = Ry(Q2)
R34 = Ry(Q3)
R45 = Ry(Q4)
R345 = Ry(Q2+Q3+Q4)
R24 = Ry(Q2+Q3)
R56 = Rx(Q5)

# print(Rw1)
# print(R12)
# print(R23)
# print(R34)
# print(R45)
# print(R56)

Pw1 = np.matrix([[-0.005], [0.037], [-0.1222]])
P12 = np.matrix([[0], [0], [0]])
P23 = np.matrix([[0], [0], [0]])
P34 = np.matrix([[0], [0], [-0.093]])
P45 = np.matrix([[0], [0], [-0.093]])
P56 = np.matrix([[0], [0], [0]])
P6a = np.matrix([[0], [0], [0]])

# FK_result = Rw1 @ R12 @ R23 @ R34 @ R45 @ R56 @ P6a + Rw1 @ R12 @ R23 @ R34 @ R45 @ P56 + Rw1 @ R12 @ R23 @ R34 @ P45 + Rw1 @ R12 @ R23 @ P34 + Rw1 @ R12 @ P23 + Rw1 @ P12 + Pw1
# FK_result = Rw1@R12@R23@R34@P45 + Rw1@R12@R23@P34 + Pw1
FK_result = Rw1 @ R12 @ R24 @ P45 + Rw1 @ R12 @ R23 @ P34 + Pw1
# print(Rw1 @ R12 @ R24 @ P45, Rw1 @ R12 @ R23 @ P34)
# FK_result = np.dot(Rw1, np.dot(R12, np.dot(R23, np.dot(R34, P45)))) + np.dot(Rw1, np.dot(R12, np.dot(R23, P34))) + Pw1

print("FK_result[m]: ", FK_result.T, "\n")
