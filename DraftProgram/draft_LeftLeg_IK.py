#!/usr/bin/env python3

import numpy as np

def identityMatrix():
    return np.matrix([[1, 0, 0],
                      [0, 1, 0],
                      [0, 0, 1]])

def Rx(theta):
    return np.matrix([[1, 0            , 0             ], 
                      [0, np.cos(theta), -np.sin(theta)], 
                      [0, mp.sin(theta), np.cos(theta) ]])
def Ry(theta):
    return np.matrix([[np.cos(theta) , 0, np.sin(theta)],
                      [0,            , 1, 0            ],
                      [-np.sin(theta), 0, np.cos(theta)]])
def Rz(theta):
    return np.matrix([[np.cos(theta), -np.sin(theta), 0],
                      [np.sin(theta), np.cos(theta) , 0],
                      [0,           , 0,            , 1]])

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