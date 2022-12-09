#!/usr/bin/env python3

import numpy as np


# 各リンク位置
p_w1 = np.array([0.0, 0.0575+0.0245, 0.0])
p_12 = np.array([0.0, 0.0, -0.016])
p_23 = np.array([0.016, 0.0, -0.06])
p_3a = np.array([0.0, 0.0, -0.129])

""" FK """
# 出力：目標リンク位置
fk_wa = np.array([0.0, 0.0, 0.0])

# 入力：各関節角度
fk_q = np.array([np.deg2rad(0.0), np.deg2rad(0.0), np.deg2rad(0.0)])

# 各リンク回転行列（fk_q依存）
fk_Rw1 = np.array([[np.cos(fk_q[0]), 0.0, np.sin(fk_q[0])], 
                  [0.0, 1.0, 0.0],
                  [-1*np.sin(fk_q[0]), 0.0, np.cos(fk_q[0])]])

fk_R12 = np.array([[1.0, 0.0, 0.0],
                  [0, np.cos(fk_q[1]), -1*np.sin(fk_q[1])], 
                  [0, np.sin(fk_q[1]), np.cos(fk_q[1])]])

fk_R23 = np.array([[np.cos(fk_q[2]), 0.0, np.sin(fk_q[2])], 
                  [0.0, 1.0, 0.0],
                  [-1*np.sin(fk_q[2]), 0.0, np.cos(fk_q[2])]])

fk_wa = fk_Rw1 @ fk_R12 @ fk_R23 @ p_3a + fk_Rw1 @ fk_R12 @ p_23 + fk_Rw1 @ p_12 + p_w1

print("import: joint angles -> ", np.rad2deg(fk_q))
print("export: target point -> ", fk_wa)


""" IK """
# 出力：各関節角度(初期値: 0.0)
ik_q = np.array([0.0, 0.0, 0.0])

# 入力：目標リンク位置
ik_wa = np.array([0.0, 0.0, 0.0])



# theta3


# theta2


# theta1


print("import: target point -> ", ik_wa)
print("export: joint angles -> ", np.rad2deg(ik_q))
