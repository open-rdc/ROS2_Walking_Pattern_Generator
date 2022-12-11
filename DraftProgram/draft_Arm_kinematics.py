#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()

# 3D表示
# ax = fig.add_subplot(projection='3d')
# ax.set_title('Left-Arm Kinematics')
# ax.set_xlabel('axis x')
# ax.set_ylabel('axis y')
# ax.set_zlabel('axis z')

# 2D表示
plt.title("Left_Arm Kinematics")
plt.xlabel("axis x")
plt.ylabel("axis z")

# 各リンク位置
p_w1 = np.array([0.0, 0.0575+0.0245, 0.0])
p_12 = np.array([0.0, 0.0, -0.016])
p_23 = np.array([0.016, 0.0, -0.06])
p_3a = np.array([0.0, 0.0, -0.129])

for i in range(0, 91):
    """ FK """
    # 出力：目標リンク位置
    fk_wa = np.array([0.0, 0.0, 0.0])

    # 入力：各関節角度
    fk_q = np.array([np.deg2rad(-0), np.deg2rad(0), np.deg2rad(-i)])

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

    # target point 
    fk_wa = fk_Rw1 @ fk_R12 @ fk_R23 @ p_3a + fk_Rw1 @ fk_R12 @ p_23 + fk_Rw1 @ p_12 + p_w1

    # output result> ", ik_q)
    print("import: joint angles -> ", np.rad2deg(fk_q))
    print("export: target point -> ", fk_wa)
    # ax.scatter(fk_wa[0], fk_wa[1], fk_wa[2], label='target', color='b')
    plt.scatter(fk_wa[0], fk_wa[2], color="b")


    """ IK """
    # 出力：各関節角度(初期値: 0.0)
    ik_q = np.array([0.0, 0.0, 0.0])

    # 入力：目標リンク位置
    # ik_1a = np.array([0.016, 0.082, -0.205])
    ik_1a = fk_wa - p_w1
    plt.scatter(fk_wa[0], fk_wa[2], color="b")
    # リンク長etc, 定数
    l_pitch = -1 * np.sqrt(ik_1a[0]**2 + ik_1a[2]**2)
    l_1 = -1 * np.sqrt((p_12[0] + p_23[0])**2 + (p_12[2] + p_23[2])**2)
    # l_1 = p_12[2] + p_23[2]
    l_2 = p_3a[2]

    l_roll = -1 * np.sqrt(ik_1a[1]**2 + ik_1a[2]**2)

    # joint1 (pitch)
    ik_q[0] = np.arctan(ik_1a[0] / ik_1a[2]) - np.arccos((l_1**2 - l_2**2 + ik_1a[0]**2 + ik_1a[2]**2) / (2 * l_1 * l_pitch))

    # joint3 {pitch}
    ik_q[2] = np.arccos((ik_1a[0]**2 + ik_1a[2]**2 - l_1**2 - l_2**2) / (2 * l_1 * l_2))

    # joint2 (roll)
    ik_Rw1 = np.array([[np.cos(ik_q[0]), 0.0, -1*np.sin(ik_q[0])], 
                    [0.0, 1.0, 0.0],
                    [np.sin(ik_q[0]), 0.0, np.cos(ik_q[0])]])
    ik_R23 = np.array([[np.cos(ik_q[2]), 0.0, -1*np.sin(ik_q[2])], 
                    [0.0, 1.0, 0.0],
                    [np.sin(ik_q[2]), 0.0, np.cos(ik_q[2])]])
    ik_2a = ik_Rw1 @ ik_1a - p_12
    ik_q[1] = np.arctan(ik_2a[1] / ik_2a[2])

    # output result
    print("import: target point(joint1~end) -> ", ik_1a)
    print("export: joint angles(deg) -> ", np.rad2deg(ik_q))
    print("export: joint angles(rad) -> ", ik_q)

    """ FK2 """
    # 出力：目標リンク位置
    fk2_wa = np.array([0.0, 0.0, 0.0])

    # 入力：各関節角度
    fk2_q = ik_q

    # 各リンク回転行列（fk_q依存）
    fk2_Rw1 = np.array([[np.cos(fk2_q[0]), 0.0, np.sin(fk2_q[0])], 
                    [0.0, 1.0, 0.0],
                    [-1*np.sin(fk2_q[0]), 0.0, np.cos(fk2_q[0])]])

    fk2_R12 = np.array([[1.0, 0.0, 0.0],
                    [0, np.cos(fk2_q[1]), -1*np.sin(fk2_q[1])], 
                    [0, np.sin(fk2_q[1]), np.cos(fk2_q[1])]])

    fk2_R23 = np.array([[np.cos(fk2_q[2]), 0.0, np.sin(fk2_q[2])], 
                    [0.0, 1.0, 0.0],
                    [-1*np.sin(fk2_q[2]), 0.0, np.cos(fk2_q[2])]])

    # target point 
    fk2_wa = fk2_Rw1 @ fk2_R12 @ fk2_R23 @ p_3a + fk2_Rw1 @ fk2_R12 @ p_23 + fk2_Rw1 @ p_12 + p_w1

    # output result
    print("import: joint angles -> ", np.rad2deg(fk2_q))
    print("export: target point -> ", fk2_wa)
    # ax.scatter(fk2_wa[0], fk2_wa[1], fk2_wa[2], label='result', color='r')
    plt.scatter(fk2_wa[0], fk2_wa[2], color="r")

plt.scatter(fk_wa[0], fk_wa[2], label="target", color="b")
plt.scatter(fk2_wa[0], fk2_wa[2], label="result", color="r")
plt.legend()
plt.show()