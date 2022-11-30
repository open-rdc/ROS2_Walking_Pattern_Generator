#!/usr/bin/env python3

""" Reference
1. https://cyberbotics.com/doc/guide/robotis-op2#position-of-the-motors
2. https://github.com/ROBOTIS-GIT/ROBOTIS-OP2-Common/tree/master/robotis_op2_description/urdf
3. https://github.com/cyberbotics/webots/blob/master/projects/robots/robotis/darwin-op/protos/Darwin-op.proto
"""

import numpy as np


def Tx(theta, vec):
    return  np.matrix([
            [1, 0, 0, vec[0]], 
            [0, np.cos(theta), np.sin(-theta), vec[1]], 
            [0, np.sin(theta), np.cos(theta), vec[2]], 
            [0, 0, 0, 1]]
    )
def Ty(theta, vec):
    return np.matrix([  
            [np.cos(theta), 0, np.sin(theta), vec[0]], 
            [0, 1, 0, vec[1]], 
            [np.sin(-theta), 0, np.cos(theta), vec[2]], 
            [0, 0, 0, 1]]
    )
def Tz(theta, vec):
    return np.matrix([  
            [np.cos(theta), np.sin(-theta), 0, vec[0]], 
            [np.sin(theta), np.cos(theta), 0, vec[1]], 
            [0, 0, 1, vec[2]], 
            [0, 0, 0, 1]]
    )


class Kinematics():
    def __init__(self, link_type, jointsAngle):
        # 各関節の同次変換行列（ID別）
        # protoファイルに記述されている情報からして、関節が持つ座標系は、全て共通しているわけではない。xyzもあれば、zxyもある。
        # しかし結局、出力は方向を持たない只の関節角度値だけである。
        # そのため今回は、全てxyzの座標系を持つと読み替えて、同次変換行列を作成する。
        # でも座標変換の有無が発生するから、それじゃあうまくいかない可能性？

        # 位置ベクトルはprotoファイルの方に合わせた。全関節の座標系（つまりxyz座標系）はURDFに合わせた。位置ベクトルはURDFのほうが細かく設定されている。

        # ShoulderR(axis y-1)
        theta01 = 0
        T01 = Ty(theta01, [0.0, -0.082, 0.0])

        # ShoulderL(axis y1)
        theta02 = 0
        T02 = Ty(theta02, [0.0, 0.082, 0.0])

        # ArmUpperR(axis x-1)
        theta03 = 0
        T03 = Tx(theta03, [0.0, 0.0, -0.016])

        # ArmUpperL(axis x-1)
        theta04 = 0
        T04 = Tx(theta04, [0.0, 0.0, -0.016])

        # ArmLowerR(axis y-1)
        theta05 = 0
        T05 = Ty(theta05, [0.016, 0.0, -0.06])

        # ArmLowerL(axis y1)
        theta06 = 0
        T06 = Ty(theta06, [0.016, 0.0, -0.06])

        # PelvYR(axis z-1)
        theta07 = 0
        r07 = [-0.005, -0.037, -0.1222]
        T07 = Tz(theta07, r07)

        # PelvYL(axis z-1)
        theta08 = 0
        T08 = Tz(theta08, [-0.005, 0.037, -0.1222])

        # PelvR(axis x-1)
        theta09 = 0
        r09 = [0.0, 0.0, 0.0]
        T09 = Tx(theta09, r09)

        # PelvL(axis x-1)
        theta10 = 0
        T10 = Tx(theta10, [0.0, 0.0, 0.0])

        # LegUpperR(axis y1)
        theta11 = 0
        r11 = [0.0, 0.0, 0.0]
        T11 = Ty(theta11, r11)

        # LegUpperL(axis y-1)
        theta12 = 0
        T12 = Ty(theta12, [0.0, 0.0, 0.0])

        # LegLowerR(axis y1)
        theta13 = 0
        r13 = [0.0, 0.0, -0.093]
        T13 = Ty(theta13, r13)

        # LegLowerL(axis y-1)
        theta14 = 0
        T14 = Ty(theta14, [0.0, 0.0, -0.093])

        # AnkleR(axis y-1)
        theta15 = 0
        r15 = [0.0, 0.0, -0.093]
        T15 = Ty(theta15, r15)

        # AnkleL(axis y1)
        theta16 = 0
        T16 = Ty(theta16, [0.0, 0.0, -0.093])

        # FootR(axis x1)
        theta17 = 0
        r17 = [0.0, 0.0, 0.0]
        T17 = Tx(theta17, r17)

        # FootL(axis x1)
        theta18 = 0
        T18 = Tx(theta18, [0.0, 0.0, 0.0])

        # Neck(axis z1)
        theta19 = 0
        T19 = Tz(theta19, [0.0, 0.0, 0.051])

        # Head(axis y-1)
        theta20 = 0
        T20 = Ty(theta20, [0.0, 0.0, 0.0])

        # 必ず両足計算しないといけないが、結局左右別々で計算するのだから、この場合、ROS2であることを活かして左右それぞれ別nodeとして用意してやれば、マルチタスクで両足を計算できるよ。

        # 目標リンクの位置p_ref、姿勢R_Ref
        p_ref = np.zeros(3)
        R_ref = np.zeros((3,3)) # 目標姿勢（＝目標回転行列）は、目標角度(roll, pitch, yaw)から計算してやれば良い。（ベタ踏みで歩行するなら目標姿勢は０のまま？）
        self.ref = [p_ref, R_ref]

        # 注目リンクの位置p_now、姿勢R_now
        p_now = np.zeros(3)
        R_now = np.zeros((3,3))
        self.cur = [p_now, R_now]

        # 位置の誤差p_diff、姿勢の誤差R_diff
        p_diff = np.zeros(3)
        R_diff = np.zeros((3,3))
        self.diff = [p_diff, R_diff]

        # 現在において、注目リンクまでの各関節角度を並べたベクトル（＝配列）q_now、関節角度修正量q_diff
        q_now = np.array(jointsAngle)
        q_diff = np.zeros(len(jointsAngle))
        self.q = [q_now, q_diff]

        # LR: LegRight, LL: LegLeft, AR: ArmRight, AL: ArmLeft
        if link_type == "LR":
            self.angles = [theta07, theta09, theta11, theta13, theta15, theta17]
            self.links = [T07, T09, T11, T13, T15, T17]
            self.r = [r07, r09, r11, r13, r15, r17]
            self.T = [Tz, Tx, Ty, Ty, Ty, Tx]
        elif link_type == "LL":
            self.angles = [theta08, theta10, theta12, theta14, theta16, theta18]
            self.links = [T08, T10, T12, T14, T16, T18]
        elif link_type == "AR":
            self.angles = [theta01, theta03, theta05]
            self.links = [T01, T03, T05]
        elif link_type == "AL":
            self.angles = [theta02, theta04, theta06]
            self.links = [T02, T04, T06]

        # 指定角度を代入
        for i in range(0, len(self.q[0])):
            self.angles[i] = self.q[0][i]


    def FK(self): # 順運動学
        # for i in range(0, len(self.q[0])):  # debug: 表示
        #     print(self.angles[i])
        #     print(self.links[i])

        for i in range(0, len(self.q[0])):  # 各関節の同次変換行列を現在の各関節角度で更新
            self.links[i] = self.T[i](self.angles[i], self.r[i])

        #  順運動学解の算出（各同次変換行列の内積から同次変換行列を算出）
        T_now = np.dot(self.links[0], np.dot(self.links[1], np.dot(self.links[2], np.dot(self.links[3], np.dot(self.links[4], self.links[5])))))

        # for i in range(0, len(self.q[0])):  # debug: 表示
        #     print(self.angles[i])
        #     print(self.links[i])

        # print(T_now)

        return T_now


    def IK(self): # 逆運動学
        print(self.links[2])

        return self.q[0]


def main():
    rad = 90*np.pi/180
    jointsAngle = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    z = Kinematics.FK(self=Kinematics(link_type="LR", jointsAngle=jointsAngle))
    print(jointsAngle, "\n 脚付け根: [-0.005, 0.037, -0.1222], 脚長さ: [0.0, 0.0, -0.186](足首〜足裏は無視) ")
    print("result: \n", z, "\n")

    jointsAngle = [0.0, 0.0, -rad, 0.0, 0.0, 0.0]
    z = Kinematics.FK(self=Kinematics(link_type="LR", jointsAngle=jointsAngle))
    print(jointsAngle, "\n 脚付け根: [-0.005, 0.037, -0.1222], 脚長さ: [0.0, 0.0, -0.186](足首〜足裏は無視) ")
    print("result: \n", z)
    

if __name__ == "__main__":
    main()