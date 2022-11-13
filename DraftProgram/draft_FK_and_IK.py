import numpy as np

class Kinematics():
    def __init__(self, kinematics, link, jointsAngle, reference):
        # 各関節の同次変換行列（ID別）
        # protoファイルに記述されている情報からして、関節が持つ座標系は、全て共通しているわけではない。xyzもあれば、zxyもある。
        # しかし結局、出力は方向を持たない只の関節角度値だけである。
        # そのため今回は、全てxyzの座標系を持つと読み替えて、同次変換行列を作成する。
        id01 = np.matrix([
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 1]]
        )
        id02 = np.matrix([
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 1]]
        )
        id03 = np.matrix([
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 1]]
        )
        id04 = np.matrix([
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 1]]
        )
        id05 = np.matrix([
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 1]]
        )
        id06 = np.matrix([
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 1]]
        )
        id07 = np.matrix([
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 1]]
        )
        id08 = np.matrix([
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 1]]
        )
        id09 = np.matrix([
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 1]]
        )
        id10 = np.matrix([
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 1]]
        )
        id11 = np.matrix([
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 1]]
        )
        id12 = np.matrix([
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 1]]
        )
        id13 = np.matrix([
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 1]]
        )
        id14 =np.matrix([
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 1]]
        )
        id15 = np.matrix([
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 1]]
        )
        id16 = np.matrix([
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 1]]
        )
        id17 = np.matrix([
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 1]]
        )
        id18 = np.matrix([
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 1]]
        )
        id19 = np.matrix([
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 1]]
        )
        id20 = np.matrix([
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 0], 
            [0, 0, 0, 1]]
        )

        # LR: LegRight, LL: LegLeft, AR: ArmRight, AL: ArmLeft
        self.LR_links = [id07, id09, id11, id13, id15, id17]
        self.LL_links = [id08, id10, id12, id14, id16, id18]
        self.AR_links = [id01, id03, id05]
        self.AL_links = [id02, id04, id06]

        # ここで、全Linkに対して変数を用意しているが、４セットもいらず最低１セットあれば良い。てか１セットのほうが良い。歩行では必ず両足計算しないといけないが、結局左右別々で計算するのだから、この場合、ROS2であることを活かして左右それぞれ別nodeとして用意してやれば、マルチタスクで両足を計算できるよ。

        # 目標リンクの位置p_ref、姿勢R_Ref
        p_ref = np.zeros(3)
        R_ref = np.zeros((3,3)) # 目標姿勢（＝目標回転行列）は、目標角度(roll, pitch, yaw)から計算してやれば良い。（ベタ踏みで歩行するなら目標姿勢は０のまま？）
        self.ref = np.matrix([p_ref, R_ref])

        # 注目リンクの位置p_cur、姿勢R_cur
        p_cur = np.zeros(3)
        R_cur = np.zeros((3,3))
        self.cur = np.matrix([p_cur, R_cur])

        # 位置の誤差p_diff、姿勢の誤差R_diff
        p_diff = np.zeros(3)
        R_diff = np.zeros((3,3))
        self.diff = np.matrix([p_diff, R_diff])

        # 現在において、注目リンクまでの各関節角度を並べたベクトル（＝配列）q_cur、関節角度修正量q_diff
        q_cur = np.array(jointsAngle)
        q_diff = np.zeros(len(jointsAngle))
        q = np.matrix([q_cur, q_diff])

        if kinematics == "FK": # 順運動学
            Kinematics.FK(link)

        elif kinematics == "IK": # 逆運動学
            Kinematics.IK(link)
            

    def FK(self, link): # 順運動学
        if link == "LR":
            links = self.LR_links
        elif link == "LL":
            links = self.LL_links
        elif link == "AR":
            links = self.AR_links
        elif link == "AL":
            links = self.AL_links

        return self.q[0]


    def IK(self, link): # 逆運動学
        if link == "LR":
            links = self.LR_links
        elif link == "LL":
            links = self.LL_links
        elif link == "AR":
            links = self.AR_links
        elif link == "AL":
            links = self.AL_links

        return self.q[0]


def main():
    return 0

if __name__ == "__main__":
    main()