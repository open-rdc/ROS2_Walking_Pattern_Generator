import numpy as np

class Kinematics():
    id1, id2, id3, id4, id5, id6, id7, id8, id9, id10, id11, id12, id13, id14, id15, id16, id17, id18, id19, id20 = None
    LR_links, LL_links, AR_links, AL_links = None
    
    def __init(self, kinematics, target, data):
        # 各関節の同次変換行列（ID別）
        id1 = [
            [], 
            [], 
            [], 
            [0, 0, 0, 1]]
        id2 = [
            [], 
            [], 
            [], 
            [0, 0, 0, 1]]
        id3 = [
            [], 
            [], 
            [], 
            [0, 0, 0, 1]]
        id4 = [
            [], 
            [], 
            [], 
            [0, 0, 0, 1]]
        id5 = [
            [], 
            [], 
            [], 
            [0, 0, 0, 1]]
        id6 = [
            [], 
            [], 
            [], 
            [0, 0, 0, 1]]
        id7 = [
            [], 
            [], 
            [], 
            [0, 0, 0, 1]]
        id8 = [
            [], 
            [], 
            [], 
            [0, 0, 0, 1]]
        id9 = [
            [], 
            [], 
            [], 
            [0, 0, 0, 1]]
        id10 = [
            [], 
            [], 
            [], 
            [0, 0, 0, 1]]
        id11 = [
            [], 
            [], 
            [], 
            [0, 0, 0, 1]]
        id12 = [
            [], 
            [], 
            [], 
            [0, 0, 0, 1]]
        id13 = [
            [], 
            [], 
            [], 
            [0, 0, 0, 1]]
        id14 = [
            [], 
            [], 
            [], 
            [0, 0, 0, 1]]
        id15 = [ 
            [], 
            [], 
            [], 
            [0, 0, 0, 1]]
        id16 = [
            [], 
            [], 
            [], 
            [0, 0, 0, 1]]
        id17 = [
            [], 
            [], 
            [], 
            [0, 0, 0, 1]]
        id18 = [
            [], 
            [], 
            [], 
            [0, 0, 0, 1]]
        id19 = [
            [], 
            [], 
            [], 
            [0, 0, 0, 1]] # Neck
        id20 = [
            [], 
            [], 
            [], 
            [0, 0, 0, 1]] # Head

        LR_links = [id7, id9, id11, id13, id15, id17]
        LL_links = [id8, id10, id12, id14, id16, id18]
        AR_links = [id1, id3, id5]
        AL_links = [id2, id4, id6]

        # ここで、全Linkに対して変数を用意しているが、４セットもいらず最低１セットあれば良い。てか１セットのほうが良い。両足計算しないといけないが、その場合、ROS2であることを活かして左右それぞれ別nodeとして用意してやれば、マルチタスクで両足を計算できるよ。

        # LR: LegRight, LL: LegLeft, AR: ArmRight, AL: ArmLeft
        # 目標リンクの位置p_ref、姿勢R_Ref
        LR_p_ref = []
        LR_R_ref = [[], [], []]
        LR_ref = [LR_p_ref, LR_R_ref]
        LL_p_ref = []
        LL_R_ref = [[], [], []]
        LL_ref = [LL_p_ref, LL_R_ref]
        AR_p_ref = []
        AR_R_ref = [[], [], []]
        AR_ref = [AR_p_ref, AR_R_ref]
        AL_p_ref = []
        AL_R_ref = [[], [], []]
        AL_ref = [AL_p_ref, AL_R_ref]

        # 注目リンクの位置p、姿勢R
        LR_p_cur = []
        LR_R_cur = [[], [], []]
        LR_cur = [LR_p_cur, LR_R_cur]
        LL_p_cur = []
        LL_R_cur = [[], [], []]
        LL_cur = [LL_p_cur, LL_R_cur]
        AR_p_cur = []
        AR_R_cur = [[], [], []]
        AR_cur = [AR_p_cur, AR_R_cur]
        AL_p_cur = []
        AL_R_cur = [[], [], []]
        AL_cur = [AL_p_cur, AL_R_cur]

        # 位置の誤差p_diff、姿勢の誤差R_diff
        LR_p_diff = []
        LR_R_diff = [[], [], []]
        LR_diff = [LR_p_diff, LR_R_diff]
        LL_p_diff = []
        LL_R_diff = [[], [], []]
        LL_diff = [LL_p_diff, LL_R_diff]
        AR_p_diff = []
        AR_R_diff = [[], [], []]
        AR_diff = [AR_p_diff, AR_R_diff]
        AL_p_diff = []
        AL_R_diff = [[], [], []]
        AL_diff = [AL_p_diff, AL_R_diff]

        # 現在において、注目リンクまでの各関節角度を並べたベクトル（＝配列）q、関節角度修正量q_diff
        LR_q_cur = []
        LR_q_diff = []
        LR_q = [LR_q_cur, LR_q_diff]
        LL_q_cur = []
        LL_q_diff = []
        LL_q = [LL_q_cur, LL_q_diff]
        AR_q_cur = []
        AR_q_diff = []
        AR_q = [AR_q_cur, AR_q_diff]
        AL_q_cur = []
        AL_q_diff = []
        AL_q = [AL_q_cur, AL_q_diff]

    # def kinematics(self, kinematics, target, data):
        if kinematics == "FK":
            # targetのq_cur = data
            if target == "LR":
                LR_q[0] = data
                print(LR_q)
            elif target == "LL":
                LL_q[0] = data
                print(LL_q)
            elif target == "AR":
                AR_q[0] = data
                print(AR_q)
            elif target == "AL":
                AL_q[0] = data
                print(AL_q)
            Kinematics.FK(target)

        elif kinematics == "IK":
            # targetのq_cur = data
            if target == "LR":
                LR_q[0] = data
                LR_q[1] = np.zero(len(data), dtype="float32")
                print(LR_q)
            elif target == "LL":
                LL_q[0] = data
                LL_q[1] = np.zero(len(data), dtype="float32")
                print(LL_q)
            elif target == "AR":
                AR_q[0] = data
                AR_q[1] = np.zero(len(data), dtype="float32")
                print(AR_q)
            elif target == "AL":
                AL_q[0] = data
                AL_q[1] = np.zero(len(data), dtype="float32")
                print(AL_q)

            # targetのq_diff = np.zero(dataと同じ要素数の、全要素が０のベクトルを代入)
            Kinematics.IK(target)
            
    def FK(self, target): # 順運動学
        pass

    def IK(self, target): # 逆運動学
        pass


def main():
    return 0

if __name__ == "__main__":
    main()