import numpy as np

class Kinematics():
    def __init(self, kinematics, target, data):
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
        LR_p = []
        LR_R = [[], [], []]
        LR = [LR_p, LR_R]
        LL_p = []
        LL_R = [[], [], []]
        LL = [LL_p, LL_R]
        AR_p = []
        AR_R = [[], [], []]
        AR = [AR_p, AR_R]
        AL_p = []
        AL_R = [[], [], []]
        AL = [AL_p, AL_R]
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
        LR_q_now = []
        LR_q_diff = []
        LR_q = [LR_q_now, LR_q_diff]
        LL_q_now = []
        LL_q_diff = []
        LL_q = [LL_q_now, LL_q_diff]
        AR_q_now = []
        AR_q_diff = []
        AR_q = [AR_q_now, AR_q_diff]
        AL_q_now = []
        AL_q_diff = []
        AL_q = [AL_q_now, AL_q_diff]

        if kinematics == "FK":
            # targetのq_now = data
            Kinematics.FK(target)
        elif kinematics == "IK":
            # targetのq_now = data
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