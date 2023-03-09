# req: WSC, WPJ <=> res: Kinematics

# "static array" = "std::array<T, N>" (T: 有効な型, N: 要素数), R_target -> std::array<double, 6> (Eigen::Matrix3dの要素をこの型に移し替える必要がある)

char Arm_or_Leg  # char "A" or "L"
double[6] R_target_R  # std::array<double, 6> <= Eigen::Matrix3d
double[3] P_target_R  # std::array<double, 3> <= Eigen::Vector3d
double[6] Q_target_R  # std::array<double, 6>, [rad]
double[6] R_target_L  # std::array<double, 6> <= Eigen::Matrix3d
double[3] P_target_L  # std::array<double, 3> <= Eigen::Vector3d
double[6] Q_target_l  # std::array<double, 6>, [rad]
---
double[3] P_result_R  # IK_result, std::array<double, 3> => Eigen::Vector3d
double[6] Q_result_R  # FK_result, std::array<double, 6>, [rad]
double[3] P_result_L  # IK_result, std::array<double, 3> => Eigen::Vector3d
double[6] Q_result_L  # FK_result, std::array<double, 6>, [rad]


# R_target
# R -> Rotarion-Matrixes
# P_target
# P -> Joint-Points
# rad -> Joint-Rotation-Angles

# RかLかの判断基準
# LegかArmかの判断基準

# R_targetとP_targetは、IKで必須
# 関節角度[rad]は、FKで必須
# RとPは、Parameterから読み込みたい。
# で、四肢すべてをKinematicsの方で保持しておきたい。四肢のRとPはFK,IK以外の一箇所でParameterServerから読み込んで、それをIncludeする形のほうが良いのではないか？それかSetup関数でも作っておいて、最初だけ（ライフサイクルなりフラグなりで判断？）それを読み込んで各nodeに定義しておくとか？