#include "iostream"
#include "cmath"
#include "Eigen/Dense"

bool DEBUG = true;
#define PI 3.141592

using namespace Eigen;
using namespace std;


Matrix3d Rx(double theta) {
    Matrix3d R;
    R << 1, 0, 0,
         0, cos(theta), -sin(theta),
         0, sin(theta), cos(theta);
    return(R);
}
Matrix3d Ry(double theta) {
    Matrix3d R;
    R << cos(theta) , 0, sin(theta),
         0          , 1, 0         ,
         -sin(theta), 0, cos(theta);
    return(R);
}
Matrix3d Rz(double theta) {
    Matrix3d R;
    R << cos(theta), -sin(theta), 0,
         sin(theta), cos(theta) , 0,
         0         , 0          , 1;
    return(R);
}
Matrix3d IdentifyMatrix(void) {
    Matrix3d R;
    R << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;
    return(R);
}

double sign(double arg) {
    return((arg >= 0) - (arg < 0));
}


namespace Parameters {
    Matrix3d R_target;
    Vector3d P_target;

    array<Vector3d, 7> P;

    array<float, 6> Q;

    void set_Parameters(void) {
        // パラメータの正式な取得方法は、は他ファイルに記述してそれを読み込む、 他nodeから読み込む
        if(DEBUG == true) {
            Vector3d P_w1(-0.005, 0.037, -0.1222);
            Vector3d P_12(0, 0, 0);
            Vector3d P_23(0, 0, 0);
            Vector3d P_34(0, 0, -0.093);
            Vector3d P_45(0, 0, -0.093);
            Vector3d P_56(0, 0, 0);
            Vector3d P_6a(0, 0, 0);

            P = {P_w1, P_12, P_23, P_34, P_45, P_56, P_6a};
        }
        else if(DEBUG ==false) {
            // 正式な処理
        }
    }
}


namespace Kinematics {
    using namespace Parameters;

    Vector3d FK(void) {
        array<Matrix3d, 6> R;
        R = {Rz(Q[0]), Rx(Q[1]), Ry(Q[2]), Ry(Q[3]), Ry(Q[4]), Rz(Q[5])};

        return (
            R[0] * R[1] * R[2] * R[3] * R[4] * R[5] * P[6] 
            + R[0] * R[1] * R[2] * R[3] * R[4] * P[5]
            + R[0] * R[1] * R[2] * R[3] * P[4]
            + R[0] * R[1] * R[2] * P[3]
            + R[0] * R[1] * P[2]
            + R[0] * P[1]
            + P[0]
        );
    }

    void IK(void) {
        Vector3d P_16;
        P_16 = R_target.transpose() * (P[0] - P_target);

        double A, B, C;
        A = abs(P[3](2));
        B = abs(P[4](2));
        C = sqrt(pow(P_16(0), 2) + pow(P_16(1), 2) + pow(P_16(2), 2));

        Q[3] = -1 * acos((pow(A, 2) + pow(B, 2) - pow(C, 2)) / (2 * A * B)) + PI;
        
        double alfa;
        alfa = asin((A * sin(PI - Q[3])) / C);

        Q[4] = -1 * atan2(P_16(0), sign(P_16(2)) * sqrt(pow(P_16(1), 2) + pow(P_16(2), 2))) - alfa;
        Q[5] = atan2(P_16(1), P_16(2));

        Matrix3d R_w3;
        R_w3 = R_target * Rx(Q[5]).transpose() * Ry(Q[4]).transpose() * Ry(Q[3]).transpose();

        Q[0] = atan2(-R_w3(0, 1), R_w3(1, 1));
        Q[1] = atan2(R_w3(2, 1), -R_w3(0, 1) * sin(Q[0]) + R_w3(1, 1) * cos(Q[0]));
        Q[2] = atan2(-R_w3(2, 0), R_w3(2, 2));
    }
}


int main(void) {
    using namespace Parameters;

    // 最初に、パラメータ設定を行う
    bool SETUP;  // 他nodeからのフラグで決めたい
    if(DEBUG == true) {SETUP = true;}
    if(DEBUG == true || SETUP == true) {set_Parameters();}

    if(DEBUG == true) {
        Q = {0, 0, 0, 0, 0, 0};
        R_target = IdentifyMatrix();
        P_target << -0.005, 0.037, -0.1687;
    }
    else if(DEBUG ==false) {  // 他nodeからの情報を記録
        Q = {0, 0, 0, 0, 0, 0};
        R_target = IdentifyMatrix();
        P_target << -0.005, 0.037, -0.1687;
    }

    Kinematics::IK();
    Vector3d result_FK;
    result_FK = Kinematics::FK();

    cout << "Target Points[m]: " << P_target.transpose() << endl; 
    cout << "Target Rotation-Matrix: \n" << R_target << endl;
    cout << "Result IK: Joint-Angles[rad]: ";
    for(const auto &el : Q) {
        cout << el << ", ";
    }
    cout << endl;
    cout << "Result FK: Target Points[m]: " << result_FK.transpose() << endl;


    return(0);
}

/*
MEMO
    // MatrixXd mat(2, 2);  // 空の行列を、サイズ指定で作成 動的？
    Matrix2d A, B;  // 静的？ 上と同じ
    // mat(0, 0) = 1;
    // mat(0, 1) = 2;
    // mat(1, 0) = 3.5;
    // mat(1, 1) = mat(0, 0) + mat(0, 1);
    A << 1, 2,
         3, 4;
    B << 1, 0,
         0, 1;
    cout << A << endl;
    cout << B << endl;
    cout << A.transpose() << endl;
    cout << B.transpose() << endl;
    MatrixXd result_dot = A * B;
    cout << "dot A, B: \n" << result_dot << endl;
*/