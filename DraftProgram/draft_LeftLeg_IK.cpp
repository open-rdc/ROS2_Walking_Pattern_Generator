#include "iostream"
#include "cmath"
#include "Eigen/Dense"

bool DEBUG = true;

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


namespace Parameters {
    Matrix3d R_target;
    Vector3d P_target;

    array<Vector3d, 6> P;

    array<float, 6> Q;
}
void set_Parameters() {
    using namespace Parameters;

    array<Vector3d, 6> P_list;
    
    // パラメータの正式な取得方法は、は他ファイルに記述してそれを読み込む、 他nodeから読み込む
    if(DEBUG == true) {
        R_target = IdentifyMatrix();
        P_target << -0.005, 0.037, -0.1687;

        Vector3d P_w1(-0.005, 0.037, -0.1222);
        Vector3d P_12(0, 0, 0);
        Vector3d P_23(0, 0, 0);
        Vector3d P_34(0, 0, -0.093);
        Vector3d P_45(0, 0, -0.093);
        Vector3d P_56(0, 0, 0);

        P_list = {P_w1, P_12, P_23, P_34, P_45, P_56};
    }
    else if(DEBUG ==false) {
        // 正式な処理
    }

    for(int i = 0; i < P.size(); i++) {
        P[i] = P_list[i];  // ココ、わざわざfor loopにする必要はない。まんま代入でイケるはず
    }
}


int main() {
    using namespace Parameters;

    bool SETUP;  // 他nodeからのフラグで決めたい
    if(DEBUG == true) {SETUP = true;}
    if(DEBUG == true || SETUP == true) {set_Parameters();}

    array<float, 6> Q_list;
    if(DEBUG == true) {Q_list = {0, 0, 0, 0, 0, 0};}
    for(int i = 0; i < Q.size(); i++) {
        Q[i] = Q_list[i];  // ココ、わざわざfor loopにする必要はない。まんま代入でイケるはず
    }

    // cout << Rx(3.14) << endl;
    // cout << Ry(3.14) << endl;
    // cout << Rz(3.14) << endl;

    cout << R_target << endl;
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