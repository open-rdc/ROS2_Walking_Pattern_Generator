#include "iostream"
#include "Eigen/Dense"


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


int main() {
    cout << Rx(0) << endl;
    cout << Ry(0) << endl;
    cout << Rz(0) << endl;
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