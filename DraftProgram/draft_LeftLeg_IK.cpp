#include "iostream"
#include "Eigen/Dense"

using namespace Eigen;
using namespace std;

int main() {
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

}

/*
参考
・https://qiita.com/yohm/items/a03006790dc1e54a87be
*/