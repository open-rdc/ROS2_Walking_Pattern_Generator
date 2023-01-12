#include "iostream"
#include "Eigen/Dense"

using Eigen::MatrixXd;

int main() {
    MatrixXd mat(2, 2);  // 空の行列を、サイズ指定で作成
    mat(0, 0) = 1;
    mat(0, 1) = 2;
    mat(1, 0) = 3;
    mat(1, 1) = mat(0, 0) + mat(0, 1);
    std::cout << mat << std::endl;
}

/*
参考
・https://qiita.com/yohm/items/a03006790dc1e54a87be
*/