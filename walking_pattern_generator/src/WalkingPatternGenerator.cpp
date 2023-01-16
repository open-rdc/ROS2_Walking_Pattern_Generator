#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/string.hpp"

#include "walking_pattern_generator/WalkingPatternGenerator.hpp"
#include "webots/Node.hpp"
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>

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


// webots関連の関数を使いやすくするために、using namepaceで省略できるようにしたほうが良いかも。

namespace walking_pattern_generator 
{
    using namespace Parameters;
    
    void WalkingPatternGenerator::init (
        webots_ros2_driver::WebotsNode *node, 
        std::unordered_map<std::string, std::string> &parameters
    ) {
        std::vector<std::string> MotorsName = {"ShoulderR", "ShoulderL", "ArmUpperR", "ArmUpperL", "ArmLowerR", "ArmLowerL", "PelvYR", "PelvYL", "PelvR", "PelvL", "LegUpperR", "LegUpperL", "LegLowerR", "LegLowerL", "AnkleR", "AnkleL", "FootR", "FootL", "Neck", "Head"};

        // debug
        if(MotorsName[0] != "ShoulderR"){ 
            std::cout << "ERROR!: MotorsName is fucking array!!" << std::endl; 
        }
        std::string hoge = MotorsName[0] + "S";
        if( hoge != "ShoulderRS"){
            std::cout << "ERROR!: MotorsName is fucking array.push_back()!!" << std::endl; 
        }

        // NODE
        Node = node;
        robot = Node->robot();

        for(int i = 0; i <20; i++) {
            motor[i] = robot->getMotor(MotorsName[i]);
            positionSensor[i] = robot->getPositionSensor(MotorsName[i]+"S");  // value type: double
            positionSensor[i]->webots::PositionSensor::enable(100);  // sampling period: 100[ms]. この宣言の直後に値は得られない。1周期後（ここでは100[ms）後に１つ目の値が得られる。
        }
        accelerometer = robot->getAccelerometer("Accelerometer");  // values type: double* (need check document)
        gyro = robot->getGyro("Gyro");  // values type: double* (need check document)

        accelerometer->webots::Accelerometer::enable(100);  // sampling period: 100[ms]
        gyro->webots::Gyro::enable(100);  // sampling period: 100[ms]

        RCLCPP_INFO(Node->get_logger(), "Hello my mind...");

        // __pub = node->create_publisher<std_msgs::msg::String>("test", rclcpp::QoS(10));
        
        Q = {0, 0, 0, 0, 0, 0};
        R_target = IdentifyMatrix();
        P_target << -0.005, 0.037, -0.1687;

        // Supervisor
        // supervisor = Node->robot();
        // SupervisorNode = supervisor->getFromDef("ROBOTIS_OP2");
        // if (SupervisorNode == NULL) {
        //     std::cout << "ERROR!: getFromDef == NULL" << std::endl;
        //     while(true){ 
                 
        //     }
        // }
        // field_translation = SupervisorNode->getField("translation");
        // field_rotation = SupervisorNode->getField("rotation");
    }



    void WalkingPatternGenerator::step() {
        // NODE: CHECK SENSOR DATA

        for(int i = 0; i < 20; i++){
            positionSensorValue[i] = float(positionSensor[i]->webots::PositionSensor::getValue());  // 毎stepで値を再取得しないと、値が更新されない。
        }
        accelerometerValue = accelerometer->webots::Accelerometer::getValues();  // 毎stepで値を再取得せずとも、値は更新される。値を保持するためには、他変数にコピーする必要がある。
        gyroValue = gyro->webots::Gyro::getValues();  // 加速度センサ値と同様。

        // 最初、Tポーズ
        if(first_step){
            first_step = false;
            for(int i = 0; i < 20; i++){
                motorValue[i] = 0;  // [rad]

                if( (i == 2) or (i == 3) ) {  // 両肩（2: R, 3: L）
                    if(i == 2) motorValue[i] = 0.79;  // 0.79[rad] == 45[deg]
                    if(i == 3) motorValue[i] = -0.79;
                    motor[i]->webots::Motor::setVelocity(0.5);
                    motor[i]->webots::Motor::setPosition(double(motorValue[i]));  // 単位: [rad]
                    continue;
                }
                else if( (i == 4) or (i == 5) ) {  // 両肘（4: R, 5: L）
                    if(i == 4) motorValue[i] = -1.57;  // 1.57[rad] == 90[deg]
                    if(i == 5) motorValue[i] = 1.57;
                    motor[i]->webots::Motor::setVelocity(0.5);
                    motor[i]->webots::Motor::setPosition(double(motorValue[i]));  // 単位: [rad]
                    continue;
                }
                else if( (i == 10) or (i == 11) or (i == 14) or (i == 15) ) {  // 立たせるための角速度調整（両脚付け根Y軸, 両足首Y軸）
                    motorValue[i] = 0;  // [rad]
                    motor[i]->webots::Motor::setVelocity(0.25);  // [rad/s]
                    motor[i]->webots::Motor::setPosition(double(motorValue[i]));  // 単位: [rad]
                    continue;
                }
                else if(i == 19) {  // まっすぐ前を向かせる調整（Head）
                    motorValue[i] = 0.26;  // 0.26[rad] == 15[deg]
                    motor[i]->webots::Motor::setVelocity(0.5);
                    motor[i]->webots::Motor::setPosition(double(motorValue[i]));  // 単位: [rad]
                    continue;
                }
                else{  // 他、全関節
                    motorValue[i] = 0;  // [rad]
                    motor[i]->webots::Motor::setVelocity(0.5);
                    motor[i]->webots::Motor::setPosition(double(motorValue[i]));  // 単位: [rad]
                    continue;
                }
            }
        }

        // for(int i = 0; i < 20; i++){
        //     std::cout << positionSensorValue[i] << ", " ;
        // }
        // std::cout << std::endl;

        Kinematics::IK();  // Q更新
        motor[7]->webots::Motor::setPosition(Q[0]);
        motor[9]->webots::Motor::setPosition(Q[1]);
        motor[11]->webots::Motor::setPosition(Q[2]);
        motor[13]->webots::Motor::setPosition(Q[3]);
        motor[15]->webots::Motor::setPosition(Q[4]);
        motor[17]->webots::Motor::setPosition(Q[5]);
        // 7, 9, 11, 13, 15, 17

        // RCLCPP_INFO(Node->get_logger(), "acc: [x: %F, y: %F, z: %F], gyro: [x: %F, y: %F, z: %F] ", 
        //     accelerometerValue[0], accelerometerValue[1], accelerometerValue[2],gyroValue[0], gyroValue[1], gyroValue[2]);

        // RCLCPP_INFO(Node->get_logger(), "pos: %F, acc: [x: %F, y: %F, z: %F], gyro: [x: %F, y: %F, z: %F] ", 
        //     positionSensorValue[0], accelerometerValue[0], accelerometerValue[1], accelerometerValue[2],gyroValue[0], gyroValue[1], gyroValue[2]);
        
        // SUPERVISOR: CHECK DATA
        // translation = field_translation->getSFVec3f();  // ロボットが持つ、自身の位置ベクトル[x[m], y[m], z[m]]
        // rotation = field_rotation->getSFRotation();  // ロボットが持つ、自身の座標系各軸の回転角度[x[rad], y[rad], z[rad]]

        // position = SupervisorNode->getPosition();  // ワールド座標系から見た、ロボットのローカル座標系の位置ベクトル[x[m], y[m], z[m]]translation, rotationともに、値は同一
        // orientation = SupervisorNode->getOrientation();  // ワールド座標系から見た、ロボットのローカル座標系との回転行列[[x,y,z], [x,y,z], [x,y,z]]
        // pose = SupervisorNode->getPose();  // ワールド座標系とロボット（ローカル座標系）間の同次変換行列
        // centerOfMass = SupervisorNode->getCenterOfMass();  // ロボットの重心
        // velocity = SupervisorNode->getVelocity();  // ワールド座標系で表現された、ロボットの速度、角速度
        // int size;
        // contactPoint = SupervisorNode->getContactPoints(true, &size);

        // RCLCPP_INFO(  // 位置ベクトル: 各関数での数値の比較（結果: 同値（ワールド座標系からロボットのローカル座標系への位置ベクトル））
        //     Node->get_logger(), "\n      position: [%F, %F, %F],"
        //                         "\n   translation: [%F, %F, %F],"
        //                         "\n  pose[3,7,11]: [%F, %F, %F]"
        //                         , position[0], position[1], position[2]
        //                         , translation[0], translation[1], translation[2]
        //                         , pose[3], pose[7], pose[11]
        // );
        // RCLCPP_INFO(  // 同次変換行列(4*4)
        //     Node->get_logger(), "\n  pose: [[%F, %F, %F, %F],"
        //                         "\n         [%F, %F, %F, %F]," 
        //                         "\n         [%F, %F, %F, %F]," 
        //                         "\n         [%F, %F, %F, %F]]"
        //                         , pose[0], pose[1], pose[2], pose[3]
        //                         , pose[4], pose[5], pose[6], pose[7]
        //                         , pose[8], pose[9], pose[10], pose[11]
        //                         , pose[12], pose[13], pose[14], pose[15]
        // );
        // RCLCPP_INFO(// 回転行列: 各関数での数値の比較（結果: 同値（ワールド座標系からロボットのローカル座標系への回転行列））
        //     Node->get_logger(), "\n                   orientation: [[%F, %F, %F], [%F, %F, %F], [%F, %F, %F]],"
        //                         "\n  pose[[0,1,2][4,5,6][8,9,10]]: [[%F, %F, %F], [%F, %F, %F], [%F, %F, %F]]"
        //                         , orientation[0], orientation[1], orientation[2], orientation[3], orientation[4], orientation[5], orientation[6], orientation[7], orientation[8]
        //                         , pose[0], pose[1], pose[2], pose[4], pose[5], pose[6], pose[8], pose[9], pose[10]
        // );
        // RCLCPP_INFO(  // CHECK: Rotation, Velocity, CenterOfMass
        //     Node->get_logger(), "\n      rotation: [%F, %F, %F]"
        //                         "\n      velocity: [[%F, %F, %F], [%F, %F, %F]]"
        //                         "\n  CenterOfMass: [%F, %F, %F]"
        //                         , rotation[0], rotation[1], rotation[2]
        //                         , velocity[0], velocity[1], velocity[2], velocity[3], velocity[4], velocity[5]
        //                         , centerOfMass[0], centerOfMass[1], centerOfMass[2]
        // );
        // CHECK: ContactPoints
        // RCLCPP_INFO(Node->get_logger(), "%d", size);
        // for(int i = 0, i <= size, i++){
        //     if(i == size){
        //         std::cout << contactPoint[i] << std::endl;
        //     }
        //     else {
        //         std::cout << contactPoint[i];
        //     }
        // }

        // 以上のstep周期: 約 25[ms]
        // データを100[ms]で取得しており、標準出力でセンサ値が４回の出力記録の周期で変化しているため。

        // PUBLISH
        // auto datamsg = std::make_shared<std_msgs::msg::String>();
        // static int count = 0;
        // datamsg->data = "Hello: " + std::to_string(count++);
        // __pub->publish(*datamsg);

        // TEST
        // double hage;
        // std::cout << "Ride On!" << std::endl;
        // hage = positionSensor[0]->webots::PositionSensor::getValue();
        // RCLCPP_INFO(Node->get_logger(), "hogehoge: %lf", hage);
        // sleep(1.0);  
        
        // sleepすると、webotsのシミュレーション時間も止まる。止まるというより、時間の進みが*0.08とかになる。サンプリングも止まるから、注意。
        // サンプリング周期を設定して、その周期でstepを進めてやれば、リアルタイム性のある制御となる。
            // 計算時間が確保できないなら、その間シミュレートを止めることもできる。<ーはじめはコレでいきたい。
        // rosbagも有り

        // ココからロボットのデータを逐次Publishをしたい。
        // 他nodeから計算結果などもsubscribeしたい。
            // 別に新規でpluginを作成し、PubとSubを分けたほうが良さげ？
    }
}

PLUGINLIB_EXPORT_CLASS (
    walking_pattern_generator::WalkingPatternGenerator, webots_ros2_driver::PluginInterface
)
