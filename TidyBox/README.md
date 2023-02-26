# SORTING NOTES

## Software Configulation (draft)
![image](https://user-images.githubusercontent.com/91410662/221401013-759136a4-1a70-4bc1-ace8-0d8ee9ab73b3.png)
<br>

## Packages (draft)
* WebotsRobotHandler: <br>
  webots_ros2のCppPluginになるPackage.<br>
  * WebotsRobotHandler.hpp<br>
  * WebotsRobotHandler.cpp<br>
  * CMakelists.txt(WebotsRobotHandler.CMakelists)<br>
  * package.xml<br>
<br>

* WalkingStabilizationController: <br>
  歩行を安定化させるための制御系。歩行安定化制御系。<br>
  * WalkingStabilizationController.hpp<br>
  * WalkingStabilizationController.cpp<br>
  * CMakeLists.txt(WalkingStabilizationController.CMakeLists)<br>
  * package.xml<br>
<br>

* WalkingPatternGenerator<br>
  歩行パターンを生成して、関節角度などを示す。<br>
  * WalkingPatternGenerator.hpp<br>
  * WalkingPatternGenerator.cpp<br>
  * CMakeLists.txt(WalkingPatternGenerator.CMakeLists)<br>
  * package.xml<br>
<br>

* Kinematics<br>
  複数のnodeで共有する運動学を提供。頻繁には使われない、FK・IKの計算を提供。（頻繁に使われると、通信の遅延でリアルタイム性が駄目）<br>
  * FK.hpp: <br>
    * FK: 順運動学<br>
  * IK.hpp: <br>
    * IK: 逆運動学<br>
  * FK.cpp<br>
  * IK.cpp<br>
  * CMakeLists.txt(Kinematics.CMakeLists)<br>
  * package.xml<br>
<br>

* MsgsPackage: <br>
  messageファイルを定義するPackage.<br>
  * ToWebotsRobotHandler_msgs.msg: <br>
    * WebotsRobotHandlerとのService通信に使うmessage.<br>
  * ToWalkingStabilizationController_msgs.msg: <br>
    * WalkingStabilizationControllerとのTopic通信に使うmessage.<br>
  * ToKinematics_msgs.msg: <br>
    * KinematicsとのService通信に使うmessage.<br>
<br>

* (ParameterServer): <br>
  undecided<br>
  * (Robot.urdf): <br>
    * undecided<br>
  * (Robot.yaml): <br>
    * undecided<br>

