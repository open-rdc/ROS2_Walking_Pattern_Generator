# ROS2_Walking_Pattern_Generator
Walking Pattern Generator using ROS2 for Humanoid Robots<br>
<br>
(Under Construction...)<br>
<br>

## 静歩行の実装 (**static_gait branch only**)
現在のVersionでは、静歩行動作をシミュレーションすることができます。Development eviromentに環境を合わせた後、以下のコマンド群を実行すればできます。<br>

```bash
mkdir -p ~/ros2_ws/src/
cd ~/ros2_ws/src/
git clone https://github.com/open-rdc/ROS2_Walking_Pattern_Generator.git
cd ..
colcon build --symlink-install
. install/setup.bash
ros2 launch webots_robot_handler start_launch.py
```

以下、静歩行シミュレーションの動画です。<br>

https://user-images.githubusercontent.com/91410662/231434693-c9bd6288-f6b9-442e-affd-260f978284c8.mp4


## Development environment
* OS: [Xubuntu 22.04](https://xubuntu.org/)<br>
* ROS 2 Distribution: [Humble](https://docs.ros.org/en/humble/index.html)<br>
* Simulator: [Webots R2023a](https://cyberbotics.com/)<br>
* required package: [webots_ros2](https://github.com/cyberbotics/webots_ros2)<br> 

## Configuration Plan (Draft)
* ROS_2_Package & Node<br>
  * robot_manager<br>
    * robot_manager node<br>
  * walking_pattern_generator<br>
    * walking_pattern_generator node<br>
  * walking_stabilization_controller<br>
    * walking_stabilization_controller node<br>
  * webots_robot_handler (webots_ros2 C++ plugin)<br>
    * ROBOTIS OP 2 node (plugin)<br>
  <!-- * kinematics<br>
    * FK_SrvServer node<br>
    * IK_SrvServer node<br> -->
<br>

![image](https://user-images.githubusercontent.com/91410662/234569468-f75ff588-d25a-49c7-9ceb-60174b0049f0.png)
<div align="center">Configulation Plan</div>
<br>

<!-- ![image](https://user-images.githubusercontent.com/91410662/228191771-cca5eb6a-7219-4a2e-819b-28e3249042ab.png)
<div align="center">rqt_graph</div>
<br>
<br> -->

## Robot Configulation
* Used Robot: ROBOTIS OP2 ([official](http://en.robotis.com/model/board.php?bo_table=print_en&wr_id=39)<br>
  * Webots simulation source data ([GitHub, cyberbotics, Webots, Darwin-op.proto](https://github.com/cyberbotics/webots/blob/master/projects/robots/robotis/darwin-op/protos/Darwin-op.proto
))<br>
  * Joint Status: [STATUS.txt](https://github.com/open-rdc/ROS2_Walking_Pattern_Generator/blob/main/STATUS.txt)
<br>

  * Webots User Guide ([ROBOTIS' Robotis OP2](https://cyberbotics.com/doc/guide/robotis-op2))
<br>
<br>

## Reference
・[Open Robotics, ROS 2 Documentation: Humble](https://docs.ros.org/en/humble/index.html)<br>
・[cyberbotics, Webots公式サイト](https://cyberbotics.com/)<br>
・[cyberbotics, Webots Reference Manual](https://cyberbotics.com/doc/reference/index)<br>
・[cyberbotics, Webots User Guide](https://cyberbotics.com/doc/guide/index)<br>
・[cyberbotics, Webots, GitHub](https://github.com/cyberbotics/webots)<br>
・[cyberbotics, webots_ros2, GitHub](https://github.com/cyberbotics/webots_ros2)<br>
・[cyberbotics, webots_ros2 Wiki, GitHub](https://github.com/cyberbotics/webots_ros2/wiki)<br>
・[@Nek, ROS2導入＆レクチャー, Qiita](https://qiita.com/NeK/items/7ac0f4ec10d51dbca084)<br>
　↑ 特にROS2のコードの記述の参考にさせていただいているサイト様
<br>
・[オーム社, 梶田秀司, 『ヒューマノイドロボット（改訂２版）』](https://www.ohmsha.co.jp/book/9784274226021/)<br>
　↑ 特に理論の参考にさせていただいている参考書
