# ROS2_Walking_Pattern_Generator
Walking Pattern Generator using ROS2 for Humanoid Robots<br>
<br>
(Under Construction...)<br>
<br>

## Configuration Plan (Draft)
・ROS2_to_Webots_Plugin<br>
　・Get Robot(Sensors) Status (Publish Robot(Sensors) Status)
<br>
　・Set Robot(Sensors) Status (Subscribe Robot(Sensors) Status)<br>
・ROS2_Node<br>
　・Walking-Pattern-Generator<br>
　・Gait-Stabilization-Controller<br>
　・Calculator<br>
<br>

## Robot Configulation
・Used Robot: ROBOTIS OP2 ([official](https://e-shop.robotis.co.jp/product.php?id=14))<br>
　・Webots simulation source data ([GitHub, cyberbotics, Webots, Darwin-op.proto](https://github.com/cyberbotics/webots/blob/master/projects/robots/robotis/darwin-op/protos/Darwin-op.proto
))<br>
　　・Joint Status: [STATUS.txt](https://github.com/open-rdc/ROS2_Walking_Pattern_Generator/blob/main/STATUS.txt)
<br>
　・Webots User Guide ([ROBOTIS' Robotis OP2](https://cyberbotics.com/doc/guide/robotis-op2))
<br>
<br>

## Reference
・[Open Robotics, ROS 2 Documentation: Foxy](https://docs.ros.org/en/foxy/index.html)<br>
・[cyberbotics, Webots公式サイト](https://cyberbotics.com/)<br>
・[cyberbotics, Webots Reference Manual](https://cyberbotics.com/doc/reference/index)<br>
・[cyberbotics, Webots User Guide](https://cyberbotics.com/doc/guide/index)<br>
・[cyberbotics, Webots, GitHub](https://github.com/cyberbotics/webots)<br>
・[cyberbotics, webots_ros2, GitHub](https://github.com/cyberbotics/webots_ros2)<br>
・[cyberbotics, webots_ros2 Wiki, GitHub](https://github.com/cyberbotics/webots_ros2/wiki)<br>
・[@Nek, ROS2導入＆レクチャー, Qiita](https://qiita.com/NeK/items/7ac0f4ec10d51dbca084)<br>
　↑ 特にROS2のコードの記述の参考にさせていただいたサイト様
<br>
・[オーム社, 梶田秀司, 『ヒューマノイドロボット（改訂２版）』](https://www.ohmsha.co.jp/book/9784274226021/)<br>
　↑ 特に理論の参考にさせていただいた参考書
