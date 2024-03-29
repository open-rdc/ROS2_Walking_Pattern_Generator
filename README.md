> This README was created by translating the Japanese text using Google Translate.

# ROS2_Walking_Pattern_Generator
Walking Controller using ROS_2 for Humanoid Robots.
<br><br>
A walking control software for humanoid robots using ROS 2. We are proceeding with development with an emphasis on scalability, versatility, and simplicity, and we plan to be able to make humanoid robots walk without any advanced work.<br>
<br>

## Emvironment
It is compatible with the following OS:<br>
* Ubuntu 22.04 ([official](https://ubuntu.com/desktop))

Not tested on other OSes, but may work.<br>

## Dependencies
It depends on the following software:<br>
* ROS 2 Humble ([official](https://docs.ros.org/en/humble/index.html))
* Eigen 3.4 ([official](https://eigen.tuxfamily.org/index.php?title=Main_Page))
* Webots R2023b ([official](https://cyberbotics.com/))
* webots_ros2 ([official](https://github.com/cyberbotics/webots_ros2))

The version of this dependency is determined based on your development environment, so it may work with other versions.<br>

## Install

### Step1
  Please install dependencies.

  * Install ROS 2 -> [official document](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
  * Install Eigen ->  Installed together with ROS 2.
  * Install Webots -> [official document](https://cyberbotics.com/doc/guide/installation-procedure)
  * Install webots_ros2 -> [official document](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Installation-Ubuntu.html)

### Step2
  Please clone this repository.<br>

  Example:
```bash
mkdir -p ~/ros2_ws/src/ && cd ~/ros2_ws/src/
git clone https://github.com/open-rdc/ROS2_Walking_Pattern_Generator.git
```

### Step3
  Please build packages.<br>

  Example:
```bash
cd ~/ros2_ws/
colcon build --symlink-install
source install/setup.bash
```

## Usage
Run the program with the following command:
```bash
ros2 launch robot_bringup robot_bringup_launch.py
```

Example of operation by executing this command:

https://github.com/open-rdc/ROS2_Walking_Pattern_Generator/assets/91410662/f884f510-2d0a-4b5f-9acd-73ad64a8446d

<br>
<br>

## Software Architecture
![software structure](https://github.com/open-rdc/ROS2_Walking_Pattern_Generator/assets/91410662/f3dd77da-4cc7-4a26-92b7-a974c0c666ee)

<br>
<br>

## License & Author
The robot_description package contains parts of [ROBOTIS-GIT/ROBOTIS-OP2-Common](https://github.com/ROBOTIS-GIT/ROBOTIS-OP2-Common) (license: Apache-2.0 license). <br>
For details, see README.md in [robot_description/models/robotis_op2](https://github.com/open-rdc/ROS2_Walking_Pattern_Generator/tree/devel/robot_description/models/robotis_op2). <br>

This repository is licensed under [Apache License 2.0](https://opensource.org/license/apache-2-0/). <br>

Main developer & maintainer: Yusuke-Yamasaki-555 ([GitHub](https://github.com/Yusuke-Yamasaki-555), [X (twitter)](https://twitter.com/OdoOdomeme555), [blog (jp)](https://odome.hatenablog.com/)) <br>




<!-- ## Robot Configulation
* Used Robot: ROBOTIS OP2 ([official](http://en.robotis.com/model/board.php?bo_table=print_en&wr_id=39))<br>
  * Webots simulation source data ([GitHub, cyberbotics, Webots, Darwin-op.proto](https://github.com/cyberbotics/webots/blob/master/projects/robots/robotis/darwin-op/protos/Darwin-op.proto
))<br>
  * Joint Status: [STATUS.txt](https://github.com/open-rdc/ROS2_Walking_Pattern_Generator/blob/main/STATUS.txt)
<br>



  * Webots User Guide ([ROBOTIS' Robotis OP2](https://cyberbotics.com/doc/guide/robotis-op2))
<br>
<br> -->

<!-- ## Reference
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
　↑ 特に理論の参考にさせていただいている技術書 -->
