# <center>ROS入门教程代码示例</center>

## 示例介绍
本仓库为ROS入门教程的代码示例，包括以下ROS软件包:
* **robot_sim_demo**: 机器人仿真程序，大部分内容会用到这个软件包
* **topic_demo**: topic通信，自定义msg，分别以C++和python两种语言实现
* **service_demo**: service通信，自定义srv，分别以C++和python两种语言实现
* **param_demo**: param操作，分别以C++和python两种语言实现
* **msgs_demo**: msg与srv文件示例，演示msg与srv的格式规范
* **tf_demo**: TF和URDF的操作演示，分别以C++和python两种语言实现
* **urdf_demo**: 创建机器人urdf模型，在RViz中显示
* **navigation_sim_demo**: 导航演示工具包，包括AMCL, Odometry Navigation等演示
* **slam_sim_demo**: 同步定位与建图演示，包括Gmapping, Karto, Hector, Cartographer等SLAM演示
* **ros_academy_for_beginners**: Metapacakge，依赖了本仓库所有的pacakge

---
## 下载和编译

1. 克隆或下载ROS-Academy-for-Beginners教学包到工作空间的`/src`目录下，例如 `~/catkin_ws/src`
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/sychaichangkun/ROS-Academy-for-Beginners.git
```

2. 安装教学包所需的依赖
```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
```

3. 编译并刷新环境
```sh
$ catkin_make
$ source ~/catkin_ws/devel/setup.bash
```

4. 运行示例

---
## 运行须知
1. 运行robot\_sim\_demo前，确认Gazebo版本在**7.0以上**。

  查看Gazebo版本
  ```sh
  $ gazebo -v   #确认7.0以上
  ```
  如果低于7.0，请升级Gazebo
  ```sh
  $ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
  $ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
  $ sudo apt-get update
  $ sudo apt-get install gazebo7
  ```
2. 建议在**本地Ubuntu**下运行仿真程序。虚拟机对Gazebo的兼容性存在问题，可能会有错误或卡顿。

3. 确保所有依赖都已安装，如`gazebo_ros`, `gmapping`, `slam_karto`, `amcl`。



---
## Copyright
