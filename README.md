# <center>ROS入门教程代码示例</center>

## 示例介绍
本仓库为ROS入门教程的代码示例，包括以下ROS软件包:
* **robot_sim_demo**: 机器人仿真程序，大部分内容会用到这个软件包
* **topic_demo**: topic通信，自定义msg，分别以C++和python两种语言实现
* **service_demo**: service通信，自定义srv，分别以C++和python两种语言实现
* **param_demo**: param操作，分别以C++和python两种语言实现
* **tf_demo**: tf和urdf的操作演示，分别以C++和python两种语言实现
* **navigation_sim_demo**: TODO

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
1. 运行robot_sim_demo前，确认Gazebo版本在**7.0以上**。

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

---
## 课程介绍
本课程介绍ROS机器人操作系统的基本概念、原理和应用。教学内容包括视频、教材和演示代码。本仓库为ROS入门教程配套的软件包。

课程安排和知识点为：
1. ROS介绍与安装
  - 欢迎      
  - 什么是ROS
  - Xbot机器人演示
  - ROS安装和配置
2. 工程结构
  - catkin工作空间   
  - package
  - 操作演示
3. 通信架构（上）
  - master和node
  - 操作演示
  - topic和msg
  - 操作演示
4. 通信架构（下）
  - service和srv
  - parameter server
  - 操作演示
5. 常用工具
  - gazebo
  - rviz
  - rqt
6. 客户端库-roscpp
  - TODO
7. 客户端库-rospy
  - TODO
8. tf和urdf
  - TODO
9. 导航入门
  - TODO
10. 常见问题和小技巧
  - TODO

---
## Copyright
