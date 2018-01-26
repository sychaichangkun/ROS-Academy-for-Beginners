# navigation\_sim\_demo

导航演示功能包，本演示包包含以下内容：

* **amcl_demo**: map\_server+amcl 已知地图+自适应蒙特卡洛定位
* **odometry\_navigation\_demo**: 已知地图+仅用里程计（编码器）定位

## amcl示例运行方法

首先运行gazebo仿真场景

	roslaunch robot_sim_demo robot_spawn.launch

启动键盘控制程序

	rosrun robot_sim_demo robot_keyboard_teleop.py

再运行定位程序AMCL

	roslaunch navigation_sim_demo amcl_demo.launch

最后，启动rviz可视化工具

	roslaunch navigation_sim_demo view_navigation.launch

然后用键盘控制小车运动，就能在rviz上看到机器人定位的过程

## odometry navigation示例运行方法

	roslaunch robot_sim_demo robot_spawn.launch
	rosrun robot_sim_demo robot_keyboard_teleop.py
	roslaunch navigation_sim_demo odometry_localization_demo.launch
	roslaunch navigation_sim_demo view_navigation.launch


## 注意事项
如果gazebo出现错误，比如无法查看摄像头换面，你需要升级gazebo到gazebo7及以上版本:
```sh
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install gazebo7
```
