# robot_sim_demo

睿思学院ROS入门课程  robot_sim_demo功能包

涵盖package, node, launch, topic, msg, service, srv等知识点

![demo_pic](./worlds/ROS-Academy.png "ROS Academy World")

### 运行方法

确认`robot_sim_demo`已被编译, 执行以下命令启动仿真环境:
```sh
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch robot_sim_demo robot_spawn.launch
```

### 注意事项
如果gazebo出现错误，比如无法查看摄像头换面，你需要升级gazebo到gazebo7及以上版本:
```sh
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install gazebo7
```
