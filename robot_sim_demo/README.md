# robot_sim_demo

robot_sim_demo软件包，包括Xbot机器人仿真程序，在Gazebo中运行，涵盖package, node, topic, msg, service, srv, param等知识点。

### 运行方法

确认`robot_sim_demo`已被编译, 执行以下命令启动仿真环境
```sh
$ roslaunch robot_sim_demo robot_spawn.launch
```
启动速度控制程序
```sh
$ rosrun robot_sim_demo robot_keyboard_teleop.py
```

### 注意事项
确认升级gazebo到**7.0及以上版本**
```sh
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install gazebo7
```


如果你是用的是VMware虚拟机，运行Gazebo可能会出现`vmw_ioctl_command`错误，解决办法：
```sh
$ export SVGA_VGPU10=0
```
也可以将其追加到.bashrc文件中
```sh
$ echo “export SVGA_VGPU10=0” >> ~/.bashrc
```
