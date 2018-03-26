# 运行ORB_SLAM2

本软件包已经设置好了相机参数和launch文件，在编译好ORB_SLAM2的前提下可结合仿真程序直接运行。

1. 安装编译DBoW2、g2o、Eigen、OpenCV、Pangolin等orb_slam2所需的依赖
2. 编译构建orb_slam2以及**相应ROS包**
3. 启动仿真程序和robot_orbslam2_demo

```sh
$ roslaunch robot_sim_demo robot_spawn.launch  #启动仿真
$ roslaunch robot_orbslam2_demo ros_orbslam2.launch #启动ORB SLAM的ROS程序
$ rosrun robot_sim_demo robot_keyboard_teleop.py #移动机器人，开始建图
```

**Tips:**　具体步骤参考－＞运行包详细步骤.md
