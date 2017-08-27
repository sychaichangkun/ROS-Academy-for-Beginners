# 驱动一个Grizzly
---
# 驾驶一个Grizzly
## 更新虚拟机
打开一个终端窗口（Ctrl+Alt+T），并且输入以下内容：

`sudo apt-get update`    
`sudo apt-get install ros-indigo-grizzly-simulator`     
`sudo apt-get install ros-indigo-grizzly-desktop`          
`sudo apt-get install ros-indigo-grizzly-navigation`

## 运行虚拟的GRIZZLY
打开一个终端窗口，然后输入：

`roslaunch grizzly_gazebo grizzly_empty_world.launch`

打开另一个终端窗口，然后输入：

`roslaunch grizzly_viz view_robot.launch`

你应该已经得到两个窗口，并且都有黄色的看起来很坚固的机器人（Grizzly！)下面显示的是Gazebo。这是我们得到我们的机器人的真实模拟，包括车轮打滑，打滑和惯性。我们可以添加对象到这个模拟，甚至是真实地方的整个地图。

![](https://i.loli.net/2017/08/16/5993b152a5e2f.png)

以下窗口为RViz。这个工具允许我们从机器人看到传感器数据，并向它发布命令（在将来的一篇文章中将详细介绍）。RViz是一个更简化的模拟，速度更快。

![](https://i.loli.net/2017/08/16/5993b1b4279fe.png)

你可以命令机器人向前进。只需要打开终端，然后复制一下内容：

![](https://i.loli.net/2017/08/16/5993b231199ff.png)

在上面的命令中，我们发布到cmd_vel topic，主题类型为**geometry_msgs /Twist**。我们发布的数据告诉模拟的灰熊（Grizzly）以0.5米/秒的速度前进，没有任何旋转。你应该看到你的灰熊前进。在Gazebo窗口中，您还可能会注意到模拟车轮滑动和打滑。

---
