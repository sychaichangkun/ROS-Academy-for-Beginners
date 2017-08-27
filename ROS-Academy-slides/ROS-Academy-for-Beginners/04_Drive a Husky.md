# 驱动一个Husky
---
# 驾驶一个Husky
## 更新虚拟机
打开终端窗口（Ctrl + Alt + T），然后输入以下内容：

`sudo apt-get update`                                                                                                  			
`sudo apt-get install ros-indigo-husky-desktop`
`sudo apt-get install ros-indigo-husky-simulator`

## 驱动一个虚拟的HUSKY
打开一个终端窗口，然后输入：

`roslaunch husky_gazebo husky_empty_world.launch`

打开另外一个新的终端然后输入：

`roslaunch husky_viz view_robot.launch`

你可以得到两个窗口，并且都有黄色的看起来很坚固的机器人（Husky!）

RViz

![](https://i.loli.net/2017/08/16/5993a75dc3f71.png)

Gazebo

![](https://i.loli.net/2017/08/16/5993a7a8e5047.png)

第二个显示的是Gazebo。这是我们得到我们的机器人的真实模拟，包括车轮打滑，打滑和惯性。我们可以添加对象到这个模拟，如上面的多维数据集，甚至是真实地方的整个地图。

第一个显示的是RViz。这个工具允许我们从机器人看到传感器数据，并向它发布命令（在将来的一篇文章中会详细介绍）。

我们现在可以命令机器人前进。打开一个终端窗口，并使用下面的命令，注意，复制粘贴下列语句无效！您可以通过在geometry_msgs / Twist之后按Tab键来完成此命令：

![](https://i.loli.net/2017/08/16/5993a88c2facd.png)

在上述命令中，我们发布到**/ husky_velocity_controller / cmd_vel topic**，主题类型为**geometry_msgs / Twist**。我们发布的数据告诉模拟的赫斯基以0.5米/秒的速度前进，没有任何旋转。你应该看到你的赫斯基向前移动。在Gazebo窗口中，您可能会注意到模拟车轮滑动和打滑。

## 使用RQT_GRAPH
我们还可以看到系统中传递主题的结构。离开发布窗口运行，并打开一个终端窗口。输入：

`rosrun  rqt_graph  rqt_graph`

此命令生成关于当前ROS主站上运行的节点和主题的相关性的表示。你应该得到类似于以下内容：

![](https://i.loli.net/2017/08/16/5993a936c8166.png)

突出显示的节点和箭头显示您要发布到模拟赫斯基的主题。然后，这个赫斯基继续更新Gazebo虚拟环境，该环境负责关节（车轮）的运动和机器人的物理变化。当您不确定谁在发布到ROS中时，rqt_graph命令非常方便。一旦找出您感兴趣的主题，您就可以使用**rostopic echo**来查看该主题的内容。

## 使用TF
在ROS中，tf是跟踪坐标系以及它们如何相互关联的特殊主题。所以，我们的模拟赫斯基从世界坐标框（0,0,0）开始。当赫斯基动作时，它自己的坐标框变化。每个车轮都有一个坐标框，跟踪它是如何旋转，以及它在哪里。通常，机器人上没有固定在空间中的任何东西，都会有一个描述它的tf。在**rqt_graph**部分，您可以看到**/tf topic**由许多不同的节点发布和订阅。

一个直观的方法来查看tf主题是如何构造的，是使用ROS提供的**view_frames**工具。打开一个终端窗口。输入：

`rosrun  tf  view_frames`

等待完成，然后键入：

`evince frames.pdf`

这将产生类似于以下图像的东西。

![](https://i.loli.net/2017/08/16/5993a9e4917d2.png)

这里我们可以看到，HUSKY的四个轮都被引用到**base_link**。我们也看到，**odom topic** 正是驱动整个机器人的参考。这意味着，如果你写了**odom topic**（即，当你发布到**/cmd_vel topic**），那么整个机器人将移动。

---
