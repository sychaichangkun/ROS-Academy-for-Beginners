## ROS导航基础

# ROS导航基础

如果您已经使用ROS和机器人技术，您可能已经听说过gmaping，本地化，SLAM，costmaps和路径，但这是什么意思？他们不仅仅是机器人的嗡嗡声; 这些允许机器人从一个点到另一个点而不会碰到障碍物，在本教程中，我们将介绍构成自主机器人的一些关键概念。

在本教程中，我们将在ROS Indigo中使用Clearpath的Jackal模拟。如果您以前从未使用ROS，或者您的机器上没有安装ROS Indigo，请查看[ROS wiki](http://wiki.ros.org/indigo)以开始使用。

## 入门

我们将首先安装Clearpath的Jackal模拟，桌面和导航软件包。

	sudo apt-get install ros-indigo-jackal-simulator ros-indigo-jackal-desktop

此外，我们需要使用Jackal导航包，而不是使用apt-get安装debians软件包，我们将创建一个源代码工作区，以便我们可以对这些文件进行更改，并了解它们如何影响我们的模拟。有关创建和编译源程序工作区的完整说明，请查看我们的“ [创建工作区和包](http://www.clearpathrobotics.com/assets/guides/ros/Creating%20publisher.html)”教程，但为了本教程的目的，以下是一系列命令将将jackal_navigation包（和其他几个）复制到一个采购的工作区。

	cd ~
	mkdir -p jackal_navigation/src
	d jackal_navigation/src && catkin_init_workspace
	git clone https://github.com/jackal/jackal.git
	git clone https://github.com/jackal/jackal_simulator.git
	git clone https://github.com/clearpathrobotics/LMS1xx.git
	git clone https://github.com/ros-drivers/pointgrey_camera_driver.git
	cd ..
	catkin_make
	
![](https://i.loli.net/2017/08/24/599e9751c2db8.png)

该 **Jackal_navigation** 包仅包含在PARAMS目录的配置文件，并启动文件来加载这些参数，并在ROS导航堆栈启动特定的软件包。导航堆栈是使用ROS机器人进行导航时真正的魔法发生的地方。我们开始使用模拟激光器启动我们的Jackal模拟。

	
source ~/jackal_navigation/devel/setup.bash
roslaunch jackal_gazebo jackal_world.launch config:=front_laser

当你不能使用真正的Jackal时，Gazebo是下一个最好的事情！这个模拟为我们提供了测距和扫描数据，这就是我们需要让Jackal自己开车的！**在新终端中**，输入您的jackal_navigation工作区并启动odom_navigation_demo.launch。

	source ~/jackal_navigation/devel/setup.bash
	roslaunch jackal_navigation odom_navigation_demo.launch

此启动文件启动[move_base](http://wiki.ros.org/move_base)包。 **move_base** 允许Jackal试图通过激光和测距数据来达到目的。

我们将开始我们的导航教程，首先使用 **另一个终端中** 的Jackal导航包中的   **gmapping.launch** 文件构建我们的gazebo世界的地图，记住来源您的工作区！

	source ~/jackal_navigation/devel/setup.bash
	roslaunch jackal_navigation gmapping.launch

如果你熟悉ROS发布的文件，我会鼓励你看看[这里](https://github.com/jackal/jackal/blob/indigo-devel/jackal_navigation/launch/include/gmapping.launch)，否则，你可以看看我们在启动文件教程[这里](http://www.clearpathrobotics.com/assets/guides/ros/Launch%20Files.html)。这个启动文件只是启动[gmaping包](http://wiki.ros.org/gmapping)，并设置了[gmapping ROS wiki](http://wiki.ros.org/gmapping)页面上描述的几个gmapping参数。**在新终端中** ，使用Jackal的gmaping配置打开Rviz。


	source ~/jackal_navigation/devel/setup.bash
	roslaunch jackal_viz view_robot.launch config:=gmapping

![](https://i.loli.net/2017/08/24/599e9859958ca.png)

看看周围，熟悉窗口左侧的所有不同的显示。注意有两个地图。一个是可视化/ **地图** 主题，这是当机器人移动时，由gmapping演示构建的。如果您单击其他地图旁边的复选框，现在可以从 **/ move_base / global_costmap / costmap中查看成本图** 。成本图是负责通货膨胀的障碍，以解决机器人的足迹; 通货膨胀半径越大，机器人将远离这些障碍。

在开始映射之前，让我们再来看看我们的Rviz可视化。单击左下角的 **添加**，并查找 **/ move_base / NavfnROS / plan**，然后单击 **路径** 。这让我们看看Jackal在想什么！它会告诉你，豺狼要尝试用来达到目的，你可以看到它实时变化。

## 制图


现在是时候开始映射了！使用Rviz窗口顶部的工具栏，将2D导航目标发送到Jackal。随着它的驱动，更多的地图将被发现。还要注意路径，以及如何变化，因为Jackal发现障碍。

![](https://i.loli.net/2017/08/24/599e98dd72a31.png)

## 本地化

一旦您对所做的地图感到满意，请继续使用

	
	rosrun map_server map_saver -f jackal_world

这将在您当前的目录中创建一个名为 **jackal_world** 的地图文件，我们将在我们的[AMCL](http://wiki.ros.org/amcl)演示中使用。继续使用CTRL-C终止所有的ROS终端。然后再次启动模拟器，AMCL演示与我们刚刚创建的地图，Rviz与我们的本地化配置，**都在单独的终端**。如果您关闭了Windows，则需要重新发送终端。

当启动下面的AMCL演示（第二行代码）时，请确保包含jackal_world.yaml的绝对路径。

	roslaunch jackal_gazebo jackal_world.launch config:=front_laser
	roslaunch jackal_navigation amcl_demo.launch map_file:=<ABSOLUTE_PATH>/jackal_world.yaml
	roslaunch jackal_viz view_robot.launch config:=localization
为了本演示的目的，我建议隐藏机器人模型和轴，并使用侧面的复选框显示姿势阵列。基于蒙特卡洛本地化估计，Jackal周围的红色箭头是Jackal可能会采取的姿势。这将接收扫描数据和转换，并根据我们之前记录的地图评估数据，以确定其位于Jackal世界中的位置。

![](https://i.loli.net/2017/08/24/599e99eb99d61.png)

如果您给予Jackal一个2D导航目标，您可以看到姿态数组如何更精致，因为系统会收集更多有关其周围的信息，并排除可能的姿势。

![](https://i.loli.net/2017/08/24/599e9a1bf27cd.png)


另外一个重要的工具是2D姿态估计。您可以帮助Jackal进行本地化，大致了解它在地图上的位置。你会注意到姿态数组将被填充在二维姿态估计的一般区域，并再次得到细化，因为它消除姿势。

## 定制

现在我们已经介绍了一些基础知识，让我们带上一个完整的Jackal来玩。

 **urdf**文件夹中有一个 **custom_example.urdf** 文件。我鼓励你自己检查这个文件，看看你能否知道什么传感器正在添加到Jackal，哪里！

一旦你准备好实际启动这个被认可的Jackal，将 **JACKAL_URDF_EXTRAS** 参数设置为此 **custome_example.urdf**文件的文件路径。

	export JACKAL_URDF_EXTRAS=~/jackal_navigation/src/jackal/jackal_description/urdf/custom_example.urdf

从那里，来源您的终端，只需像往常一样启动Jackal模拟。

	source ~/jackal_navigation/devel/setup.bash
	roslauch jackal_gazebo jackal_world.launch

你现在应该看到一个满载的Jackal！有2只大黄蜂相机在前方和后方向下倾斜，2枚LIDARS向前和向后扫描，并有一个点焦相机给我们直接在Jackal前面的图像。

从这里开始，我们可以打开我们之前使用过的任何演示，或者直接进入Rviz，以便可视化我们的新传感器。

	roslauch jackal_viz view_robot.launch

在这一点上，我邀请您玩这些新的传感器，并尝试将其添加到Rviz中，以便您可以看到相机中的不同图像。

![](https://i.loli.net/2017/08/24/599e9acae0cce.png)

如果您真的想要冒险，请尝试更改 **jackal_navigation**启动文件中的一些导航参数。您还可以尝试使用许多在线可用的映射不同的Gazebo世界，甚至制作自己的！

本教程只是划伤了ROS导航包的表面，但我希望这有助于您了解一些基本概念，并让您开始自己探索ROS导航！

---
