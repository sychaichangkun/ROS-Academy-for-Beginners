## 创建RQT仪表板

# 创建RQT仪表板

在终端，gazebo 和 RViz工作后，现在是改变步伐的时候了。对于本教程，我们将详细介绍创建自己的rqt仪表板的基础知识。仪表板是单个rqt窗口，一个或多个插件显示在可移动，可调整大小的框架中。仪表板通常由许多插件组成，提供一套UI功能，用于使用机器人和机器人数据。

仪表板可以在rqt会话中进行交互式填充和配置。优选的配置可以保存为“透视图”，可以节省加载的插件，布局以及支持的位置，设置和上次保存的初始参数（例如我们上次绘制的主题）。在本教程中，我们将使用ROS Indigo中的赫斯基模拟。要安装ROS Indigo，请参阅[这些说明](http://wiki.ros.org/indigo/Installation/Ubuntu)，并访问我们的[Husky](http://wiki.ros.org/Robots/Husky)页面来安装赫斯基模拟。

## 入门

第一步是安装rqt！我们还将安装一些常见的插件来创建我们的仪表板

	sudo apt-get install ros-indigo-rqt ros-indigo-rqt-common-plugins ros-indigo-rqt-robot-plugins

然后，我们可以通过简单地使用RQT来启动

	roscore

然后在新的终端标签中，我们可以启动rqt

	rqt

在插件菜单中，选择要加载的每个插件。您可以通过拖动和重新调整每个插件的标题栏或边缘来更改布局。

![](https://i.loli.net/2017/08/23/599d6bd3e2e04.png)

## 一个实际例子

我们现在将创建一个带有几个实用插件的rqt仪表板，并展示了这个特定仪表板的潜在用例。对于本教程，我们将使用我们的虚拟赫斯基来模拟传感器数据。在Gazebo上打开赫斯基：

	roslaunch husky_gazebo husky_playpen.launch

我们现在设置了rqt仪表板，现在可以模仿Gazebo。首先从插件下拉菜单中打开以下插件，然后根据需要重新调整大小，

* Rviz
* plot x2
* Bag
* Robot Steering

![](https://i.loli.net/2017/08/23/599d6c58058ca.png)


每个插件都有自己的使用和设置，有关特定插件的更多信息，请访问[ROS Wiki的rqt插件页面](http://wiki.ros.org/rqt/Plugins)。

当您对仪表板配置感到满意时，您可以通过选择“透视图”>“ **创建透视** 图”来保存透视图，为其 **创建** 名称，并要求其克隆当前透视图。这些视角在本地保存并在会话之间持续存在。


要导出透视图以进行更密切的管理，例如共享或持久存储库，请选择 **“透视图”>“导出”**，并为文件名扩展名.perspective指定名称。

## 加载视角

透视图可以通过选择透视图进行交互式加载到RQT中。然而，直接从命令行启动它们是有用的，它允许我们将它们包装在一个可以是rosrun或roslaunched的脚本中：

	rqt --perspective-file "$(rospack find my_ros_rqt_package)/config/my_dashboard.perspective"

一些插件允许您配置影响其安装和行为的选项。例如， **Python控制台** 插件允许您选择要使用的控制台实现。您可以通过在标题栏中选择齿轮图标来访问任何插件的这些选项。如果没有齿轮图标，则插件尚未配置为提供选项菜单。

Rviz：要将Husky加载到Rviz插件中，请从下拉菜单中选择 **打开的配置** ，然后导航到 **/opt/ros/indigo/share/husky_viz/rviz/view_robot.rviz**。你现在应该看到在Rviz装载的赫斯基模型！默认情况下，此配置文件将包含模拟激光，您可以在Gazebo环境中的Husky路径中查看该对象。

![](https://i.loli.net/2017/08/23/599d70460e9ec.png)


![](https://i.loli.net/2017/08/23/599d7071b6a80.png)


**plot** ：plot工具可用于实时绘制特定主题，因为这个例子，我们将绘制指定的odometery主题与模拟的odometrey。在插图右上角的输入窗口中，在每个图中添加以下主题。

/odometry/filtered/twist/twist/angular/z

和

/husky_velocity_controller/odom/twist/twist/angular/z

**机器人转向** ：机器人转向插件为我们提供了一种手动驱动赫斯基的简单方法，所需要的就是指定接受速度命令来移动机器人的主题，对于我们的虚拟赫斯基，该主题是 **/cmd_vel**。

现在是时候把它放在一起了！尝试使用机器人转向插件命令赫斯基转到位，并在更新激光扫描时观察您的赫斯基是RViz旋转的位置！您还应该在其中一个地块中看到指定的测距仪，而实际测距仪则会稍微滞后，因为它达到所需的值。

![](https://i.loli.net/2017/08/23/599d71556b8f7.png)

**Rqt包** ：Rosbag是一个非常有用的记录工具，我们的支持团队可能经常要求一个bag文件来仔细观察你的系统。可以通过终端记录一个行李，但使用rqt更简单和更直观。让我们通过点击录音按钮，选择要录制的主题，来记录赫斯基的行李箱文件。一旦您对所记录的数据感到满意，请停止录制。

打包包文件也是一样简单。让我们继续关闭rqt和Gazebo，所以ROS不再运行，然后再次使用roscore启动ROS

	
	roscore

并打开rqt备份并再次加载ROS包插件

	rqt

这一次我们打开我们刚刚记录的包文件，点击第二个按钮。您现在将看到所有记录的主题，以及通过该主题发送消息时。您可以通过右键单击并选择查看值或绘制特定主题来仔细查看特定票据。

![](https://i.loli.net/2017/08/23/599d734d9d65a.png)

有关rqt的更多信息，请访问[ROS Wiki](http://wiki.ros.org/rqt)页面，如果您对此特定教程有任何疑问，请随时与我们联系！

---