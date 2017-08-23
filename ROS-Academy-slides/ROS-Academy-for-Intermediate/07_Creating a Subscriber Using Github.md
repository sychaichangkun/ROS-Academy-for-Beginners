## 使用Github创建订阅者

# 使用GitHub创建订阅者

我们以前学会了如何编写一个发布者节点来随机移动赫斯基。但！发布所有这些消息有什么好处，如果没有人在读它？在本教程中，我们将编写一个订阅者，从Odom主题中读取赫斯基的位置，并绘制其动作。而不是将代码复制到文本文件中，我们将从GitHub中提取所需的软件包，这在开发人员中是非常常见的做法。

在我们开始之前，安装Git以从GitHub和pygame中提取软件包，为我们提供绘制赫斯基动作的工具：

	sudo apt-get install git
	sudo apt-get install python-pygame

## 从GITHUB拉出

由于使用版本控制，GitHub是开发人员中的一个受欢迎的工具 - 大多数ROS软件都有一个关联的GitHub存储库。用户可以从GitHub服务器“拉”文件，进行更改，然后将这些更改“推送”回服务器。在本教程中，我们将使用GitHub来拉出我们将要使用的ROS包。第一步是制作一个新的目录来拉出包：

	mkdir -p ~/catkin_ws/src/ros101
	cd ~/catkin_ws/src/ros101
	git init

由于我们现在知道托管存储库的URL，因此我们很容易从GitHub中提取软件包。使用以下命令访问存储库：

	git pull https://github.com/mcoteCPR/ROS101.git

就这样！您应该在ros101文件夹中看到一个src和launch文件夹，以及一个 **CMakelist.txt** 和 **package.xml** 。您现在有包 **ros101** 其中包括节点 **random_driver.cpp** 和 **odom_graph.py** 。

## 编写订阅者

我们已经在最后一个教程中浏览了random_driver C ++代码，所以这次我们将介绍 **odom_graph.py** 的python代码。该节点使用Pygame库跟踪赫斯基的运动。Pygame是一组用于在python中创建视频游戏的模块; 但是，我们将重点介绍此代码的ROS部分。有关Pygame的更多信息可以在他们的网站上“[点击这里](http://www.pygame.org/news)”找到。**odom_graph**节点的代码可以在以下位置找到：

	gedit ~/catkin_ws/src/ros101/src/odom_graph.py

我们来看一下这个代码：

	import rospy
	from nav_msgs.msg import Odometry

很像C ++发行商代码，其中包括rospy库，并从 **nav_msgs.msg** 导入 **Odometry** 消息类型。要了解有关特定消息类型的更多信息，可以访问[http://docs.ros.org](http://docs.ros.org)查看它的定义，例如，我们使用http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html。下一个代码块导入pygame库并设置显示的初始条件。

	def odomCB(msg)

这是一个odometry回调函数，每次我们的用户收到消息时都会调用它。这个功能的内容只是简单地在我们的显示器上绘制一条从距离测距位置消息读取的最后坐标之间的一行。这个功能将在我们的主循环中不断被调用。
	
	def listener():

以下行启动ROS节点，**anonymous = True** 表示同一节点的倍数可以同时运行：

	rospy.init_node('odom_graph', anonymous=True)

订阅者设置节点从“odom”主题中读取消息，其类型为Odometry，并在接收到消息时调用 **odomCB** 函数：

	rospy.Subscriber("odom", Odometry, odomCB)

此函数的最后一行将保持节点处于活动状态，直到它关闭：

	rospy.spin()

## 把它放在一起

现在是时候测试了！关闭 **odom_graph.py** 文件并使用工作空间目录中的catkin_make函数构建工作区。

	cd ~/catkin_ws
	catkin_make

下一步是启动我们的赫斯基模拟，以启动ROS和所有与赫斯基相关的节点。

	roslaunch husky_gazebo husky_emepty_world.launch

在本教程中，我们提供了一个启动文件，将启动 **random_driver** 和 **odom_graph** 节点。启动文件位于 **〜/ ros101 / src / launch** 中，称为 **odom_graph_test.launch** 。如果您想了解有关启动文件的更多信息，请查看我们的支持知识库中的启动文件文章。现在我们将在新的终端窗口中引用我们的工作区，并将启动文件启动两个节点。

	source ~/catkin_ws/devel/setup.bash
	roslaunch ros101 odom_graph_test.launch

![](https://i.loli.net/2017/08/23/599d3583cbb7e.png)


这样你就有了 ROS101 GitHub1！我们的用户现在正在收听关于odom主题的消息，并绘制出赫斯基的路径。

---