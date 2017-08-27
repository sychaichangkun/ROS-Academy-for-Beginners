## 创建工作区和发布者
# 创建工作区和发布者
我们将首先创建一个工作区，然后我们将编写一个简单的发行商，使我们的虚拟赫斯基随机附带。本教程面向Ubuntu 14.04和 [ROS Indigo](http://wiki.ros.org/indigo)。

## 创建工作区和包
在开始编写节点之前，我们需要创建一个工作区和一个包。工作区只是存储所有包的目录。首先我们需要创建一个新的目录。

	mkdir〜/ ros101

这将在您的主文件夹中创建一个目录，我们将使用它作为工作区目录。我们现在需要在工作区目录中创建一个子目录来存储您的软件包的所有源代码。

	mkdir〜/ ros101 / src

创建工作区的最后一步是使用**catkin_init_workspace**初始化工作区。

	cd〜/ ros101 / src
	catkin_init_workspace

现在我们的工作区已经创建，我们将在刚刚创建的src目录中创建一个包。这可以通过导航到 **~/ ros101 / src directory** 来实现，您最近应该已经完成​​了。我们可以使用 **catkin_create_pkg** 命令创建包，然后我们想要将包命名，然后是包将依赖于什么其他包。所有这个命令真的是为您的新包创建另一个目录，并在该目录中的两个新配置文件具有一些默认设置。

	catkin_create_pkg random_husky_driver roscpp std_msgs

您可以看到，这个在random_husky_driver目录中创建了 **CMakeLists.txt** 和 **package.xml**，这也是您存储所有包的源代码的地方。该 **roscpp** 和 **std_msgs** 依赖加入到 **MakeLIst.txt** 和**package.xml** 中。

## 编写出版商
如我们之前的帖子所述，发布商将消息发布到特定主题。对于本教程，我们将发布随机命令到 **/ husky_velocity_controller / cmd_vel** 主题，使您的赫斯基可视化驱动器本身。首先在 **~/ ros101 / src / random_husky_driver** 目录中创建一个名为 **random_driver.cpp** 的文件。首先确保你在  **random_husky_driver** 目录中，然后键入以下内容。

	gedit random_driver.cpp


这将创建并打开新的驱动程序文件。在新的驱动程序文件中，复制以下代码。

	#include  <ros / ros.h>
	#include  <geometry_msgs / Twist.h>
	#include  <stdlib.h>
	int  main （int  argc ， char  ** argv ） { 
	//Initializes ROS, and sets up a node
	ros :: init （argc ， argv “random_husky_commands” ）; 
	ros :: NodeHandle  nh ;
	//Ceates the publisher, and tells it to publish
	//to the husky_velocity_controller/cmd_vel topic, with a queue size of 100
	ros :: Publisher  pub = nh.advertise<geometry_msgs :: Twist > （“husky_velocity_controller / cmd_vel” ， 100 ）;
	//Sets up the random number generato
	srand （time （0 ））;
	//Sets the loop to publish at a rate of 10Hz
	ros :: Rate  rate （10 ）;
		while （ros :: ok （）） { 
		 //Declares the message to be sent
		geometry_msgs :: Twist  msg ; 
		//Random x value between -2 and 2
		 msg.linear.x=4*double(rand())/double(RAND_MAX)-2;
		//Random y value between -3 and 3 
		msg.angular.z=6*double(rand())/double(RAND_MAX)-3;
		//Publish the message
		 pub.publish(msg);
		//Delays until it is time to send another message
		rate.sleep();
		}
	}

让我们逐行分解这个代码，

	#include  <ros / ros.h>
	#include  <geometry_msgs / Twist.h>

这些行包括我们将需要的标题。ROS功能需要使用 **<ros / ros.h>** 标题，并添加了 **<geometry_msgs / Twist.h>** ，以便我们可以创建该类型的消息。

	ros :: init （argc ， argv “random_husky_commands” ）;
	ros :: NodeHandle  nh ;
	
第一行，ros：/：init用于初始化ROS节点，并将其命名为“random_husky_commands”，而ros：NodeHandle启动节点。

	ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("husky_velocity_controller/cmd_vel", 100);

发布 **geometry_msga :: Twist**，以及我们要发送的主题对我们来说，也是**husky_velocity_controller / cmd_vel** 。100是消息队列大小，也就是说，如果您发送消息更快，那么roscpp可以发送，则100个消息将被保存在队列中以便发送。队列越大，缓冲时机器人移动的延迟越多。因此，在现实生活的例子中，在机器人移动的情况下，您将需要一个更小的队列，其中运动命令的延迟是不希望的，甚至是危险的，但丢弃的消息是可以接受的。在传感器的情况下，建议使用较大的队列，因为延迟可以接受，以确保没有数据丢失。

	ros::Rate rate(10)
	...
	rate.sleep()

ROS能够使用 **ros：Rate** 来控制回路频率并且来指示循环以Hz为单位运行的速度。rate.sleep会延迟一段可变的时间，使您的循环以所需的频率循环。这占据了循环的其他部分消耗的时间。所有Clearpath机器人需要10Hz的最小回路速率。

	while （ros :: ok （））

通过使用rosnode kill命令或用户在终端中按Ctrl-C，它将返回true，除非它收到关闭的命令。

	geometry_msgs :: Twist  msg ;

这将创建我们要发送的消息，msg，类型为 **geometry_msgs：Twist**

	msg.linear.x=4*double(rand())/double(RAND_MAX)-2;
	msg.angular.z=6*double(rand())/double(RAND_MAX)-3;

![](https://i.loli.net/2017/08/22/599be2930ad04.png)

这些线计算将发送到赫斯基的随机线性x和角z值。

	pub.publish(msg)


我们终于准备好发布消息！**pub.publish**加入msg发布队列去发送。

## 编译随机赫斯基驱动程序 
编译在ROS中由catkin构建系统处理。第一步通常是在 **CMakeLists.txt** 和 **package.xml** 中设置我们的包依赖关系。但是，当我们创建包并指定我们的依赖关系时，这已经为我们做了。接下来的步骤是将我们的新节点声明为可执行文件，通过将以下两行添加到 **~/ ros101 / src / random_husky_driver** 中的 **CMakeLists.txt** 文件中完成。您可以将它们添加为文件中的最后两行。

	add_executable （ random_driver random_driver.cpp ） 
	target_link_libraries (random_driver $ { catkin_LIBRARIES } )


第一行创建一个名为random_driver的可执行文件，并将ROS指向它的源文件。第二行指定将使用哪些库。现在我们需要使用workspace目录中的catkin_make命令构建我们的工作区。


	cd〜/ ros101
	catkin_make

让我们像以前的一篇博客文章一样，提出赫斯基可视化。

	roslaunch husky_gazebo husky_empty_world.launch


最后一步是在您创建的工作区中输入setup.bash文件。这允许ROS找到工作区中包含的包。不要忘记这个过程必须在每个新的终端实例上完成！
	
	source ~/ros101/devel/setup.bash

现在是时候测试了！gazebo还在运行，让我们启动节点。

	rosrun random_husky_driver random_driver

你现在应该看到赫斯基车！在新的终端窗口中，我们可以确认我们的节点是通过回显关于此主题的所有消息发布到
 **/husky_velocity_controller/cmd_vel** 主题

	rostopic echo / husky_velocity_controller / cmd_vel

![](https://i.loli.net/2017/08/22/599be4836b46a.png)


您现在应该看到随机线性x和角z值的流。

---

		
		
		