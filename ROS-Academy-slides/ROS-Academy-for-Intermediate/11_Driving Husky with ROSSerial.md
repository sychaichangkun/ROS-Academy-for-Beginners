## 驱动有ROSserial的HUSKY和Ardunio

# 驱动有ROSserial的HUSKY和Ardunio

Arduino微控制器系列由于其简单易用而迅速成为业余爱好者，但是经常次机器人必须创建通信协议以允许其嵌入式硬件与计算机通信。那就是rosserial来了！Rosserial是通过串行接口发送ROS消息的通用协议，例如Arduino上的UART。这使您可以轻松地将连接到Arduino的任何传感器连接到ROS环境中！

本教程将让您开始设置Arduino IDE并安装rosserial库。然后，我们将通过使用Arduino板上的整洁技巧驱动赫斯基模拟来测试它。不用说你应该需要一个Arduino来完成这个教程。我们还将使用赫斯基模拟器，因此请确保运行我们的文档：doc：如果尚未完成，请[驱动赫斯基](http://www.clearpathrobotics.com/assets/guides/ros/DriveaHusky)教程。

## 设置


第一步是设置您的Arduino环境，首先安装Arduino IDE和rosserial使用：

	sudo apt-get install arduino arduino-core ros-hydro-rosserial ros-hydro-rosserial-arduino

首次打开Arduino IDE后，将在您的主目录中创建一个名为“sketchbook”的目录。现在我们需要将rosserial库导入到arduino IDE中。如果您的素描本目录为空，则可能需要在其中创建一个名为“libraries”的目录。

	mkdir ~/sketchbook/libraries
	cd ~/sketchbook/libraries
	rosrun rosserial_arduino make_libraries.py .

重新启动您的Arduino IDE，您应该看到您的库的ros_lib部分！

![](https://i.loli.net/2017/08/24/599e3474bde07.png)


您将要确保您的Ubuntu用户是“Dailout”组的一部分，它允许您访问串行端口。您可以使用检查

	groups "username”

如果您没有看到“dailout”，您可以轻松地添加自己使用

	
	sudo gpasswd --add “username” dialout

使我们的生活更轻松的最后一步将是创建udev规则，以便在插入时识别Arduino并设置正确的权限。有关udev规则的更多信息，请查看我们的udev文章。开始插入你的Arduino。

**注意** ：如果您正在使用虚拟机，则在插入虚拟机后，必须将Arduino连接到虚拟机。

![](ttps://i.loli.net/2017/08/24/599e353aa1a00.jpg)


您可以确认您的系统实际上是通过运行连接到Arduino

	ls -l /dev

你应该看到一条类似的行


	Bus 002 Device 005: ID 2341:0043

确认这个确实是你的Arduino，断开它并再次运行命令，注意哪个条目已经消失了。记住ID号，在这种情况下，2341是供应商ID，0043是产品ID。现在冒险到你的udev规则：

	cd /etc/udev/rules.d/

并创建我们的新规则文件，规则文件的命名约定遵循“## - name.rules”。选择一个没有使用的数字！

	sudo gedit 97 -arduino.rules


将以下内容复制到新的规则文件中，将####替换为您的产品ID和供应商ID。有关这些标签的含义的更多信息，请查看我们关于udev规则的文章。

	
	SUBSYSTEMS=="usb", ACTION=="add", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="00[0-f][0-f]", MODE="0666", SYMLINK+="arduino arduino_$attr{serial}", GROUP="dialout",

剩下的就是更新你的udev规则并重新启动你的系统

	sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger

您现在应该看到“arduino”作为具有完全权限的ls -l/dev中的条目！（rw-rw-rw-）

## 码

我们现在将我们的代码上传到Arduino！代码很简单，但是如果您有任何困难，请查看我们的“创建发布商”教程。将以下代码复制到Arduino IDE中，然后单击上传。如果您的udev规则设置正确，您应该能够上传没有任何错误。

如果遇到任何错误，请确保您的arduino在ls -l/dev中成为“arduino”，并设置正确的权限。您可能还需要将Arduino IDE指向工具 ->串口中的正确USB端口。

	#include <ArduinoHardware.h> 
	#include <ros.h>
	#include <geometry_msgs / Twist.h>

	ros :: NodeHandle nh ;
	geometry_msgs :: Twist msg ;
	ros :: Publisher pub （“husky / cmd_vel”，＆ msg ）;
	void setup （）
	{ 
	nh.initNode （）; 
	nh.advertise （pub）; 
	} void loop （）
	{ 
	if （ digitalRead （8 ）== 1 ） 
	msg.linear.x = -0.25 ;

	else if  （ digitalRead （4 ）== 1 ） 
	msg.linear.x = 0 .25 ;

	else  if  （ digitalRead （8 ）== 0  && digitalRead （4 ）== 0 ） 
	msg.linear.x = 0 ;

	pub.publish （＆ msg ）; 
	nh.spinOnce （）; 
	}

## 驱动HUSKY

现在，Arduino加载了我们的代码和发布速度命令，我们可以将这些消息传递到我们的ROS环境中。我们将从启动赫斯基模拟开始：

	roslaunch husky_gazebo husky_empty_world.launch

剩下的就是将Arduino附加到我们的ROS环境中：

	rosrun rosserial_python serial_node.py _port：= / dev / arduino

我们已经准备好尝试了！去触摸数字引脚8，你应该看到赫斯基的前进！同样地，如果你触摸数字引脚4，赫斯基将向后推。

![](https://i.loli.net/2017/08/24/599e375ddbeb3.jpg)

这个技巧可以通过称为[寄生电容](https://en.wikipedia.org/wiki/Parasitic_capacitance)的现象成为可能，这在电子设计中通常是不希望有的影响，但是为了我们的例子的目的很好地服务。话虽如此，这不是最可靠的方法，而是用最小的设备提供一个简单的例子。如果您在移动模拟赫斯基时遇到困难，请尝试使用rostopic echo / husky / cmd_vel来验证某些命令事实上当您触摸引脚时被发送到赫斯基（Husky）。

一定要通过我们的[知识库中](https://support.clearpathrobotics.com/hc/en-us)的其他ROS教程。如果您想了解有关ROSserial的更多信息，请务必访问[ROS wiki](http://wiki.ros.org/rosserial)的rosserial页面。

---