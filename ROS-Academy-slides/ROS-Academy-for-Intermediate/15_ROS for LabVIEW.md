## labVIEW的ROS

# LabVIEW的ROS

ROS for LabVIEW是一组LabVIEW VI，可在不同机器上实现ROS和LabVIEW之间的双向通信。它允许用户初始化节点，发布和订阅各种类型的主题，并在LabVIEW中创建一个ROS Master。

本演示展示了如何将LabVIEW插件安装到LabVIEW库中，正确配置网络，以及如何与LabVIEW的Clearpath Husky进行交互。请注意，这些说明是为在Windows 10上运行的LabVIEW 2014和ROS Indigo在Ubuntu Trusty（14.04）上运行的。不同的操作系统可能会有轻微的变化。

## 安装ROS FOR LABVIEW

**从LabVIEW软件包管理器安装**

打开labVIEW并按照以下步骤操作：

1. 在LabVIEW中打开包管理器，方法是单击工具 -> VI包管理器
2. 向下滚动到ROS的LabVIEW软件，点击并选择安装，
3. 接受许可协议并进行安装

默认情况下，该软件包的内容将安装到您的LabVIEW目录中：

	\LabVIEW 2014\vi.lib\Tufts CEEO ROS for LabVIEW Software

**从源安装**

退出LabVIEW并按照以下步骤操作：

1. 转到[LabVIEW Github仓库的ROS](https://github.com/tuftsBaxter/ROS-for-LabVIEW-Software)
2. 选择 **克隆或下载** -> **下载** 右上角的ZIP
3. 解压缩文件夹，并打开子文件夹 **ROS-forLabVIEW-Software-master** 
4. 将 **LabVIEW软件的ROS** 文件复制到LabVIEW目录中的user.lib

在Windows中：

	C:\Program Files\National Instruments\LabVIEW 2014\user.lib

在Mac中：


	Applications:National Instruments:LabVIEW 2014:user.lib

## 使用ROS进行LABVIEW VI


您现在可以使用刚刚安装的库将块插入到项目VI中。

如果您使用程序包管理器安装了插件，请右键单击项目VI框图，导航到 **Addons** 并选择 **ROS for LabVIEW** 图标，如下所示：

![](https://ooo.0o0.ooo/2017/08/25/599f7c7a9301e.png)

如果从源代码安装了库，请右键单击项目VI框图，导航到 **用户库** 并选择 **ROS for LabVIEW** 图标，如下所示：

![](https://ooo.0o0.ooo/2017/08/25/599f7cba102c3.png)

现在可以将各种VI插入到项目中。一些可用的VI如下图所示。一些最有用的VI包括ROS主题启动/读/写/重复/关闭。还有消息解析和消息构建VI，允许您构建可由ROS读取的消息，以及将ROS中的消息解析为可由LabVIEW读取的格式。这些VI是为常见的ROS消息类型构建的，如std_msgs，geometry_msgs sensor_msgs和nav_msgs。但是，您可能必须创建自己的。这将在本教程的后面更详细地讨论。

![](https://ooo.0o0.ooo/2017/08/25/599f7cf048daf.png)

使用户可以与Baxter，ROSRIO，NAO和Turtlebot等各种设备进行通信。有关使用Baxter VI的示例，请参阅[NI的案例研究](http://sine.ni.com/cs/app/doc/p/id/cs-16503)。

本教程假设您已正确安装和配置ROS。请参阅 **ROS 101：一个实际的例子** 示例，或参考[http://wiki.ros.org/ROS/Tutorials](http://wiki.ros.org/ROS/Tutorials)

## 配置网络

假设您将LabVIEW安装在一台MAC或PC机上（我们称之为LabVIEW计算机），并将ROS安装在另一台Linux机器上（我们称之为ROS计算机）。请注意，本教程尚未使用安装在Windows或Mac操作系统上的ROS进行测试，也没有使用虚拟机进行测试。我们建议您在使用Ubuntu的计算机上安装ROS。

第一步是确保LabVIEW计算机和ROS计算机可以互相通信。您将需要将两台计算机连接到同一网络（通过Wi-Fi或以太网电缆）。有两种配置网络的方法，以确保您将始终在两台计算机之间进行双向通信。这可以使用DHCP预留或使用主机文件完成。

我们建议使用DHCP预留，以确保每次连接到网络时，每台计算机的IP地址保持不变。如果您有许多设备尝试相互通信，那么您也不需要不断检查和更新每个计算机的主机文件，这也更容易去操作。

**确定当前网络参数**

1. 将所有的计算机连接到网络
2. 在每台计算机上运行IP配置命令：

对于Windows，打开命令提示符并输入：

	ipconfig /all

找到并记下 **主机名** ，以及正在使用的适配器的 **物理地址** （或MAC）和 **IP地址** （即，如果通过以太网连接，则使用无线适配器或以太网适配器）。

![](https://ooo.0o0.ooo/2017/08/25/599f7e1f75e9b.png)

对于Ubuntu，打开终端并输入：

	ifconfig -a

找到并写下 **HWaddr** （或物理/ MAC地址），**inet addr** （IP地址）以及将在命令行中显示的 **主机名** 。您还可以通过输入以下命令来查找和/或编辑主机名：

	sudo gedit /etc/hosts

![](https://ooo.0o0.ooo/2017/08/25/599f7e7240c3b.png)

**使用DHCP保留**

对于正在使用的每种类型的路由器，本节将有所不同，但一般步骤如下：

1. 登录您的路由器网页（您将需要路由器的管理员用户名和密码）
2. 在网络设置中查找已连接的客户端，或DHCP Reservation
3. 对于每个设备（即LabVIEW和ROS计算机），添加新的预留，并输入上述部分确定的MAC地址，IP地址和主机名。

您也可以分配其他IP地址而不是最初分配的IP地址。

**使用主机文件** 

如果您不想更改网络设置（或无法执行此操作），则可以通过编辑每台计算机的相应主机文件来存储主机名及其关联的IP地址。

对于Windows，请输入：

	notepad C:\windows\system32\drivers\etc\hosts

对于Ubuntu，请输入：

	sudo gedit /etc/hosts

添加上面确定的每台计算机的主机名和IP地址。请注意，您需要将所有主机名/ IP对添加到所有计算机上的主机文件中。每当您的设备重新连接到网络时，都需要检查，因为IP地址可能会更改。

**检查双向通信**

我们需要确认每台电脑现在可以在网络上相互交流。假设ROS计算机的IP为192.168.0.155，主机名为ClearpathNC，LabVIEW计算机的IP为192.168.0.120，主机名为SDIC-PC1。

首先打开终端并从ROS计算机ping ROS计算机：

	ping 192.168.0.155

与LabVIEW计算机类似，打开命令提示符并输入：

	ping 192.168.0.120

在这两种情况下，您都应该收到同一个IP地址的回复。您还应该能够使用主机名进行ping操作。

现在，从ROS计算机ping LabVIEW计算机：

	ping 192.168.0.120

类似地，从LabVIEW计算机ping ROS计算机：

	ping 192.168.0.155

如果失败，则计算机无法在网络上看到对方。您将需要重新配置您的网络或主机文件。有关更多信息和一些其他故障排除方法，请参阅[网络设置的ROS教程](http://wiki.ros.org/ROS/NetworkSetup)。

阻止双向通信的一个常见问题是Windows防火墙。如果您无法从Ubuntu计算机ping您的Windows计算机，请尝试创建防火墙规则。如果您的计算机可以ping通，可以跳过下一节。

**创建Windows防火墙规则**

如果您在LabVIEW计算机上使用Windows，则可能需要创建防火墙规则。请按照以下步骤操作，然后重试以ping计算机：

1. 打开 **控制面板**  ->  **系统和安全**  ->  **Windows防火墙**  ->  **高级设置**
2. 点击“ **入站规则** ”，然后选择“** 新建规则** ”。
3. 将以下属性分配给新规则：
	* 在规则类型下选择 **自定义** 规则
	* 应用于 **程序**下的 **所有程序** 
	* 在协议和端口下为协议类型选择 **ICMPv4** 
	* 适用于范围下的本地和远程IP地址的 **任何IP地址**
	* 选择 **允许** 在“操作”下的 **连接** 
	* 在配置文件下检查 **域** ，**私有** 和 **公共**
	* 分配名称，如“ICMPv4 ROS通信规则”，然后选择“ **完成** ”


注意：您可能还需要使用 **Outbound Rule** 做同样的事情。

## 用LABVIEW驱动赫斯基

我们现在准备开始在LabVIEW中构建代码并与ROS进行通信。在本节中，我们将学习如何在LabVIEW中创建一个VI，以便在另一个设备上的模拟中驱动一个赫斯基（Husky）。为此，我们将需要在ROS计算机上启动一个ROS Master，然后在LabVIEW中初始化一个节点，该节点将发布可以连接到ROS Master的任何设备订阅的速度命令。

首先，让我们在凉亭里推出一个将会从LabVIEW订阅速度命令的赫斯基。您将需要安装哈士奇桌面和哈士奇模拟器，请参阅 **ROS 101：驱动赫斯基** 教程的指导。一旦这些安装在您的ROS计算机上，打开一个新的终端并输入：

	roslaunch husky_gazebo husky_empty_world.launch

这应该在一个空旷的世界中与赫斯基启动Gazebo，如下所示：

![](https://ooo.0o0.ooo/2017/08/25/599f80db8d182.png)

现在我们来打开rqt图，以便在新终端中显示所有当前的节点和主题：

	rqt_graph

![](https://ooo.0o0.ooo/2017/08/25/599f81020f0f8.png)

你应该看到像上面的照片。要驱动赫斯基，我们要发布到 **/husky_velocity_controller/cmd_vel** 主题。您也可以发布到/ cmd_vel，但是如果您有多个设备由cmd_vel命令（例如turtlebot或turtlesim）控制，那么这可能会导致问题。现在我们知道我们需要在LabVIEW中发布的主题，所以我们现在需要的是找到我们需要发送给该主题的消息类型。在您的ROS计算机上的新终端中，输入：

	rostopic info /cmd_vel

您还可以输入 **rostopic信息/ husky_velocity_controller / cmd_vel** ，这将给出相同的结果。这将显示这些主题需要类型为 **geometry_msgs /Twist** 的消息。现在我们在LabVIEW中创建这个！

在LabVIEW计算机上，打开一个空白的LabVIEW项目，然后添加一个新的VI。在这个新VI的框图（BD）上，点击并导航到您的ROS for LabVIEW库。如果您通过软件包管理器或用户库（如果从源安装）安装了库，则这将在Addons下。

在ROS下，将以下VI拖放到BD上，如图所示：

* ROS_Topic_Init
* ROS_Topic_Write
* ROS_Topic_Close

![](http://www.clearpathrobotics.com/assets/guides/ros/_images/DriveAHusky3.png)

这三个VI对于将发布给ROS的大多数程序至关重要。ROS_Topic_Init启动ROS中的节点，并定义节点名称，要发布/订阅的主题，消息的类型，队列大小和更新速率。ROS_Topic_Write是实际发布在项目VI中创建的消息的VI。ROS_Topic_Close会杀死该节点。每次完成运行VI时，都要关闭节点。否则，如果您停止并重新启动VI，而不会杀死您的节点，或重新启动ROS Master，则可能无法正确重新初始化该节点。

接下来，我们来定义节点的名称和属性：

* 右键单击ROS_Topic_Init块的 **节点** 终端，选择 **创建 ->常量** 并将其命名为 **/LVHuskyControl** 
* 对ROS_Topic_Close块重复
* 右键点击“ **定义** ”终端，然后选择“ **创建** ” ->“ **常量** ”
* 填写如下所示的定义

![](http://www.clearpathrobotics.com/assets/guides/ros/_images/DriveAHusky4.png)

“定义常数”中的顶级元素是我们要发布的主题，第二个是我们之前确定的要发布的消息的类型。

接下来，转到LabVIEW库的ROS，并在我们的ROS_Topic_Write块的左侧添加一个 **Twist** 消息块。这可以在 **ROS - > MessageBuilder - > geometry_msgs - > add_Twist** 下找到。右键单击 **Twist** 终端并创建一个 **控件** 。我们现在可以连接所有VI块。连接以下终端：

* ROS_Topic_Init **ROS out**  -> ROS_Topic_Write **ROS in**
* add_twist **msg_out**  -> ROS_Topic_Write **msg_in**
* ROS_Topic_Write **ROS out**  -> ROS_Topic_Close **ROS in** 
* ROS_Topic_Init **Error out**  -> ROS_Topic_Write **Error in** 
* ROS_Topic_Write **Error out**  -> ROS_Topic_Close **ROS in**
* ROS_Topic_Close **Error out** -> Indicator


现在让我们创建一个while循环来连续运行ROS_Topic_Write和add_Twist VI。这将不断发布我们的消息。您的代码应该类似于此（清理电线后）：


![](http://www.clearpathrobotics.com/assets/guides/ros/_images/DriveAHusky5.png)


最后，我们将while循环上的停止图标映射到控件，并在while循环中添加一个启动/停止控件的案例结构。您还可以向ROS_Topic_Write输出添加一个指示符，以便可视化正在发布的消息。经过一些视觉重新配置，你的最终代码应该类似于：

![](http://www.clearpathrobotics.com/assets/guides/ros/_images/DriveAHusky6.png)

现在我们准备用LabVIEW驱动赫斯基！添加速度命令，命中运行，LabVIEW将要求您输入主机（ROS计算机）IP地址，然后在仿真中观察赫斯基移动。

如果您在LabVIEW计算机上打开并刷新rqt图，应该会看到一个名为n_LVHuskyControl的节点，它将按预期发布到/ husky_velocity_controller / cmd_vel。

重要笔记：

* 当您控制真正的赫斯基时，停​​止/开始发布控制将非常重要。建议您在布尔值设置为关闭时更改速度命令。这将允许更容易的控制。
* 只有线性x和角z可以改变来驱动赫斯基。这是因为赫斯基只能在x方向（向前/向后）移动并围绕z轴旋转。
* **始终** 通过按前面板上的 **STOP** 按钮来结束脚本很重要。不要按顶部工具栏上的“中止执行”按钮。您需要退出while循环才能关闭该节点，否则您将遇到问题（可以通过重新启动项目来修复它）。
* 您可以提前设置Master的IP地址，以便在运行代码时不必继续输入。为此，搜索_ROSDefinition VI，更改ROS Master IP，然后在 **编辑** 选项卡中选择 **Make Current Values Default**。

下一节将介绍如何下载和安装Clearpath的自定义VI，说明如何订阅赫斯基测距数据和激光扫描数据，以及如何在LabVIEW中绘制这些测量。

## 下载并安装CLEARPATH VI

转到[ROSforLabVIEW-Clearpath Github仓库](https://github.com/nickcharron/ROSforLabVIEW-Clearpath)。克隆或下载zip并将内容保存到项目目录中。如果您希望Clearpath VI可以作为您的BD中的块使用，您可以将提取的VI保存到您的 **user.lib** 目录中，然后可以通过在BD中点击并转到 **用户LIbraries** 来访问该块。

项目控制Husky.lvproj应包含三个VI和两个子VI。Velocity Publisher VI是我们在上一节中生成的代码。OdometrySubscriber.vi订阅来自Gazebo（或您的赫斯基机器人）的EKF Localized odometry，显示姿势（位置和方向）并绘制xy位置。OdomLaserSubscriber.vi还订阅了激光扫描并绘制了这个xy数据。parseLaserScan.vi和parseOdometry.vi将在下一节讨论。

## 追踪一个赫斯基，并绘制激光雷达扫描图

本节将讨论OdomLaserSubcriber.vi如何工作以及如何使用它。在打开用户VI之前，让我们开一个新的Gazebo模拟，但是这次我们会选择一个有激光检测障碍的世界。结束当前的Gazebo模拟，并在终端中输入以下命令：

	roslaunch husky_gazebo husky_playpen.launch

现在打开OdomLaserSubscriber VI以及VelocityPublisher VI。这样我们就可以控制赫斯基（Husky），同时使用LabVIEW进行测量和激光扫描。在用户的前面板中，将 **更新时间** 设置为300ms，将 **最大Odom Pts设置** 为500.这将有助于确保用户有足够的时间跟上传入的消息，并且您不会在LabVIEW内部运行内存。然后您可以按订户上的运行，然后运行VelocityPublisher一些速度命令输入。

下图显示了OdomLaserSubscriber的代码以及Husky移动时的输出。

![](http://www.clearpathrobotics.com/assets/guides/ros/_images/TrackPlotLidar1.png)

让我们更详细地看看代码。在这段代码中，我们启动了类似于VelocityPublisher的节点，但是我们在主题定义中选择了Subscriber。在这种情况下，我们启动两个节点，包括一个节点来检索测距数据（/odometry / filtered topic）和一个节点以检索激光扫描数据（/scan topic）。我们还使用ROS_Topic_Read而不是ROS_Topic_Write。

顶部案例结构允许用户通过在true时删除数组来清除绘图。2D姿态从parseOdometry子VI输出为具有双倍x和y的集群。然后将姿势从循环的最后一次迭代追加到先前的姿势集群，然后绘制。只要while循环运行，姿态集群将继续增长，直到用户清除图形或直到达到前面板中指定的最大数量点数为止。

![](http://www.clearpathrobotics.com/assets/guides/ros/_images/TrackPlotLidar2.png)


paseLaserScan子VI输出范围阵列和尺寸为1 xn的轴承阵列，其中n是您的特定激光器发布的光束数量。然后将范围和轴承转换为赫斯基坐标系中的光脉冲的x和y位置。然后可以将该数据捆绑并绘制。

![](http://www.clearpathrobotics.com/assets/guides/ros/_images/TrackPlotLidar3.png)


**消息构建和解析**

当您要发布或订阅尚未包含在LabVIEW库的ROS中的特定类型的消息时，您将需要编写自己的消息构建器和解析器。消息从ROS传输到LabVIEW，首先将数据压缩成字符串，然后将其解析为可用格式。同样，当您想从不属于ROS for LabVIEW库的LabVIEW发布特定类型的消息到ROS时，您必须将自己的消息构建为ROS节点订阅该消息的可读格式。塔夫茨大学在他们的[Github图书馆](https://github.com/tuftsBaxter/ROS-for-LabVIEW-Software)有一些消息解析和构建的教程。

本项目中使用的parseLaserScan.vi和parseOdometry.vi是如何完成消息解析的好例子。这些VI专门用于分析由husky发布的/scan和/ odometry/filters主题的sensor_msgs/LaserScan和nav_msgs/ Odometry消息类型。

注意：如果在运行激光用户时遇到内存问题，您可能需要考虑减少激光扫描数据的大小。您可以通过将范围测量值保存为不同的数据类型或通过减少要保存的波束数量来在parseLaserScan.vi中执行此操作。

**视频教程** 

以下视频显示了上述教程正在仿真和真正的赫斯基机器人中实现：

[视频教程](https://youtu.be/nu2S1gwlyIk)

## 支持

ROSforLabVIEW附加软件是由塔夫茨大学（Tufts University）提供支持的。除了本演示文稿中提供的VI外，National Instruments和Clearpath Robotics都不会为插件的内容提供技术支持

**联系塔夫茨大学技术支持** ：

* 电话：（617）627-5888
* 电子邮件：[tuftsBaxter@gmail.com](mailto:tuftsBaxter%40gmail.com)
* 社区组：[https://decibel.ni.com/content/groups/ros-for-labview](https://forums.ni.com/t5/ROS-for-LabVIEW-TM-Software/gp-p/8525)

## 附加信息

有关更多信息和附加演示，请参阅：

* [ROSforLabVIEW网站](https://sites.google.com/site/rosforlabview/home)
* [NI ROSforLabVIEW附件](http://sine.ni.com/nips/cds/view/p/lang/zhs/nid/213279)
* [塔夫茨大学Github仓库](https://github.com/tuftsBaxter/ROS-for-LabVIEW-Software)
* [社区小组](https://forums.ni.com/t5/ROS-for-LabVIEW-TM-Software/gp-p/8525)
* [塔夫斯大学演示视频](https://www.youtube.com/channel/UCsvu5N8XdfguzpDoAJGS2Ew)
* [Google网上论坛](https://groups.google.com/forum/#!forum/ros-sig-rosforlabview)

有关如何在各种NI RIO设备上实现此功能的信息，请参阅塔夫茨大学Github仓库中的帮助目录。在部署源分发时，您需要仔细阅读说明。ROSforLabVIEW库已经由塔夫茨大学在myRIO和roboRIO上进行测试，并应与其他RIO设备配合使用。

---
