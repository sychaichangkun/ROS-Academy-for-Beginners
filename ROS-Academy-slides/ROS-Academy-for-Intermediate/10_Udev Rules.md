## Udev规则

# UDEV 规则

Udev是Linux的设备管理器，可动态创建和删除硬件设备的节点。简而言之，它可以帮助您的电脑轻松找到您的机器人。默认情况下，连接到Linux（Ubuntu）PC的硬件设备将属于root用户。这意味着作为未启动（即不是root）的用户运行的任何程序（例如ROS节点）将无法访问它们。除此之外，设备将根据插入的顺序任意地接收到ttyACMx和ttyUSBx等名称，幸运的是，您可以使用 **udev规则** 解决这个问题。

您可能已经在系统上至少有一个udev规则解决了网络设备的命名问题，您可以在 **/etc/udev/rules.d/** 文件夹中查看它，它可能命名为 **70-persistent-net .rules**。

一些驱动程序/软件包将提供可以使用的udev规则。检查 **/etc/udev/rules.d/** 文件夹，看看是否有任何安装。如果包是懒惰的，并给你一个udev规则来安装自己，你可以这样做：

	sudo cp <rule file> /etc/udev/rules.d/>

## 写一个新的UDEV规则：

如果您仍然需要编写自己的规则来设置设备的命名和权限，请继续阅读。规则可以变得非常复杂，但下面应该涵盖用于ROS应用的99％的用例。如果你正在寻找99.9％，我建议你[从这里开始](http://www.reactivated.net/writing_udev_rules.html)。作为一个例子，我们将检查ROS中的urg_nodedriver提供的udev规则：

	SUBSYSTEMS=="usb", KERNEL=="ttyACM[0-9]*", ACTION=="add", ATTRS{idVendor}=="15d1", ATTRS{idProduct}=="0000", MODE="666", PROGRAM="/opt/ros/hydro/lib/urg_node/getID /dev/%k q", SYMLINK+="sensors/hokuyo_%c", GROUP="dialout"

udev规则由一堆逗号分隔的标签组成，如上所述。标签分为两部分：匹配和配置，但是它们可以按任何顺序写入规则（足够令人困惑）。

# 匹配：

匹配的部分使udev设备管理器将规则与所需的设备相匹配。经理将尝试在插入所有新设备时匹配所有新设备，因此重要的是规则具体到足以捕获您正在查找的设备，否则最终将使用/ dev / hokuyo符号链接IMU。有很多潜在的匹配标签，最好的选择有用的方法是从udev直接获取所有的设备属性。

运行以下cammand，插入 **<devpath>** ，如 **/dev/ttyACM0** ：

	udevadm info -a -p $(udevadm info -q path -n <devpath>)

您将获得udev可见的所有设备属性的列表。看着设备'... / ttyACM0'：

	KERNEL == “ttyACM0” 
	SUBSYSTEM == “tty” 
	DRIVER == “” 
	查看父设备'...'：
	 KERNELS == “3-3：1.0” 
	SUBSYSTEMS == “usb” 
	DRIVERS == “cdc_acm” 
	ATTRS { bInterfaceClass } == “02” 
	ATTRS { bInterfaceNumber } == “00” 
	看着父装置'...' ：
	... 
	ATTRS { idVendor } == “0483”
	ATTRS { idProduct } == “5740”
	...

每个设备属性都是一个潜在的标签。您可以使用第一部分中的任何标签过滤，以及父设备中的标签。使用正则表达式使匹配更灵活（例如[0-9]匹配任何数字，*匹配任何东西）。例：

	SUBSYSTEM=="tty", KERNEL=="ttyACM[0-9]*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", ...

由于udev的设计方式，您只能从一个父设备拉入标签。因此，在上述示例中，您可以随时在udev规则中使用KERNEL和SBSSTEM标记，但不能同时使用DRIVERS和ATTRS {idVendor}进行匹配。通常这不是问题，因为idVendor和idProduct始终存在，并且唯一地标识大多数设备类型。

您还应该添加ACTION标签，通常是“添加”，如果要在拔下设备时执行某些操作，有时候“删除”。

	...，ACTION == “add”，...

## 配置：

现在，您有一个与您想要的设备相匹配的规则，您可以添加几个配置标签：


| 标签 | 用法 |
| ------ | ------ |
| MODE =” 0666” | 设置权限以允许任何用户对设备的读/写访问。 |
| SYMLINK+=”hokuyo” | 在/dev/中为此设备创建一个符号链接。 |
|RUN+=”/bin/echo ‘hello world’|执行任意命令。 [用于高级用途](http://www.reactivated.net/writing_udev_rules.html#external-run)|

确保符号链接对于每个设备是唯一的，所以上面实际上是很差的做法！如果您有多个相同类型的设备（例如2个Hokuyos），或者如果您有多个设备使用通用USB到串行转换器（例如FTDI），则基本idVendor和idProduct规则将无法正确区分这些设备，因为udev将映射所有匹配的设备到相同的符号链接。有几种方法：

 **直接通过设备属性：**

如果您的设备具有编号为其属性的唯一标识符（例如序列号），则可以无痛地为每个设备创建唯一的符号链接：

	..., SYMLINK+=”device_$attr{serial}”, ...

制造商不会总是让你这么容易。如果父设备具有序列号，则可以使用环境变量使用以下技巧。为父设备创建一个udev规则来存储环境变量：

	..., <match parent device>..., ENV{SERIAL_NUMBER}="$attr{serial_number}"

以及在符号链接中使用该变量的子设备的规则：

	..., <match child device>..., SYMLINK+="device_$env{SERIAL_NUMBER}"

 **通过外部程序：**

如果制造商根本没有通过设备属性公开唯一的标识符，则可以使用PROGRAM标签执行外部命令：


	PROGRAM="/bin/device_namer %k", SYMLINK+="%c"

与旋转的RUN标签不同，此命令将阻止（需要在规则完全处理之前执行），因此必须快速返回。上面的theurg_node驱动程序使用此标签来执行ROS二进制文件：

	PROGRAM="/opt/ros/hydro/lib/urg_node/getID /dev/%k q", SYMLINK+="sensors/hokuyo_%c"

替换参数％k是指相对于/ dev /的设备路径，％crefers到PROGRAM标签的输出。

 **运行一个新的udev规则：**

将您的规则复制（sudo cp）到/etc/udev/rules.d/文件夹后，可以使用设备进行测试。要使udev识别您的规则，请运行以下命令：

	sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger

您应该可以在/ dev /中找到链接到完整设备路径的符号链接（例如/ dev / ttyACM0），并且所有用户都应该读/写设备路径的权限。如果未设置权限，并且/ dev /中未正确创建符号链接，则可以尝试使用适当的设备路径运行以下命令来模拟udev对规则的处理：

	udevadm test $(udevadm info -q path -n /dev/ttyACM0)

## 要记住的事情

* 检查您的规则是否符合命名约定 - <priority> - <device name> .rules。从技术上讲，您可以为同一台设备制定多条规则，而且该号码决定执行的顺序。由于我们正在编写插件规则，99的优先级最为安全。
* 文件中可以有多个规则，用换行符分隔。确保每个单独的规则在一行。
* 检查所有标签（匹配和配置）是否以逗号分隔。
* 检查你的规则文件是否有一个尾随的换行符。
* 检查您的规则是否由root用户拥有  – ll /etc/udev/rules.d/should 对规则文件说“root root”。

---
 


