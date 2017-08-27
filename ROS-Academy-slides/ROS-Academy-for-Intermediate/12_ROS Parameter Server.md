## ROS参数服务器

# ROS参数服务器

ROS中的配置信息通常保存到参数服务器。参数服务器是可以通过命令提示符，节点或启动文件请求访问的值的集合。参数旨在是相当静态的，全局可用的值，如整数，浮点数，字符串或bool值。

## 终端参数

可以使用各种rosparam命令直接从终端查看和更改参数。最简单的命令列出了当前所有的所有参数。

	rosparam list

随着Husky gazebo 模拟运行，以下是rosparam列表的输出示例，并给出了最适合参数服务器的值的类型。

	rosparam get

例如，我们可以看到，在凉亭的z方向的重力设置为-9.8，我们目前正在运行ROS水电站。

也可以使用rosparam set来更改参数

	rosparam set

说我们想模拟赫斯基在月球上的驾驶，那么我们可以很容易地改变重力参数是凉亭

	rosparam set /gazebo/gravit_z -1.6

## 从节点访问参数

通常情况下，您的节点在启动过程中必须访问参数服务器才能检索配置信息，或设置参数值。这可以很容易地在C++ 或 Python中完成，设置参数使用：

C++

	void:ros::param::set(parameter_name, input_value)

Python

	rospy.param_set(parameter_name, input_value)

类似地，从参数服务器检索参数值

C++

	void:ros::param::get(parameter_name)

Python

	rospy.param_get(parameter_name)

## 从启动文件访问参数

您可能需要访问参数服务器的最终来源是启动文件。在启动文件期间设置参数值是在启动时方便地初始化参数的常见做法。这可以在您的启动文件中使用

	<param name="param-name" value="param-value" />

您还可以使用YMAL格式以及参数标签，这是一个易于阅读的1到1格式设置参数

	string: 'foo'
	integer: 1234
	float: 1234.5
	boolean: true
	list: [1.0, mixed list]
	ictionary: {a: b, c: d}

---
