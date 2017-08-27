## 启动文件

# 启动文件

ROS中的启动文件在用户和开发人员都非常普遍。它们提供了启动多个节点和主节点的便捷方式，以及其他初始化要求，如设置参数。

## ROSLAUNCH

**roslaunch** 用于打开启动文件。这可以通过指定包含启动文件的包，然后指定启动文件的名称，或通过指定启动文件的文件路径来完成。

	roslaunch package_name launch_file
	roslaunch〜/.../.../.../ launch_file

注意：如果没有设置主设备，roslaunch也将启动roscore。在运行启动文件的终端中按Ctrl-C将关闭所有启动文件启动的节点。

## 编写一个.LAUNCH文件

启动文件格式为.launch，并使用特定的XML格式。它们可以放置在包目录中的任何位置，但通常在工作区目录中创建一个名为“Launch”的目录来组织所有启动文件。启动文件的内容必须包含在一对启动标签之间

	<launch> ... </ launch>

要实际启动一个节点，使用<node>标签，需要pkg，type和name参数。

	<node pkg =“...”type =“...”name =“...”respawn = true ns =“...”/>

**pkg/type/name** ：参数pkg指向与要启动的节点相关联的包，而“type”是指节点可执行文件的名称。也可以使用name参数覆盖节点的名称，这将优先于给定给代码节点的名称。

**Respawn/Required**：但是可选，通常有一个respawn参数或必需的参数，但不是两者。如果respawn = true，则由于某种原因关闭，此特定节点将重新启动。Required = true会做相反的操作，也就是说，如果该特定节点关闭，它将关闭与启动文件关联的所有节点。[ROS wiki](http://wiki.ros.org/roslaunch/XML/node)上还有其他可选参数。

**ns** ：启动文件的另一个常见用途是在命名空间内启动一个节点。当使用同一个节点的多个实例时，这是非常有用的。您可以使用“ns”参数指定名称空间。

**arg** ：有时在启动文件中需要使用局部变量。这可以使用

	<arg name =“...”value =“...”>

现在，我们来看看我们在启动时在我们的赫斯基板载PC上使用的启动文件，以使事情发生。

	<launch>
		<arg name="port" default="$(optenv HUSKY_PORT /dev/prolific)" />
		<node pkg="clearpath_base" type="kinematic_node" name="husky_kinematic" ns="husky">
			<param name="port" value="$(arg port)" />
			<rosparam>
				cmd_fill: True
				 data:
					system_status: 10
					safety_status: 10
					encoders: 10
				differential_speed: 10
				differential_output: 10
					 power_status: 1
					</rosparam>
				 </node>
		 <!-- Publish diagnostics information from low-level MCU outputs -->
		 <node pkg="husky_base" name="husky_base_diagnostics" type="diagnostics_publisher" />
		<!-- Publish wheel odometry from MCU encoder data -->
		<node pkg="husky_base" name="husky_basic_odom" type="basic_odom_publisher" />

		 <!-- Diagnostic Aggregator -->
		<node pkg="diagnostic_aggregator" type="aggregator_node" name="diagnostic_aggregator">
		<rosparam command="load" file="$(find husky_base)/config/diagnostics.yaml"/>
		 </node>
	</launch>

首先要注意的是所有启动文件所需的<launch>标签。下一行找到Husky连接的端口，并将其保存到名为“port”的参数。包“clearpath_base”中的节点“kinematic_node”然后在“哈斯基”的名称空间中启动。

<node>标签中的参数对该标记是私有的。前面定义的“端口”参数设置为端口参数。使用具有<parameter>标签的YAML格式填充其他几个参数。

与kinematic_node节点一起，此启动文件还启动了husky_base_diagnositcs和husky_base_odom。您可以看到，从YAML文件加载了diagnostics_aggregator节点的参数。

这应该涵盖大部分您需要编写自己的启动文件，但是有关启动文件的更多信息，请访问[ROS wiki](http://wiki.ros.org/roslaunch/XML/node)。

---
