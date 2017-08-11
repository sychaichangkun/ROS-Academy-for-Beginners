# ROS入门教程
## ---ROS常用工具介绍
---
# ROS常用工具--rqt_plot
**作用：图形化显示发布在topic上面的消息数值**

首先确保已经安装此工具（一般默认已安装）：   
**`$ sudo apt-get install ros-indigo-rqt`**

运行方式：**`$rqt_plot`**

上面命令执行结果如下图所示：

1. Topic：表示要显示的Topic数据
2. Topic输入框
3. 初始为灰色不可用，输入完topic之后变成绿色，点击“+”即可
4. 去除某个topic，即隐藏
5. 编辑显示界面参数，如x,y轴的范围等
6. 数据显示界面

![图1](https://i.loli.net/2017/08/11/598d54818d3b3.jpg)

---
# ROS常用工具--rqt_plot
**应用举例：**下载并编译simple_arm软件包:
**`$ cd ~/catkin_ws/src`** **`$ git clone https://github.com/buaaerhan/ROS-Academy-for-Beginners.git`**
**`$ cd ~/catkin_ws`** **`$ catkin_make`**

**注：**如果编译出错，请删除kuka-arm包后再编译运行simple-arm中的可执行文件：**`$ roslaunch simple_arm robot_spawn.launch`** **`$ rosrun simple_arm simple_mover$ rqt_plot`**

运行结果如果，由于未添加topic，当前界面空白

![图2](https://i.loli.net/2017/08/11/598d579f5b831.png)

---
# ROS常用工具--rqt_plot
**举例：**在界面上显示topic数据：

* 在topic输入框中输入要显示的topic，有自动提示

![图3](https://i.loli.net/2017/08/11/598d582fd7c1d.png)

* 在补充完整或者选择好topic之后，“+”变成绿色，点击“+”，即可显示图形

![图4](https://i.loli.net/2017/08/11/598d58d547b01.png)

* 调整坐标轴范围，最终图形显示结果：

![图5](https://i.loli.net/2017/08/11/598d5928bfbe5.png)

---
# ROS常用工具--rqt_graph
**用途：**图形化显示ros节点、topic等之间的关系

运行方式：**`$rqt_graph`**

仍以上面的simple_arm包为例，编译并运行**`$ roslaunch simple_arm robot_spawn.launch $ rosrun simple_arm simple_mover $ rqt_graph`**

运行结果如下图：

![图6](https://i.loli.net/2017/08/11/598d5a315e2b6.png)

---
# ROS常用工具--rqt_graph
**调整显示内容：**

![图7](https://i.loli.net/2017/08/11/598d5b03425b2.png)

---
# ROS常用工具--rqt_console
**用途：**显示节点输出的详细信息，相当于运行日志

运行方式：**`$rqt_console`**

仍以上面的simple_arm包为例，编译并运行

**`$ roslaunch simple_arm robot_spawn.launch $ rosrun simple_arm simple_mover $ rqt_console`**

当用Ctrl+c终止launch程序时，结果如下图：

![图8](https://i.loli.net/2017/08/11/598d5c0942984.png)

---
# ROS常用工具--gazebo
**用途：**建立并显示仿真模型

运行方式：**`$gazebo`**

显示结果为空白场景，可添加模型，Insert **->**Cafe，等待加载完成，即可以看到如下图所示：

1. World：显示当前场景
2. Insert:可加载的场景模型，一部分是自定义模型，另一部分是gazebo官方提供的场景
3. layers:暂时用不到

![图9](https://i.loli.net/2017/08/11/598d5d55decd5.png)

---
# ROS常用工具--gazebo
**用途：**建立并显示仿真模型

运行方式：**`$gazebo`**

运行simple_arm，会自动调用gazebo仿真
**`$ roslaunch simple_arm robot_spawn.launch $ rosrun simple_arm simple_mover`**

显示结果如图所示：

![图10](https://i.loli.net/2017/08/11/598d5e231be22.png)

---
# ROS常用工具--Rviz
**作用：ros中各机器人模型、传感器数据等的3D可视化**

运行方式：**`$rviz`**

在运行前面的simple_arm包中两个可执行文件的基础上，运行rviz，结果如下图：

**`$ roslaunch simple_arm robot_spawn.launch $ rosrun simple_arm simple_mover $ rviz`**

但出现错误，且未能显示机器人模型，why?

![图11](https://i.loli.net/2017/08/11/598d5f3456a15.png)

---
# ROS常用工具--Rviz
**问题：左侧显示出现错误，且机器人模型无法显示**

解决办法：

1. 将Fixed Frame的值改为base_link或者world，以解决出错问题
2. 在左侧底部点击Add，添加机器人模型，显示机器人模型：Add**->**RobotModel
3. 同时，也可以添加其他想要显示的元素，如TF坐标转换关系示意图等

![图12](https://i.loli.net/2017/08/11/598d6078281cd.png)

---
# ROS常用工具--Rviz
**注意：在Add添加一些传感器数据如Image等时，需要对应的机器人模型带有相应的传感器。**

* simple_arm带相机，可添加image，获取并显示图像信息，Add**->**Image，出现下图所示警告，且Image无法显示。

![图13](https://i.loli.net/2017/08/11/598d6208b0c8d.png)

* 原因：Image没有正确注册到相应的topic
* 解决办法：更改Image Topic为/rgb-camera/image-raw

结果如下图

![图14](https://i.loli.net/2017/08/11/598d6228cd750.png)

---
# ROS常用工具介绍
**练习：自行下载simple_arm包并操作课上所讲工具，并熟悉工具操作**

* [simple_arm包下载](https://github.com/buaaerhan/ROS-Academy-for-Beginners.git)

---




