# ROS入门课程
## ---ROS基本架构
---

# ROS入门课程--基本架构
> 机器人是一个复杂的系统，由多种传感器、执行机构和处理器组成，需要一个系统来统一管理。

> 以PR2机器人为例，包括十余种传感器、复杂的机械结构、两台服务器。ROS机器人操作，以及如何设计一个系统来管理硬件设备，统筹各个部件，联通不同的模块？参照下图。

---
# ROS入门课程--基本架构
![图1](https://i.loli.net/2017/08/11/598d0f6646362.png)

* [相关链接](http://www.willowgarage.com/pages/pr2/specs)

---
# ROS入门课程--基本架构
## ROS系统架构
* 图状结构
* 主从节点(Master和Nodes)
* 分布式计算
* 点对点
* 松耦合

![图片](https://i.loli.net/2017/08/11/598d10a07b83c.png)

---
# ROS入门课程--基本架构
## Master（节点管理器）：
* 为ROS提供node注册服务
* 维护各node之间的通信
* 启动ROS必须首先启动Master

![图3](https://i.loli.net/2017/08/11/598d118fa0afb.png)

---
# ROS入门课程--基本架构
## Node（节点）：
* 可执行程序，注册到master
* 每个node分工不同
* 单独编译、执行和管理

**启动node** `>rosrun pkg_name node_name `   
**查看当前运行的node** `>rosnode list`        
**查看node信息** `>rosnode info node_name ` 
    
![图4](https://i.loli.net/2017/08/11/598d136bd5e12.png)

---

# ROS入门课程--基本架构
## 示例1：
* 下载ros_tutorials包 `>sudo apt install ros-kinetic-ros-tutorials `      
* 启动roscore `>roscore`

![图5](https://i.loli.net/2017/08/11/598d14566d459.png)

---
# ROS入门课程--基本架构
## 示例2：
* 启动turtlesim `>rosrun turtlesim turtlesim_node`
* 启动turtle teleop key`>rosrun turtlesimturtle_teleop_key`
> **问题：控制小海龟移动，试想两个node之间是如何实现通信的？**

![图6](https://i.loli.net/2017/08/11/598d15b1a3d01.png)

---
# ROS入门课程--基本架构
## Topic（主题）：
* node之间进行通信的一种形式
* node可以发布和订阅topic
* 一个topic可以由一个node发布和多个node接受

查看当前所有的topic`>rostopic list`
订阅并查看topic中的内容`>rostopic echo / topic_name`
显示topic相关的信息`>rostopic info/topic_name`

![图7](https://i.loli.net/2017/08/11/598d1776245ce.png)

---
# ROS入门课程--基本架构
## Message（消息）：
* topic内容的数据结构（格式标准）
* msg是C++数据类型的**任意组合**
* 定义在*.msg文件中

查看msg的类型定义`>rosmsg show msg_type`
查看本地多有定义的msg`rosmsg list`

![图8](https://i.loli.net/2017/08/11/598d19586f676.png)

---
# ROS入门课程--基本架构
## 示例3：
* 查看当前所有的topic`>rostopic list`
* 查看/turtle1/cmd_vel的内容`>rostopic echo/turtle1/cmd_vel`
* 显示topic的相关信息`>rostopic info/turtle1/cmd_vel`

![图9](https://i.loli.net/2017/08/11/598d1a53bb69b.png)

---
# ROS入门课程--基本架构
## 示例4：
* 查看当前所有的topic`>rosmsg show geometry_msgs/Twist`
* 查看本地所有的msg`>rosmsg list`

---
# ROS入门课程--基本架构
## 使用rqt_graph查看计算图：
* 可视化调试工具
* 图形化显示当前ROS网络结构
* 反应节点关系和消息流动

**启动rqt_graph`>rqt_graph`**

![图10](https://i.loli.net/2017/08/11/598d1bb2db117.png)

---
# ROS入门课程--基本架构
## 总结：
>**ROS能够创建一个连接各进程（node）的抽象网络。该网络通过节点管理器（master）维护。节点之间以主题（topic）、服务（service）、参数（param）形式通信。通信的标准类型为消息（message）。**

![图11](https://i.loli.net/2017/08/11/598d1cc2ac22e.png)

---
# ROS入门课程--基本架构
## ROS指令：
* roscore
* rosrun
* rosnode
* rostopic
* rosmsg
* rqt_graph

---
# ROS入门课程--基本架构
**使用ROS，将各元器件驱动和算法节点化，封装成多个node。pr2_bringup启动的node：**

![图12](https://i.loli.net/2017/08/11/598d1fb00ee40.png)

---
# ROS入门课程--基本架构
**本节完，作业要求：练习**