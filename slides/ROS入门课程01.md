
# ROS 入门课程
## ---ROS的介绍与安装
---
# ROS入门课程--介绍
## 讲师介绍
![图1](https://i.loli.net/2017/08/09/598adc6859f01.png)

---
# ROS入门课程--介绍
## 课程介绍
 《ROS教学系列课程》是睿思学堂推出的ROS学习课程，分为ROS入门和ROS进阶两门课。
### ROS入门课程内容包括：
 * 基本架构
 * 文件系统
 * 常用工具
 * 通信实现
 ---
# ROS入门课程--介绍
 * 坐标转换与模型
 * ROS调试
 * 作业
#### 教学形式：视频讲解+教学课件+代码样例+练习作业
#### 教学目的：旨在为零基础学生提供入门教材，为机器人领域技术人员提供专业化技术参考。
#### 先修要求：基于Linux指令，基于Python/C++
 ---
# ROS入门课程--介绍
## 公司介绍
![图2](https://i.loli.net/2017/08/09/598add9d6c63b.png)           
## 产品介绍
![图3](https://i.loli.net/2017/08/09/598addbfd8583.png)
 
![图4](https://i.loli.net/2017/08/09/598adde9397a0.png)
 
 ---
# ROS入门课程--介绍
### 相关链接
 1.[xros](http://xros.org)
 2.[rosacademy](http://rosacademy.cn)
 3.[ROS](http://wiki.ros.org/Robots/Xbot)
 
 ---
# ROS入门课程--介绍
>很久以前，制造机器人是一项浩大的工程。
![图5](https://i.loli.net/2017/08/09/598adf5531e09.png)
![图6](https://i.loli.net/2017/08/09/598adf88d3995.png)

---
# ROS入门课程--介绍

![图7](https://i.loli.net/2017/08/09/598adfaedd7c6.png)
>随着软硬件快速发展，行业分工更加专业细致，代码复用和模块化需求增加。这样的背景下，2007年，ROS正式诞生，极大的简化了机器人开发的流程。

---
# ROS入门课程--介绍
## 什么是ROS？
>ROS = Robot Operating System

![图8](https://i.loli.net/2017/08/09/598adfd7709b4.jpg)

---
# ROS入门课程--介绍
## ROS特点
* 分布式架构
* 多语言支持
* 精简与集成
* 开源与社区
### 相关链接
* [ros](www.ros.org) 

---
# ROS入门课程--介绍
![图9](https://i.loli.net/2017/08/09/598adffe966a6.png)

---
# ROS入门课程--介绍
## ROS演示---Xbot2机器人slam
### //TODO
---
# ROS入门课程--安装与配置
## 1.安装ROS
* 添加sources.list(建议USTC)
`$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list`
* 添加keys
`$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116`
* 系统升级     
`$ sudo apt-get update && sudo apt-get upgrade`

---
#  ROS入门课程--安装与配置
* 安装ROS

	* Ubuntu 16.04安装Kinetic版本                                                                              
`$ sudo apt-get install ros-kinetic-desktop-full # Ubuntu 16.04` 

	* Ubuntu 14.04安装Lndigo版本         
`$ sudo apt-get install ros-indigo-desktop-full # Ubuntu 14.04`

---
# ROS入门课程--安装与配置
## 2.配置ROS
* 初始化rosdep
`$ sudo rosdep init && rosdep update`
* 导入ROS环境到bash       
`$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc #16.04`
`$ echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc #14.04`

---
# ROS入门课程--安装与配置
* 安装rosinstall
`$ sudo apt-get install python-rosinstall
`
## 3.测试ROS
运行roscore `$ roscore
`

---
# ROS入门课程--安装与配置
![图10](https://i.loli.net/2017/08/09/598ae033b950e.png)

---

# ROS入门课程--Linux基本指令
![图11](https://i.loli.net/2017/08/09/598ae057ead87.png)

---
# ROS入门课程--Python入门
## 相关课程链接
* [菜鸟教程---Python基础教程](http://www.runoob.com/python/python-tutorial.html
)
* [Udacity---编程入门](https://www.udacity.com/course/programming-foundations-with-python--ud036
)


 

 