# param_demo

param_demo软件包，介绍roscpp、rospy中的parameter service相关的操作。

## 功能介绍



## 运行方法

启动发布者

```sh
$ rosrun param_demo pytalker.py   #Python
$ rosrun param_demo talker        #C++
``` 

启动接收者

```sh
$ rosrun param_demo pylistener.py   #Python
$ rosrun param_demo listener        #C++
``` 

msg是与编程语言无关的通信协议，因此收发双方无论用哪个语言来实现，都可以实现相互的topic通信。
