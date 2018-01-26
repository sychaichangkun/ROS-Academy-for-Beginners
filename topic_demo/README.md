# topic_demo

topic_demo软件包，包括C++与Python两个版本的Topic通信的示例。

## 功能介绍

假设Topic的发布者为GPS模块，它以**1HZ**的频率向**/gps_info**这个topic上发布消息，消息格式要包括坐标(x,y)和工作状态(state)。

Topic的接受者会订阅**/gps_info**，并计算每次GPS位置到原点的距离，在屏幕上显示。

本例需要自定义msg文件，见[msg/gps.msg](./msg/gps.msg)。

C++版本代码见`src/`下的[talker.cpp](./src/talker.cpp)和[listener.cpp](./src/listener.cpp)。

Python版本代码见`scripts/`下的[pytalker.py](./scripts/pytalker.py)和[pylistener.py](./scripts/pylistener.py)。


## 运行方法

启动发布者

```sh
$ rosrun topic_demo pytalker.py   #Python
$ rosrun topic_demo talker        #C++
``` 

启动接收者

```sh
$ rosrun topic_demo pylistener.py   #Python
$ rosrun topic_demo listener        #C++
``` 

msg是与编程语言无关的通信协议，因此收发双方无论用哪个语言来实现，都可以实现相互的topic通信。
