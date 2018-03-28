# tf_demo

 tf相关API操作演示，tf示例包括C++和python两个版本

## 功能介绍

本例提供欧拉角和四元数的互换，tf常见坐标变换，监听和发布坐标变换的相关API介绍

在`src/`下的的[Euler2Quaternion.cpp](./src/Euler2Quaternion.cpp)和[Quaternion2Euler.cpp](./src/Quaternion2Euler.cpp)分别是欧拉角到四元数和四元数到欧拉角之间的转换。[coordinate_transformation.cpp](./src/coordinate_transformation.cpp)提供的是cpp版本的tf中常见的坐标变换API。[tf_broadcaster.cpp](./src/tf_broadcaster.cpp)和[tf_listerner.cpp](./src/tf_listerner.cpp)分别提供cpp版本的发布坐标变换和监听坐标变换的功能

在`scripts/`下的[py_coordinate_transformation.py](./scripts/py_coordinate_transformation.py)提供python版本的tf中常见坐标变换API。[py_tf_broadcaster.py](./scripts/py_tf_broadcaster.py)和
[py_tf_broadcaster02.py](./scripts/py_tf_broadcaster02.py)提供了两种python版本的发布坐标变化方法。
[py_tf_listerner.py](./scripts/py_tf_listerner.py)提供了python版本的监听坐标变换的方法




## 运行方法

坐标变换API

```sh
$ rosrun tf_demo py_coordinate_transformation.py   #Python
$ rosrun tf_demo coordinate_transformation           #C++
``` 

欧拉角和四元数之间的互换

```sh
$  rosrun tf_demo Euler2Quaternion   #C++
$  rosrun tf_demo Quaternion2Euler   #C++
``` 

发布坐标变换

```sh
$ rosrun tf_demo py_tf_broadcaster02.py #Python
$ rosrun tf_demo py_tf_broadcaster.py   #Python
$ rosrun tf_demo tf_broadcaster         #C++
``` 
监听坐标变换

```sh
$ rosrun tf_demo py_tf_listerner.py   #Python
$ rosrun tf_demo tf_listerner          #C++
``` 
