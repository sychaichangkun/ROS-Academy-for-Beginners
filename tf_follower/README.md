# tf_follower

制作mybot机器人 实现mybot跟随xbot的功能

## 功能介绍

制作mybot的gazebo模型，在`mybot_control`中提供左右轮的PID控制参数
详见[mybot_control.yaml](./mybot_control/config/mybot_control.yaml) 。在`mybot_description`中提供mybot的urdf模型，tf_tree以及gazebo插件的相关参数，详见[mybot.xacro](./mybot_description/urdf/mybot.xacro) 。在`scripts`文件夹中的`py_tf_follower.py`订阅`mybot_link`和`base_footprint`并且发布`/mybot_cmd_vel`实现mybot跟随xbot功能。当mybot与xbot距离小于1米的时候，mybot停止移动。


## 运行方法

在gazebo中启动xbot和mybot

```sh
$ roslaunch tf_follower robot_spawn.launch
``` 

启动跟随

```sh
$ rosrun tf_follower py_tf_follower.py  
