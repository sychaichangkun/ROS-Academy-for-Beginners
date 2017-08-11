# ROS入门课程
## ---ROS文件系统

---
#ROS入门课程--工作空间
>创建catkin_ws工作空间，并且编译simple-arm 软件包输入指令：

1. `$mkdir  -p catkin_ws/src`
2. `$ cd ~/catkin_ws/src`
3. `$ catkin_init_workspace`
4. `$ git clone https://github.com/sychaichangkun/ROS-Academy-for-Beginners.git`
5. `$ cd ~/catkin_ws`
6. `$ catkin_make`

**注:如果编译出错，请删除kuka_arm后再编译。**

![图1](https://i.loli.net/2017/08/11/598d2b5aa7b74.png)

---
# ROS入门课程--工作空间
>显示工作空间的目录结构

1. `$ cd catkin ws`
2. `$ sudo apt-get install tree`
3. `$ tree-L2`

>src 目录是存放各个package的位置。build目录是build space的默认所在位置。同时cmake和make也是调用和配置并编译你的程序包的位置。devel目录是devel space的默认所在位置，同时也是在你安装程序包之前，存放可执行文件和库文件的地方。

![图2](https://i.loli.net/2017/08/11/598d2d75835a6.png)

---
# ROS入门课程--基本指令
>文件包查找指令：**rospack**

* 最常见指令：**`$rospack find pkg_name`**
* 例如查找simple_arm包，输入指令：**`$rospack find simple_arm`**
* 查看rospack使用帮助，输入指令：**`$rospack help`**

![图3](https://i.loli.net/2017/08/11/598d2f3914ba5.png)

---
# ROS入门课程--基本指令
>进入ros文件目录：**roscd**

* 使用方法：**`$roscd pkg_name`**
* 例如进入rospy_tutorials包目录，输入指令：**`roscd simple_arm`**

**注意使用可以结合Tab键的补全功能**

![图4](https://i.loli.net/2017/08/11/598d31046f2c5.png)

---
# ROS入门课程--基本指令
>列出目录包含的文件和目录名：**rosls**

* 第一种用法：作为建议的ls指令**`$rosls dir_name`**
* 举例：列出主目录下文件和目录**`$rosls`**
* 第二种用法：直接跟ros包名：**`$rosls package_name`**
* 举例：查找simple_arm包，睡指令：**`$rosls simple_arm`**

![图5](https://i.loli.net/2017/08/11/598d32a122b8a.png)

---
# ROS入门课程--基本指令
>以默认编辑器编辑指定ros包下的文件：**rosed**

* 用法：**`$rosed package_name file_name `**
* 举例：修改simple_arm包下的package.xml文件**`$rosed rospy_tutorials package.xml`**

![图6](https://i.loli.net/2017/08/11/598d35276436c.png)

---
# ROS入门课程--基本指令
>运行指定ros包下的launch文件：**roslaunch**

* 用法：**`$roslaunch package file.launch`**
* 运行mypackage包下的mylaunch文件**`$roslaunch simple_arm robot_spawn.launch`**

**注意运行launch文件时不需要运行roscore，默认运行**

![图7](https://i.loli.net/2017/08/11/598d369ef3456.jpg)

---
# ROS入门课程--基本指令
>launch文件时ROS提供的，可以同时运行多个nodes的文件，launch文件作用一直特殊的xml格式编写，在ROS pakage中广泛使用。

举例：simple_arm里面的**robot spawn.launch**

部分文件讲解

1. launch文件中找到gazebo ros 的package，包含empty world.launch**`<include file="$(find gazebo_ros)/launch/empty_world.launch">`**
2. 启动urdf_spawner节点、并且配置相应的参数**`<node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen"`**
**`args="-urdf -param robot_description -x 3.0 -y 0 -z 0 -R 0 -P 0 -Y 0 -model simple_arm"/>`**

![图8](https://i.loli.net/2017/08/11/598d38cc8f978.png)

---
# ROS入门课程--目录结构
>创建ros的package：**catkin create pkg**

* 用法：**`$ catkin_create_pkg <package_name> [depend1] [depend2] [depend3]`**
* 举例：创建mypackage依赖于roscpp rospy std msgs**`$ cd ~/catkin_ws/src`** **`$ catkin_create_pkg mypackage roscpp rospy std_msgs`** **`$ cd ~/catkin_ws`** **`$ catkin_make`**

![图9](https://i.loli.net/2017/08/11/598d3a728587f.png)

---
# ROS入门课程--目录结构
>一个规范的packagea有一下结构：

package：

1. src       ---存放.cpp源文件
2. srv       ---存放.srv文件 设置服务器类型（可选）
3. action    ---存放.action文件 设置actionlib（可选）
4. cfg       ---存放.cfg文件 设置动态调节参数（可选）
5. launch    ---存放launch文件 启动节点以及配置相应参数（可选）
6. msg       ---存放.msg文件 存放数据类型 （可选）
7. include   ---存放.h头文件
8. config    ---存放URDF模型文件，已经其他配置文件 （可选）
9. scripts   ---存放python文件（可选）
10. package.xml
11. cmakeLists.txt

...

**举例：mypackage的结构**

![图10](https://i.loli.net/2017/08/11/598d3bb76336e.png)

---
# ROS入门课程--目录结构
>package.xml提供有关功能包的信息，分成一下四个部分：

1. 描述标签**` <description>The mypackage </description>`**
2. 维护者标签**` <maintainer email="daviahan@todo.todo">davidhan</maintainer> `**
3. 许可标签**`<license>TODO</license>`**
4. 依赖功能包标签**` <buildtool_depend>catkin</buildtool_depend> <build_depend>roscpp</build_depend>`** **`  <build_depend>rospy</build_depend> <build_depend>std_msgs</build_depend>`** **`<run_depend>roscpp</run_depend><run_depend>rospy</run_depend><run_depend>std_msgs</run_depend>`**


![图11](https://i.loli.net/2017/08/11/598d3d3e8be5b.png)

---
# ROS入门课程--目录结构
>cmakelist.txt结构，由以下五个部分：

1. cmake要求的最低版本**`cmake_minimum_required(VERSION 2.8.3)`**
2. package名称**`project(mypackage)`**
3. 编译package需要的依赖项**`find_package(catkin REQUIRED COMPONENTS )`**
4. 生成catkin程序包**`catkin_package()`**
5. 添加所需头文件**`include_directories( ${catkin_INCLUDE_DIRS})`**

![图12](https://i.loli.net/2017/08/11/598d3e87c09a9.png)


**cmake是非常高效的编译工具，如果需要深入了解，建议阅读：《cmake实践》**

---
# ROS入门课程
>**本节完，课后自行练习，并且自行下载simple_arm包并操作课上所讲工具，熟悉工具操作**

* [simple_arm包下载地址](https://github.com/buaaerhan/ROS-Academy-for-Beginners.git)











