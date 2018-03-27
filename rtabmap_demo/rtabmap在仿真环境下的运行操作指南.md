# rtabmap在仿真环境下的运行操作指南
### 操作过程
1. 根据自己的ROS版本,安装对应的rtabmap-ros
 * lunar

		     $ sudo apt-get install ros-lunar-rtabmap-ros
 * 	Kinetic
				
               $ sudo apt-get install ros-kinetic-rtabmap-ros
	
 * 	Jade

			 $ sudo apt-get install ros-jade-rtabmap-ros
 * 	Indigo
				
               $ sudo apt-get install ros-indigo-rtabmap-ros
	
 * 	Hydro

			 $ sudo apt-get install ros-hydro-rtabmap-ros
                      
2. 从源构建,添加依赖
	
    * 首先检查 `~/.bashrc`文件是否包含下面两句:
    	
      	    $ source /opt/ros/kinetic/setup.bash
			$ source ~/catkin_ws/devel/setup.bash
    * 要求安装的依赖:

			$ sudo apt-get install ros-kinetic-rtabmap ros-kinetic-rtabmap-ros
			$ sudo apt-get remove ros-kinetic-rtabmap ros-kinetic-rtabmap-ros

    * 可选择安装的依赖

	包括OpenCV,不同的版本依赖不同,g2o,GTSAM,Freenect2.可根据自己的情况选择安装.具体可参考访问[here!](https://github.com/introlab/rtabmap_ros#rtabmap_ros)
3. 安装RTAM-Map(注意不要直接克隆在catkin 工作空间下)

		$ cd ~
		$ git clone https://github.com/introlab/rtabmap.git rtabmap
		$ cd rtabmap/build
		$ cmake ..  [<---两个点]
		$ make
		$ sudo make install
        
        $ cd ~/catkin_ws
		$ git clone https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros
		$ catkin_make -j1
4. 更新版本

		$ cd rtabmap/build
  	  $ make uninstall
   	 $ sudo make uninstall
     
		$ cd ..
        $ rm -rf build/*
		$ git pull origin master
		$ cd build
        
		$ cmake -DCMAKE_INSTALL_PREFIX=~/catkin_ws/devel ..
		$ cmake ..
        
		$ make
		$ make install
        
		$ roscd rtabmap_ros
		$ git pull origin master
		$ cd ~/catkin_ws
		$ rm -rf build/CMakeCache.txt build/rtabmap_ros
		$ catkin_make -j1 --pkg rtabmap_ros  
        
5. 输入指令,开始仿真建图

		终端1:$ roslaunch robot_sim_demo robot_spawn.launch #启动仿真环境
		终端2:$ roslaunch rtabmap_demo rtabmap_robot_mapping.launch #启动rtabmap
		终端3:$ rosrun robot_sim_demo robot_keyboard_teleop.py #启动键盘控制
	