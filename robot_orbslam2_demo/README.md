# 运行ORB_SLAM2
1. 首先应该安装openCV,注意测试版本,V2.4到V3.2.openCV的安装有些繁琐,需要安装各种依赖,否则会出现问题.建议按照参考教程完成安装并测试.
2. 安装Pangolin,需要进行初始化编译.
3. 下载orb_slam2源码包,官网github上源码包含了DBoW2,和g2o.另需要下载Eigen3.1
4. 对orb_slam2进行编译构建.
5. 从本课程官网下载代码包,(如果已经下载可忽略)将/src/robot_orbslam2_demo/param/下的Zdzn.yaml文件拷贝到ORB_SLAM2/Examples/ROS/ORB_SLAM2下，并修改.bashrc文件．
6. 执行语句，进行仿真环境下建图．

** Tips: ** 具体步骤参考－＞运行包详细步骤.md