// ROS 包含的目录.
#include <ros/ros.h>
#include <string>
// 包含ROS的msg机制产生的头文件
#include <topic_demo/gps.h>
#include <std_msgs/Float32.h>
//ROS当中
int main(int argc, char **argv)
{
  // 初始化
  ros::init(argc, argv, "talker");
  //实例化句柄
  ros::NodeHandle nh;
  //实例化自定义msg
   topic_demo::gps msg;
  //! 发布消息
  ros::Publisher pub_;
  //! msg 的x值.
  std_msgs::Float32 x;
  //! msg 的y值
  std_msgs::Float32 y;
  std::string state("working");
  x.data=1.0000;
  y.data=1.0000;
  //! msg 的状态
  pub_ = nh.advertise<topic_demo::gps>("chatter", 10);
  //定义发布的频率 
  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    msg.x = x.data;
    msg.y = y.data;
  //线性比例变换，每隔1秒更新一次
    x.data=1.03*x.data;
    y.data=1.01*y.data;
    msg.state=state.c_str();
    ROS_INFO("Talker: GPS: x = %f, y = %f ",  msg.x ,msg.y);
  //以1Hz的频率发布msg
    pub_.publish(msg);
    ros::spinOnce();//不是必需的，但是保持增加这个调用，是好习惯。
    loop_rate.sleep();//根据前面的定义的loop_rate,设置1s的暂停
  }
  return 0;
} 

