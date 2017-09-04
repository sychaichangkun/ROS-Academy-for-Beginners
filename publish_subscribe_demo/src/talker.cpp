// ROS 包含的目录.
#include <ros/ros.h>
#include <string>

// 包含ROS的msg机制产生的头文件
#include <publish_subscribe_demo/gps.h>

int main(int argc, char **argv)
{
  // 初始化
  ros::init(argc, argv, "talker");
  //实例化句柄
  ros::NodeHandle nh;
  //实例化自定义msg
   publish_subscribe_demo::gps msg;

  //! 发布消息
  ros::Publisher pub_;

  //! msg 的x值.
  float x=1.0;


  //! msg 的y值
  float y=2.0;
 
  //! msg 的状态
  std::string state("working");

  pub_ = nh.advertise<publish_subscribe_demo::gps>("chatter", 10);
  //定义发布的频率 
  ros::Rate loop_rate(1);


 while (ros::ok())
  {
    msg.x = x;
    msg.y = y;
  //线性比例变换，每隔1秒更新一次
    x=1.03*x;
    y=1.01*y;
    msg.state=state.c_str();
    ROS_INFO("Talker: GPS: x = %f, y = %f ",  msg.x ,msg.y);
  //以1Hz的频率发布msg
    pub_.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
} 

