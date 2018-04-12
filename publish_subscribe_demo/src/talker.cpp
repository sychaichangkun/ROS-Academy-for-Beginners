// ROS 包含的目录.
#include <ros/ros.h>
// 包含计算距离的目录
#include <cmath>
// 包含ROS的msg机制产生的头文件
#include <publish_subscribe_demo/add.h>

int main(int argc, char **argv)
{
  // 初始化
  ros::init(argc, argv, "talker_simple");
  //实例化句柄
  ros::NodeHandle nh;
  //实例化自定义msg
   publish_subscribe_demo::add msg;

  //! 发布消息
  ros::Publisher pub_;

  //! msg第一个数.
  float x=1.0;

  //! msg第二个数
  float y=2.0;

  pub_ = nh.advertise<publish_subscribe_demo::add>("chatter", 10);
  //定义发布的频率 
  ros::Rate loop_rate(1);
 while (ros::ok())
  {
    msg.x = x;
    msg.y = y;
  //线性比例变换，每隔1秒更新一次
    x=1.03*x;
    y=1.01*y;
    ROS_INFO("Talker: GPS : x = %f, y = %f ",  msg.x ,msg.y);
  //计算距离，
    msg.distance= std::sqrt(std::pow((msg.x-1.0),2)+std::pow((msg.y-2.0),2));
  //以1Hz的频率发布msg
    pub_.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
} 

