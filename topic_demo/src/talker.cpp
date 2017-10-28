//ROS头文件
#include <ros/ros.h>
//自定义msg产生的头文件
#include <topic_demo/gps.h>
//ROS标准msg文件
#include <std_msgs/Float32.h>

int main(int argc, char **argv)
{
  //初始化
  ros::init(argc, argv, "talker");

  //实例化句柄
  ros::NodeHandle nh;

  //自定义gps msg
  topic_demo::gps msg;
  msg.x = 1.0;
  msg.y = 1.0;
  msg.state = "working";

  //创建publisher
  ros::Publisher pub;
  pub = nh.advertise<topic_demo::gps>("gps_info", 10);

  //定义发布的频率 
  ros::Rate loop_rate(1);

  //循环发布msg
  while (ros::ok())
  {
    //以指数增长，每隔1秒更新一次
    msg.x = 1.03 * msg.x ;
    msg.y = 1.01 * msg.y;
    ROS_INFO("Talker: GPS: x = %f, y = %f ",  msg.x ,msg.y);
    //以1Hz的频率发布msg
    pub.publish(msg);
    //ros::spinOnce();//用于调用可触发的回调函数，此处不是必需的
    loop_rate.sleep();//根据前面的定义的loop_rate,设置1s的暂停
  }

  return 0;
} 

