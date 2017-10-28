//ROS头文件
#include <ros/ros.h>
//包含自定义msg产生的头文件
#include <topic_demo/gps.h>

void chatterCallback(const topic_demo::gps::ConstPtr &msg)
{  
    //计算离原点的距离+pow(msg->y,2)
    float distance = sqrt(pow(msg->x,2));
    ROS_INFO("Listener: Distance to origin = %f, state: %s",distance,msg->state.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("gps_info", 10, chatterCallback);
  //ros::spin()用于调用所有可触发的回调函数。将进入循环，不会返回，类似于在循环里反复调用ros::spinOnce()。
  ros::spin(); 
  return 0;
}

