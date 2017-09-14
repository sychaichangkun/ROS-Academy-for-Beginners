// ROS 包含的目录.
#include <ros/ros.h>
// 包含计算距离的目录
#include <cmath>
//包含ROS的msg机制产生的头文件
#include <topic_demo/gps.h>
void chatterCallback( const topic_demo::gps::ConstPtr &msg)
{  
    //计算离原点的距离
    float x=msg->x;
    float y=msg->y;
    float distance = std::sqrt(std::pow((x-1.0),2)+std::pow((y-2.0),2));
    ROS_INFO("Listener: GPS: distance = %f, state: %s",distance,msg->state.c_str());
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::spin();
  return 0;
}

