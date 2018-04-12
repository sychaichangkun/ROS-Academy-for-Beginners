// ROS 包含的目录.
#include <ros/ros.h>

//包含ROS的msg机制产生的头文件
#include <publish_subscribe_demo/add.h>
void chatterCallback(  const publish_subscribe_demo::add::ConstPtr &msg)
{  
    ROS_INFO("Listener: GPS : x = %f, y = %f ,distance = %f",  msg->x ,msg->y,msg->distance);
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::spin();
  return 0;
}

