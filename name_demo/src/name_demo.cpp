#include <ros/ros.h>

int main(int argc, char* argv[])
{
    int serial_number = -1;//serial_number初始化
    ros::init(argc, argv, "name_demo");//node初始化
    /*创建命名空间*/
    //n 是全局命名空间
    ros::NodeHandle n;
    //nh 是局部命名空间
    ros::NodeHandle nh("~");
    /*全局命名空间下的Param*/
    ROS_INFO("global namespace");
    //提取全局命名空间下的参数serial
    n.getParam("serial", serial_number);
    ROS_INFO("global_Serial was %d", serial_number);
    //提取局部命名空间下的参数serial
    n.getParam("name_demo/serial", serial_number);//在全局命名空间下，要提取局部命名空间下的参数，需要添加node name
    ROS_INFO("global_to_local_Serial was %d", serial_number);
    /*局部命名空间下的Param*/
    ROS_INFO("local namespace");
    //提取局部命名空间下的参数serial
    nh.getParam("serial", serial_number);
    ROS_INFO("local_Serial was %d", serial_number);
    //提取全局命名空间下的参数serial
    nh.getParam("/serial", serial_number);//在局部命名空间下，要提取全局命名空间下的参数，需要添加“/”
    ROS_INFO("local_to_global_Serial was %d", serial_number);
    ros::spin();
    return 0;
}

