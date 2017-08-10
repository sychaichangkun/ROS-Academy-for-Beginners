#include <math.h>
#include "std_msgs/Float64.h"
#include "ros/ros.h"
#define M_PI 3.14159265358979323846 
int main(int argc, char **argv)
{
    /* code for main function */
    //初始化节点
    ros::init(argc, argv, "simple_mover_c");
    //实例化句柄
    ros::NodeHandle n;
    //发布话题
    ros::Publisher pub_j1 = n.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command", 10);
    ros::Publisher pub_j2 = n.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command", 10);
    //设置发布的频率
    ros::Rate loop_rate(10);
    //获取当前时间，单位秒
    double start_time = ros::Time::now().toSec();
    //ROS_INFO_STREAM("start_time="<<start_time);
    //设置发布数据
    std_msgs::Float64 frequence;
    //在while循环中发布数据
    while(ros::ok())
    {
        double elapsed = ros::Time::now().toSec()-start_time;
        //ROS_INFO_STREAM("elapsed="<<elapsed);
        frequence.data=frequence.data=sin(2*(M_PI)*elapsed*0.1)*M_PI/2;;
        pub_j1.publish(frequence);
        pub_j2.publish(frequence);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
