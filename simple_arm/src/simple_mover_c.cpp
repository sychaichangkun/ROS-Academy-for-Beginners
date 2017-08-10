#include <math.h>
#include "std_msgs/Float64.h"
#include "ros/ros.h"
#define M_PI 3.14159265358979323846 
int main(int argc, char **argv)
{
    /* code for main function */
    ros::init(argc, argv, "simple_mover_c");
    ros::NodeHandle n;
    ros::Publisher pub_j1 = n.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command", 10);
    ros::Publisher pub_j2 = n.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command", 10);
    ros::Rate loop_rate(10);
    double start_time = ros::Time::now().toSec();
    //ROS_INFO_STREAM("start_time="<<start_time);
    std_msgs::Float64 frequence;
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
