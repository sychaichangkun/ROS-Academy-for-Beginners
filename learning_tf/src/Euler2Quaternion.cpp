#include <ros/ros.h>
#include <tf/tf.h>
//退出用：ctrl+z
int main(int argc, char** argv){
//初始化
  ros::init(argc, argv, "Euler2Quaternion");
  ros::NodeHandle node;
  geometry_msgs::Quaternion q;
  double roll,pitch,yaw;
  while(ros::ok())
  {
  //输入一个相对原点的位置
  std::cout<<"输入的欧拉角：roll,pitch,yaw:";
  std::cin>>roll>>pitch>>yaw;
  //输入欧拉角，转化成四元数在终端输出
  q=tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
  //ROS_INFO("输出的四元数为：w=%d,x=%d,y=%d,z=%d"，q.w,q.x,q.y,q.z);
  std::cout<<"输出的四元数为：w="<<q.w<<",x="<<q.x<<",y="<<q.y<<",z="<<q.z<<std::endl;
  ros::spinOnce();
  }
  return 0;
};


