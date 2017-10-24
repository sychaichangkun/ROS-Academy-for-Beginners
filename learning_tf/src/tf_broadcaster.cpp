#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
//退出用：ctrl+z
int main(int argc, char** argv){
//初始化
  ros::init(argc, argv, "tf_broadcaster");
  ros::NodeHandle node;
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  geometry_msgs::Quaternion qw;
  tf::Quaternion q;
  double roll,pitch,yaw,x,y,z;
  while(ros::ok())
  {
  //输入一个相对原点的位置
  std::cout<<"输入相对于原点的位置：";
  std::cin>>x>>y>>z;
  std::cout<<"输入的欧拉角：roll,pitch,yaw:";
  std::cin>>roll>>pitch>>yaw;
  //输入欧拉角，转化成四元数在终端输出
  q.setRPY(roll,pitch,yaw);
  qw=tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
  transform.setOrigin(tf::Vector3(x,y,z));
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"base_link","link1"));
  std::cout<<"输出的四元数为：w="<<qw.w<<",x="<<qw.x<<",y="<<qw.y<<",z="<<qw.z<<std::endl;
  //ROS_INFO("输出的四元数为：w=%d,x=%d,y=%d,z=%d"，q.w,q.x,q.y,q.z);
  ros::spinOnce();
  }
  return 0;
};


