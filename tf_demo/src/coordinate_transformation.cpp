#include <ros/ros.h>
#include <tf/tf.h>
//退出用：ctrl+z
int main(int argc, char** argv){
//初始化
ros::init(argc, argv, "coordinate_transformation");
ros::NodeHandle node;
tf::Vector3 v1(1,1,1);
tf::Vector3 v2(1,0,1);
//第1部分，定义空间点和空间向量
std::cout<<"第1部分，定义空间点和空间向量"<<std::endl;
//1.1 计算两个向量的点积
std::cout<<"向量v1:"<<"("<<v1[0]<<","<<v1[1]<<","<<v1[2]<<"),";
std::cout<<"向量v2:"<<"("<<v2[0]<<","<<v2[1]<<","<<v2[2]<<")"<<std::endl;
std::cout<<"两个向量的点积："<<tfDot(v1,v2)<<std::endl;
//1.2 计算向量的模
std::cout<<"向量v2的模值:"<<v2.length()<<std::endl;
//1.3 求与已知向量同方向的单位向量
tf::Vector3 v3;
v3=v2.normalize();
std::cout<<"与向量v2的同方向的单位向量v3:"<<"("<<v3[0]<<","<<v3[1]<<","<<v3[2]<<")"<<std::endl;
//1.4 计算两个向量的夹角
std::cout<<"两个向量的夹角(弧度):"<<tfAngle(v1,v2)<<std::endl;
//1.5 计算两个向量的距离
std::cout<<"两个向量的距离:"<<tfDistance2(v1,v2)<<std::endl;
//1.6 计算两个向量的乘积
tf::Vector3 v4;
v4=tfCross(v1,v2);
std::cout<<"两个向量的乘积v4:"<<"("<<v4[0]<<","<<v4[1]<<","<<v4[2]<<")"<<std::endl;
//第2部分，定义四元数
std::cout<<"第2部分，定义四元数"<<std::endl;
//2.1 由欧拉角计算四元数
tfScalar yaw,pitch,roll;
yaw=0;pitch=0;roll=0;
std::cout<<"欧拉角rpy("<<roll<<","<<pitch<<","<<yaw<<")";
tf::Quaternion q;
q.setRPY(yaw,pitch,roll);
std::cout<<"，转化到四元数q:"<<"("<<q[3]<<","<<q[0]<<","<<q[1]<<","<<q[2]<<")"<<std::endl;
//2.2 由四元数得到旋转轴
tf::Vector3 v5;
v5=q.getAxis();
std::cout<<"四元数q的旋转轴v5"<<"("<<v5[0]<<","<<v5[1]<<","<<v5[2]<<")"<<std::endl;
//2.3 由旋转轴和旋转角来估计四元数
tf::Quaternion q2;
q2.setRotation(v5,1.570796);
std::cout<<"旋转轴v5和旋转角度90度，转化到四元数q2:"<<"("<<q2[3]<<","<<q2[0]<<","<<q2[1]<<","<<q2[2]<<")"<<std::endl;
//第3部分，定义旋转矩阵
std::cout<<"第3部分，定义旋转矩阵"<<std::endl;
//3.1 由旋转轴和旋转角来估计四元数
tf::Matrix3x3 Matrix;
tf::Vector3 v6,v7,v8;
Matrix.setRotation(q2);
v6=Matrix[0];
v7=Matrix[1];
v8=Matrix[2];
std::cout<<"四元数q2对应的旋转矩阵M:"<<v6[0]<<","<<v6[1]<<","<<v6[2]<<std::endl;
std::cout<<"                       "<<v7[0]<<","<<v7[1]<<","<<v7[2]<<std::endl;
std::cout<<"                       "<<v8[0]<<","<<v8[1]<<","<<v8[2]<<std::endl;
//3.2 通过旋转矩阵求欧拉角
tfScalar m_yaw,m_pitch,m_roll;
Matrix.getEulerYPR(m_yaw,m_pitch,m_roll);
std::cout<<"由旋转矩阵M,得到欧拉角rpy("<<m_roll<<","<<m_pitch<<","<<m_yaw<<")"<<std::endl;
return 0;
};
