// This is client of the service demo

// 包含必要文件，注意Service文件的包含方式，我们定义的service文件为Service_demo.srv,但在包含时需要写成Service_demo.h
# include "ros/ros.h"
# include "service_demo/Service_demo.h"
# include <cstdlib>

int main(int argc, char **argv)
{
	// 初始化节点，命名为"greetings_client"
	ros::init(argc, argv, "greetings_client");
	
	// 定义service客户端，service名字为“greetings”，service类型为Service_demo
	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<service_demo::Service_demo>("greetings");
	
	// 实例化service，并设置其request消息的内容，这里request包含两个变量，name和age，见Service_demo.srv
	service_demo::Service_demo srv;
	srv.request.name = "HAN";
	srv.request.age = 20;

	if (client.call(srv))
	{
		// 注意我们的response部分中的内容只包含一个变量response，另，注意将其转变成字符串
		ROS_INFO("Response from server: %s", srv.response.response.c_str());
	}
	else
	{
		ROS_ERROR("Failed to call service Service_demo");
		return 1;
	}
	return 0;
}
