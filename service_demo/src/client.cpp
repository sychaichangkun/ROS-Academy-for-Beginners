// This is client of the service demo
// 包含必要文件，注意Service文件的包含方式，我们定义的srv文件为Greeting.srv,在包含时需要写成Greeting.h
# include "ros/ros.h"
# include "service_demo/Greeting.h"

int main(int argc, char **argv)
{
	// 初始化，节点命名为"greetings_client"
	ros::init(argc, argv, "greetings_client");
	
	// 定义service客户端，service名字为“greetings”，service类型为Service_demo
	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<service_demo::Greeting>("greetings");
	
	// 实例化srv，设置其request消息的内容，这里request包含两个变量，name和age，见Greeting.srv
	service_demo::Greeting srv;
	srv.request.name = "HAN";
	srv.request.age = 20;

	if (client.call(srv))
	{
		// 注意我们的response部分中的内容只包含一个变量response，另，注意将其转变成字符串
		ROS_INFO("Response from server: %s", srv.response.feedback.c_str());
	}
	else
	{
		ROS_ERROR("Failed to call service Service_demo");
		return 1;
	}
	return 0;
}
