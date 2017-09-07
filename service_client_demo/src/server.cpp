// This is the C++ version server file of the service demo
//
//
//
//
//

# include "ros/ros.h"
# include "service_client_demo/Service_demo.h"
# include<string>

bool handle_function(service_client_demo::Service_demo::Request &req,
					service_client_demo::Service_demo::Response &res)
{
	ROS_INFO("Hi server,I'm client, my name is %s and I'm %ld years old!", req.name.c_str(), req.age);
	res.response = "Hi client, I'm server!";
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "greetings_server");
	ros::NodeHandle nh;

	ros::ServiceServer service = nh.advertiseService("greetings", handle_function);
	ros::spin();

	return 0;
}

