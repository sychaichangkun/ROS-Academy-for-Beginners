// This is client of the service demo
//
//
//

# include "ros/ros.h"
# include "service_client_demo/Service_demo.h"
# include <cstdlib>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "greetings_client");
	
	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<service_client_demo::Service_demo>("greetings");
	
	service_client_demo::Service_demo srv;
	srv.request.name = "HAN";
	srv.request.age = 20;

	if (client.call(srv))
	{
		ROS_INFO("Response from server: %s", srv.response.response.c_str());
	}
	else
	{
		ROS_ERROR("Failed to call service Service_demo");
		return 1;
	}
	return 0;
}
