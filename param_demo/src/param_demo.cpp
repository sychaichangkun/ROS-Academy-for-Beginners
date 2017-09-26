#include<ros/ros.h>

int main(int argc, char **argv){
	ros::init(argc, argv, "param_demo");
	ros::NodeHandle nh;
	int init_parameter;
	ros::param::get("/Param_demo", init_parameter);
	//ros::param::set("/Param_demo",init_parameter);
	ros::Rate rate(1);

	while(ros::ok()){
		int parameter = 0;
		ros::param::get("/Param_demo", parameter);
		if(parameter!=init_parameter){
			ROS_INFO("You've changed parameter /Param_demo from %d to %d", init_parameter, parameter);
			init_parameter = parameter;
		}
		ROS_INFO("parameter /Param_demo = %d", parameter);

		rate.sleep();
	}
}
