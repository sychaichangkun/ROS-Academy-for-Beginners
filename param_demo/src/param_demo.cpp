#include<ros/ros.h>

int main(int argc, char **argv){
	ros::init(argc, argv, "param_demo");
	ros::NodeHandle nh;
	int parameter1, parameter2, parameter3, parameter4, parameter5;
	
	//Get Param的三种方法
	//① ros::param::get()获取参数“param1”的value，写入到parameter1上
	bool ifget1 = ros::param::get("param1", parameter1);
	
	//② ros::NodeHandle::getParam()获取参数，与①作用相同
	bool ifget2 = nh.getParam("param2",parameter2);
	
	//③ ros::NodeHandle::param()类似于①和②
	//但如果get不到指定的param，它可以给param指定一个默认值(如33333)
        nh.param("param3", parameter3, 33333);
	
	if(ifget1)
		ROS_INFO("Get param1 = %d", parameter1);
	else
		ROS_WARN("Didn't retrieve param1");
	if(ifget2)
		ROS_INFO("Get param2 = %d", parameter2);
	else
		ROS_WARN("Didn't retrieve param2");
	if(nh.hasParam("param3"))
		ROS_INFO("Get param3 = %d", parameter3);
	else
		ROS_WARN("Didn't retrieve param3");


    //Set Param的两种方法
	//① ros::param::set()设置参数
	parameter4 = 4;
	ros::param::set("param4", parameter4);

	//② ros::NodeHandle::setParam()设置参数
	parameter5 = 5;
	nh.setParam("param5",parameter5);
	
	ROS_INFO("Param4 is set to be %d", parameter4);
	ROS_INFO("Param5 is set to be %d", parameter5);


	//Check Param的两种方法
	//① ros::NodeHandle::hasParam()
	bool ifparam5 = nh.hasParam("param5");

	//② ros::param::has()
	bool ifparam6 = ros::param::has("param6");

	if(ifparam5) 
		ROS_INFO("Param5 exists");
	else
		ROS_INFO("Param5 doesn't exist");
	if(ifparam6) 
		ROS_INFO("Param6 exists");
	else
		ROS_INFO("Param6 doesn't exist");


	//Delete Param的两种方法
	//① ros::NodeHandle::deleteParam()
	bool ifdeleted5 = nh.deleteParam("param5");

	//② ros::param::del()
	bool ifdeleted6 = ros::param::del("param6");
	

	if(ifdeleted5)
		ROS_INFO("Param5 deleted");
	else
		ROS_INFO("Param5 not deleted");
	if(ifdeleted6)
		ROS_INFO("Param6 deleted");
	else
		ROS_INFO("Param6 not deleted");


	ros::Rate rate(0.3);
	while(ros::ok()){
		int parameter = 0;
		
		ROS_INFO("=============Loop==============");
		//roscpp中尚未有ros::param::getallparams()之类的方法
		if(ros::param::get("param1", parameter))
			ROS_INFO("parameter param1 = %d", parameter);
		if(ros::param::get("param2", parameter))
			ROS_INFO("parameter param2 = %d", parameter);
		if(ros::param::get("param3", parameter))
			ROS_INFO("parameter param3 = %d", parameter);
		if(ros::param::get("param4", parameter))
			ROS_INFO("parameter param4 = %d", parameter);
		if(ros::param::get("param5", parameter))
			ROS_INFO("parameter param5 = %d", parameter);
		if(ros::param::get("param6", parameter))
			ROS_INFO("parameter param6 = %d", parameter);
		rate.sleep();
	}
}
