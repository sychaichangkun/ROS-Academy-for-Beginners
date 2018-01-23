#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <turtlesim/Spawn.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "car_broadcaster");

	ros::NodeHandle node;

	//ros::service::waitForService("spawn"); // wait for the service available
	//ros::ServiceClient add_turtle = node.serviceClient<turtlesim::Spawn>("spawn"); // create a service_client object
	//turtlesim::Spawn srv; // service type
	//add_turtle.call(srv); // call the service

	ros::Publisher turtle_vel = node.advertise<geometry_msgs::Twist>("sim_p3at/cmd_vel", 10000);

	tf::TransformListener listener;

	ros::Rate rate(10.0);
	while (node.ok()) {
		tf::StampedTransform transform;
		try {
//			// make the second turtle go to where the first turtle was 5 seconds ago
//			ros::Time now = ros::Time::now();
//			ros::Time past = now - ros::Duration(5.0);
//			listener.waitForTransform("/turtle2", now, "/turtle1", past, "/world", ros::Duration(1.0));
//			listener.lookupTransform("/turtle2", now, "/turtle1", past, "/world", transform);

			ros::Time now = ros::Time::now();
			// wait until a transform becomes available
			listener.waitForTransform("/base_footprint", "/pioneer3at_link", now, ros::Duration(10.0));
			// the second argument "/turtle1" could be replaced by "/carrot1"
			listener.lookupTransform("/base_footprint", "/pioneer3at_link", now, transform);
//			listener.lookupTransform("/turtle2", "/turtle1", ros::Time::now(), transform);

		} catch (tf::TransformException ex) {
			ROS_ERROR("%s", ex.what());
		}

		geometry_msgs::Twist vel_msg;
		vel_msg.angular.z = 4 * atan2(transform.getOrigin().y(), transform.getOrigin().x());
		vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));

		turtle_vel.publish(vel_msg);

		rate.sleep();
	}
	return 0;
}
;
