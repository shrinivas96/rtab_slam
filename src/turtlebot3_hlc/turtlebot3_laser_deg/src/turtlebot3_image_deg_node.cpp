#include <ros/ros.h>
#include "turtlebot3_laser_deg/TurtlebotHighlevelController.hpp"

int main(int argc, char **argv)
{
	// ROS always initializes here.
	// This is the main node that starts all the process
	ros::init(argc, argv, "laser_manipulator");

	// private node handle to be passed to the package class
	ros::NodeHandle nodeHandle("~");

	// instantiate object of main class and pass the node handle
	// for my reference: namespace::className
	ttb_highlevel_controller::TtbLaserManipulator turtleObj(nodeHandle, "camera");

	// wait for incoming messages. returns only when node is shutdown.
	ros::spin();

	return 0;
}
