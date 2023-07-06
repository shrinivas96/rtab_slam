#include "occupancy_grid_utils/ray_tracer.h"
#include "turtlebot3_laser_deg/rayTracer.hpp"
#include "turtlebot3_laser_deg/readFromFile.hpp"
#include "turtlebot3_laser_deg/returnMap.hpp"
#include "turtlebot3_laser_deg/simulateScan.hpp"

namespace ray_tracing
{
	rayTracer::rayTracer(ros::NodeHandle &nodeHandle) : nodeHandle_(nodeHandle)
	{
		// read parameters and load them into variables
		if(!readParameters())
		{
			ROS_ERROR_STREAM("Topic name paramteres could not be loaded. Shutting down.");
			ros::requestShutdown();
		}

		// can safely say now that the node is initiated
		ROS_INFO_STREAM("Ray tracing node.");

		// advertise topic to published
		simScanPub_ = nodeHandle_.advertise<sensor_msgs::LaserScan>(topic_names_.at(3), 10, true);
		selfSimScanPub_ = nodeHandle_.advertise<sensor_msgs::LaserScan>(topic_names_.at(4), 10, true);

		// get the map from the map server
		map_getter::mapGetter map_from_server(nodeHandle_, topic_names_.at(0));
		map_final_ = map_from_server.giveMeMyMap();
		ROS_INFO_STREAM("RT Map retrieved of size: " << map_final_.info.height << " x " << map_final_.info.width);

		// get the topics published in odom and scan
		odomVec_ = readOdomFromFile(odomFileLoc_);
		scanVec_ = readScanFromFile(scanFileLoc_);

		// and get the last pose and scan
		robPose_ = odomVec_.back();
		scanSingle_ = scanVec_.back();

		geometry_msgs::Pose currPose = robPose_.pose.pose;

		// finally simulate scan based on the function
		simulatedScan_ = occupancy_grid_utils::simulateRangeScan(map_final_, robPose_.pose.pose, scanSingle_);

		simScanPub_.publish(*simulatedScan_);

		ROS_INFO_STREAM("RT start my sim scan.");
		sim_scan::simulateScan mySimVar(map_final_, robPose_.pose.pose, scanSingle_);
		anotherSimulatedScan_ = mySimVar.returnSimulatedScan();

		selfSimScanPub_.publish(anotherSimulatedScan_);
	}

	/*!
	 * Destructor.
	 */
	rayTracer::~rayTracer()
	{
	}

    bool rayTracer::readParameters()
    {
		// contains map, scan and odom topics
		bool topicsParam = nodeHandle_.getParam("topic_names", topic_names_);
		bool scanFileParam = nodeHandle_.getParam("scan_file", scanFileLoc_);
		bool odomFileParam = nodeHandle_.getParam("odom_file", odomFileLoc_);

        if (topicsParam && scanFileParam && odomFileParam)
            ROS_INFO_STREAM("All parameters for ray tracing were found.");
        else
			ROS_INFO_STREAM("Parameters for ray tracing not found.");
        
        return topicsParam && scanFileParam && odomFileParam;
    }

	void rayTracer::callbackMap(const nav_msgs::OccupancyGrid &mapMessage)
	{
		ROS_INFO_STREAM("Callback is not being developed atm.");
		// gen config
		// ROS_INFO_STREAM("Callback function for ray tracer");
		// ROS_INFO_STREAM("Map info origin: " << mapMessage.info.origin.position.x << ", " << mapMessage.info.origin.position.y);

		// robPose_ = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", nodeHandle_);
		// scanSingle_ = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", nodeHandle_);
		// if (!robPose_)
		// {
		// 	ROS_ERROR("Robot pose message seems to be empty");
		// }
		// if (!scanSingle_)
		// {
		// 	ROS_ERROR("Scan message seems to be empty");
		// }

		// ROS_INFO_STREAM("Pose: " << robPose_->pose.pose.position.x << ", " << robPose_->pose.pose.position.y);
		// ROS_INFO_STREAM("Scan details: " << scanSingle_->angle_max);
	}
} /* namespace */
