#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

namespace ray_tracing
{
	class rayTracer
	{
	public:
		/*!
		 * Constructor.
		 */
		rayTracer(ros::NodeHandle &nodeHandle);

		/*!
		 * Destructor.
		 */
		virtual ~rayTracer();

	private:
        // parameter server
		bool readParameters();
		
		// callback function: unused atm
        void callbackMap(const nav_msgs::OccupancyGrid &mapMessage);

		// config related to backend ros handling
		ros::NodeHandle nodeHandle_;
		ros::Subscriber mapSubscriber_;
		ros::Publisher simScanPub_;
		ros::Publisher selfSimScanPub_;

		// to get the topic of map to be subscribed, and scan and odom log file locations
		std::vector<std::string> topic_names_;
		std::string scanFileLoc_, odomFileLoc_;

		// getting the whole vector of scans and odoms form file
		std::vector<nav_msgs::Odometry> odomVec_;
		std::vector<sensor_msgs::LaserScan> scanVec_;

		// getting the last pose where the scan is taken, and some scan details for the last pose
		nav_msgs::Odometry robPose_;
		sensor_msgs::LaserScan scanSingle_;

		// saving the simulated scan
		sensor_msgs::LaserScanPtr simulatedScan_;
		sensor_msgs::LaserScan anotherSimulatedScan_;

		// getting the mnap from the map server
		nav_msgs::OccupancyGrid map_final_;
	};
} /* namespace */