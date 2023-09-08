#pragma once

#include <random>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>

namespace ttb_highlevel_controller
{

	/*!
	 * Class containing the Turtlebot Highlevel Laser Manipulator
	 */
	class TtbLaserManipulator
	{
	public:
		/*!
		 * Constructor.
		 */
		TtbLaserManipulator(ros::NodeHandle &nodeHandle);

		/*!
		 * Destructor.
		 */
		virtual ~TtbLaserManipulator();

	private:
		// parameter server
		bool readParameters();

		// functions to read scan and publish new scan
		void manipulateScans(const sensor_msgs::LaserScan &scanMessage);
		void manipulateImages(const sensor_msgs::LaserScan &scanMessage);
		void publishModScan(const sensor_msgs::LaserScan &rndScan);

		// config related to backend
		ros::NodeHandle nodeHandle_;
		ros::Subscriber ttbLaserScanSubscriber_;
		ros::Publisher obstrScanPublisher_;
		ros::Publisher rndScanPublisher_;
		// sensor_msgs/Image

		// variables related to parameter server
		float mask_range_;
		int mask_start_idx_;
		int window_size_;
		float noise_;

		std::vector<std::string> topic_names_;

		nav_msgs::OccupancyGrid map_final_;
		sensor_msgs::LaserScanPtr simulatedScan_;

		nav_msgs::Odometry robPose_;
		sensor_msgs::LaserScan scanSingle_;
		ros::Publisher simScanPub_;
		ros::Publisher reScanPub_;
	};

} /* namespace */