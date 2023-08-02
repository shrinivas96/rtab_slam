#pragma once

#include <random>
#include <ros/ros.h>
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
		TtbLaserManipulator(ros::NodeHandle &nodeHandle, std::string sensor);

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

		std::vector<std::string> topic_names_;
	};

} /* namespace */