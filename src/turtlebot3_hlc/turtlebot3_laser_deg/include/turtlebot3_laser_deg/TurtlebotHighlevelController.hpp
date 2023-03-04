#pragma once

#include <random>
#include <algorithm>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

namespace ttb_highlevel_controller
{

	/*!
	 * Class containing the Husky Highlevel Controller
	 */
	class TtbHighlevelController
	{
	public:
		/*!
		 * Constructor.
		 */
		TtbHighlevelController(ros::NodeHandle &nodeHandle);

		/*!
		 * Destructor.
		 */
		virtual ~TtbHighlevelController();

	private:
		// parameter server
		bool readParameters();

		// functions to read scan and publish new scan
		void manipulateScans(const sensor_msgs::LaserScan &scanMessage);
		void publishModScan(const sensor_msgs::LaserScan &maskedScan,  const sensor_msgs::LaserScan &rndScan);

		// config related to backend
		ros::NodeHandle nodeHandle_;
		ros::Subscriber ttbLaserScanSubscriber_;
		ros::Publisher obstrScanPublisher_;
		ros::Publisher rndScanPublisher_;

		// variables related to parameter server
		float mask_range_;
		int mask_start_idx_;
		int window_size_;
	};

} /* namespace */