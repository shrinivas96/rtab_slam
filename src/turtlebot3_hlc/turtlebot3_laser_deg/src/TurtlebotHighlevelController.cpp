#include <turtlebot3_laser_deg/TurtlebotHighlevelController.hpp>

namespace ttb_highlevel_controller
{
	/*!
	 * Constructor.
	 */
	TtbLaserManipulator::TtbLaserManipulator(ros::NodeHandle &nodeHandle) : nodeHandle_(nodeHandle)
	{
		// reads parameters and loads values into variables
		if (!readParameters())
		{
			// happens if the parameter file is wrong of the parameter names were not found inside
			ROS_ERROR("Parameters needed by laser manipulator could not be loaded. Shutting down.");
			ros::requestShutdown();
		}

		// should make this kind of a message more cleaner in the future? 
		for(std::string Tnames:topic_names_)
		{
			ROS_INFO_STREAM("Topics available to pub/sub: " << Tnames);
		}
		
		// subscriber to scan topic and publishers of altered scans: obstruction and randomizing
		ttbLaserScanSubscriber_ = nodeHandle_.subscribe(topic_names_.at(0), 10, &TtbLaserManipulator::manipulateScans, this);
		obstrScanPublisher_ = nodeHandle_.advertise<sensor_msgs::LaserScan>(topic_names_.at(1), 10);
		rndScanPublisher_ = nodeHandle_.advertise<sensor_msgs::LaserScan>(topic_names_.at(2), 10);
	}

	/*!
	 * Destructor.
	 */
	TtbLaserManipulator::~TtbLaserManipulator()
	{
	}

	/*!
	 * Read the paramters from a config file (parameter server), 
	 * and return true if all the parameters have been successfully read.
	 */
	bool TtbLaserManipulator::readParameters()
	{
		// new mask range, start index and window size parameters: ${pkg}/config/default.yaml
		bool newRangeParam = nodeHandle_.getParam("new_range", mask_range_);
		bool startIdxParam = nodeHandle_.getParam("start_indx", mask_start_idx_);
		bool windowParam = nodeHandle_.getParam("window", window_size_);

		// combine all mask related params
		bool maskParams = newRangeParam && startIdxParam && windowParam;

		// subscriber and publisher topic names
		bool topicsParam = nodeHandle_.getParam("topic_names", topic_names_);
		
		// if any of them were false
		bool combinedParams = maskParams && topicsParam;

		if(combinedParams)
		{
			ROS_INFO_STREAM("All parameters for laser manipulator were found.");
		}
		else
		{
			ROS_INFO_STREAM("Parameters for laser manipulator not found.");
		}
		return combinedParams;
	}
	
	/*
	 * callback function to manipulate scans: apply a mask to the scans, and randomize each of their values.
	 */
	void TtbLaserManipulator::manipulateScans(const sensor_msgs::LaserScan &scanMessage)
	{
		// gen config
		int num_ranges = scanMessage.ranges.size();

		// mask config:
		// check for window size; shoould not exceed range bounds
		if(window_size_ >= num_ranges-mask_start_idx_)
			window_size_ = num_ranges-mask_start_idx_;

		/* config for generating random numbers
		uniform random number generator between max and min range; source:
		https://en.cppreference.com/w/cpp/numeric/random/uniform_real_distribution
		*/
		float min = -1e-1; 				// can specify a min and max bound; max=1e-2;
		std::random_device rd;  		// will be used to obtain a seed for the random number engine
		std::mt19937 rd_gen(rd()); 		// Standard mersenne_twister_engine seeded with rd()
		std::uniform_real_distribution<float> dist(min, -min);


		// applying two transforms: 
		// first one applies a fixed mask to a selected window in the scan.
		// second applies a random value chosen from a unif. dist. TO the masked scan.
		
		// TODO: masking and randomizing needs to be inside a try catch block
		// old scan values except the values where mask is placed
		sensor_msgs::LaserScan maskedScan = scanMessage;

		// applies the new range to a slice specified by begin + [a, b]
		std::transform(maskedScan.ranges.begin() + mask_start_idx_,
						maskedScan.ranges.begin() + mask_start_idx_ + window_size_,
						maskedScan.ranges.begin() + mask_start_idx_, 				// apply in place instead of to a diff vector
						[this](float& i) {											// this keyword is needed when using class member objs
							return mask_range_;
						});

		// prev. masked scan values plus samples from a uniform distribution
		sensor_msgs::LaserScan randomizedScan = maskedScan;
		std::transform(randomizedScan.ranges.begin(), randomizedScan.ranges.end(),
						randomizedScan.ranges.begin(),								// apply in place instead of to a diff vector
						[&](float &j) {												// & is used to encl. variables from outside lambda scope by ref
							return j + dist(rd_gen);
						});

		// TODO: masking and randomizing needs to be inside a try catch block
		// for (int i=0; i < num_ranges; i++)
		// {
		// 	randomizedScan.ranges[i] += dist(rd_gen);
		// }

		// publish both scans
		publishModScan(maskedScan, randomizedScan);
	}

	/*
	 * publisher function for new scans
	 */
	void TtbLaserManipulator::publishModScan(const sensor_msgs::LaserScan &maskedScan, const sensor_msgs::LaserScan &rndScan)
	{
		ros::Rate loopRate_(10);
		// while(ros::ok())
		// {
		// 	ros::spinOnce;
		// 	loopRate.sleep();
		// }
		obstrScanPublisher_.publish(maskedScan);
		rndScanPublisher_.publish(rndScan);
		loopRate_.sleep();
	}
} /* namespace */
