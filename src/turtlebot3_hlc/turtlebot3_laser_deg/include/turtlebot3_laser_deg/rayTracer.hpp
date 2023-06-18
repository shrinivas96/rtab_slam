#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

namespace ray_tracing
{
	class mapGetter
    {
    private:
        ros::NodeHandle nh_;
        ros::Subscriber map_sub_;

        nav_msgs::OccupancyGrid map_;
        int flag_;
    public:
        mapGetter(ros::NodeHandle &nodeHandle);
        nav_msgs::OccupancyGrid giveMeMyMap();
        void mapCallback(const nav_msgs::OccupancyGrid &mapMessage);
        ~mapGetter();
    };

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
		
        void callbackMap(const nav_msgs::OccupancyGrid &mapMessage);

		// config related to backend
		ros::NodeHandle nodeHandle_;
		ros::Subscriber mapSubscriber_;

		// to get the topic of map to be subscribed, and scan and odom for a single message
		std::vector<std::string> topic_names_;

		// getting the pose where the scan is taken, and some scan deets
		nav_msgs::OdometryConstPtr robPose_;
		sensor_msgs::LaserScanConstPtr scanSingle_;

		// getting the mnap from the map server
		nav_msgs::OccupancyGrid map_final_;
	};
} /* namespace */