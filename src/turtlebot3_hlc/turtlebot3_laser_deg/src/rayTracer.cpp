#include <ros/topic.h>
#include "occupancy_grid_utils/ray_tracer.h"
#include "turtlebot3_laser_deg/rayTracer.hpp"

namespace ray_tracing
{
	mapGetter::mapGetter(ros::NodeHandle &nodeHandle) : nh_(nodeHandle), flag_(0)
    {
        ROS_INFO_STREAM("Node to retrieve the map as a variable");
		std::vector<std::string> topic_names;
		bool topicsParam = nh_.getParam("topic_names", topic_names);
		if (!topicsParam)
		{
			ROS_ERROR_STREAM("Topics could not be taken in.");
			ros::requestShutdown();
		}

		ROS_INFO_STREAM("Topics were found. Will try to sub to: " << topic_names.at(0));
		
        map_sub_ = nh_.subscribe(topic_names.at(0), 10, &mapGetter::mapCallback, this);
        while (flag_ == 0 && ros::ok())
        {
            ROS_INFO_STREAM("Waiting for map to be presented.");
            ros::Duration(0.5).sleep();
        }
        ROS_INFO_STREAM("Map retrieved " << map_.info.height << "by " << map_.info.width);
    }
    
    mapGetter::~mapGetter()
    {
    }

    void mapGetter::mapCallback(const nav_msgs::OccupancyGrid &mapMessage)
    {
        ROS_INFO_STREAM("Callback function once!");
        map_ = mapMessage;
        flag_ = 1;
    }

    nav_msgs::OccupancyGrid mapGetter::giveMeMyMap()
    {
        return map_;
    }


	rayTracer::rayTracer(ros::NodeHandle &nodeHandle) : nodeHandle_(nodeHandle)
	{
		ROS_INFO_STREAM("Ray tracing node: ");

		mapGetter map_from_server(nodeHandle_);
		map_final_ = map_from_server.giveMeMyMap();

		ROS_WARN_STREAM("To prove that I got the ring");
		ROS_WARN_STREAM("Map is of size: " << map_final_.info.height << " x " << map_final_.info.width);
		// subscriber to scan topic and publishers of altered scans: obstruction and randomizing
		// mapSubscriber_ = nodeHandle_.subscribe("/map", 10, &rayTracer::callbackMap, this);
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

        if (topicsParam)
            ROS_INFO_STREAM("All parameters for ray tracing were found.");
        else
			ROS_INFO_STREAM("Parameters for ray tracing not found.");
        
        return topicsParam;
    }

	void rayTracer::callbackMap(const nav_msgs::OccupancyGrid &mapMessage)
	{
		// gen config
		ROS_INFO_STREAM("Callback function for ray tracer");
		ROS_INFO_STREAM("Map info origin: " << mapMessage.info.origin.position.x << ", " << mapMessage.info.origin.position.y);

		robPose_ = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", nodeHandle_);
		scanSingle_ = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", nodeHandle_);
		if (!robPose_)
		{
			ROS_ERROR("Robot pose message seems to be empty");
		}
		if (!scanSingle_)
		{
			ROS_ERROR("Scan message seems to be empty");
		}

		ROS_INFO_STREAM("Pose: " << robPose_->pose.pose.position.x << ", " << robPose_->pose.pose.position.y);
		ROS_INFO_STREAM("Scan details: " << scanSingle_->angle_max);
	}
} /* namespace */
