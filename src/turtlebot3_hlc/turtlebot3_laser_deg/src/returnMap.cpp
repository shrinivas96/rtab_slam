#include <ros/ros.h>
#include "turtlebot3_laser_deg/returnMap.hpp"

namespace map_getter
{
    mapGetter::mapGetter(ros::NodeHandle &nodeHandle, std::string &mapTopic) : nh_(nodeHandle), flag_(0)
    {
        ROS_INFO_STREAM("Retrieve map as a variable.");
        
        map_sub_ = nh_.subscribe(mapTopic, 10, &mapGetter::mapCallback, this);
        ros::Rate rate(10);
        
        while (flag_ == 0 && ros::ok())
        {
            ROS_INFO_STREAM("Waiting for map to be published.");
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO_STREAM("Map retrieved " << map_.info.height << " by " << map_.info.width);
    }
    
    mapGetter::~mapGetter()
    {
    }

    void mapGetter::mapCallback(const nav_msgs::OccupancyGrid &mapMessage)
    {
        map_ = mapMessage;
        flag_ = 1;
    }

    nav_msgs::OccupancyGrid mapGetter::giveMeMyMap()
    {
        return map_;
    }
}