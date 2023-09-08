#include <ros/ros.h>
#include "turtlebot3_laser_deg/returnMap.hpp"

namespace map_getter
{
    mapGetter::mapGetter(ros::NodeHandle &nodeHandle, std::vector<std::string> &topic_names) : nh_(nodeHandle), mapFlag_(0), odomFlag_(0), scanFlag_(0)
    {
        ROS_INFO_STREAM("Retrieve map as a variable.");
        
        scan_sub_ = nh_.subscribe(topic_names.at(0), 10, &mapGetter::scanCallback, this);
        map_sub_ = nh_.subscribe(topic_names.at(1), 10, &mapGetter::mapCallback, this);
        odom_sub_ = nh_.subscribe(topic_names.at(2), 10, &mapGetter::odomCallback, this);
        ros::Rate rate(10);

        while (ros::ok())
        {
            if((scanFlag_ == 1) && (mapFlag_ == 1) && (odomFlag_ == 1))
            {
                break;
            }
            ROS_INFO_STREAM("Waiting for items to be collected.");
            ROS_INFO_STREAM("Status: " << scanFlag_ << ", " << mapFlag_ << ", " << odomFlag_);
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
        mapFlag_ = 1;
    }

    void mapGetter::scanCallback(const sensor_msgs::LaserScan &scanMsg)
    {
        scan_ = scanMsg;
        scanFlag_ = 1;
    }

    void mapGetter::odomCallback(const nav_msgs::Odometry &odomMsg)
    {
        odom_ = odomMsg;
        odomFlag_ = 1;
    }

    nav_msgs::OccupancyGrid mapGetter::giveMeMyMap()
    {
        return map_;
    }

    nav_msgs::Odometry mapGetter::giveMeOdom()
    {
        return odom_;
    }

    sensor_msgs::LaserScan mapGetter::giveMeScan()
    {
        return scan_;
    }

}