#pragma once

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

namespace map_getter
{
    class mapGetter
    {
    private:
        ros::NodeHandle nh_;
        ros::Subscriber map_sub_;
        ros::Subscriber odom_sub_;
        ros::Subscriber scan_sub_;

        nav_msgs::OccupancyGrid map_;
        nav_msgs::Odometry odom_;
        sensor_msgs::LaserScan scan_;
        int mapFlag_, odomFlag_, scanFlag_;

    public:
        mapGetter(ros::NodeHandle &nodeHandle, std::vector<std::string> &topic_names);
        nav_msgs::OccupancyGrid giveMeMyMap();
        nav_msgs::Odometry giveMeOdom();
        sensor_msgs::LaserScan giveMeScan();
        void mapCallback(const nav_msgs::OccupancyGrid &mapMessage);
        void odomCallback(const nav_msgs::Odometry &odomMsg);
        void scanCallback(const sensor_msgs::LaserScan &scanMsg);
        ~mapGetter();
    };
}