#pragma once

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

namespace map_getter
{
    class mapGetter
    {
    private:
        ros::NodeHandle nh_;
        ros::Subscriber map_sub_;

        nav_msgs::OccupancyGrid map_;
        int flag_;

    public:
        mapGetter(ros::NodeHandle &nodeHandle, std::string &mapTopic);
        nav_msgs::OccupancyGrid giveMeMyMap();
        void mapCallback(const nav_msgs::OccupancyGrid &mapMessage);
        ~mapGetter();
    };
}