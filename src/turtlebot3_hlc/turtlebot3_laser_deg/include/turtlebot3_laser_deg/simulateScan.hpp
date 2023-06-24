#pragma once

// #include <cmath>
#include <limits>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

#define INF std::numeric_limits<float>::infinity();

std::array<float, 2> polar2cartesian(float rho, float phi)
{
    std::array<float, 2> pointCartesian;
    pointCartesian[0] = rho * std::cos(phi);
    pointCartesian[1] = rho * std::sin(phi);
    return pointCartesian;
}

namespace sim_scan
{
    struct PoseGridFrame
    {
        std::uint16_t x;
        std::uint16_t y;
        double yaw;
    };

    class simulateScan
    {
    private:
        nav_msgs::OccupancyGrid map_;
        geometry_msgs::Pose robPoseWorldFrame_;
        PoseGridFrame poseMapFrame_;
        geometry_msgs::Pose poseMapCoordinates_;
        
        sensor_msgs::LaserScan scan_;
        sensor_msgs::LaserScan simulatedScan_;
    public:
        simulateScan(nav_msgs::OccupancyGrid &map, geometry_msgs::Pose &pose, sensor_msgs::LaserScan &scan_info);
        ~simulateScan();

        sensor_msgs::LaserScan createSimScan();

        PoseGridFrame world_to_map_coordinates(geometry_msgs::Pose &pose, nav_msgs::OccupancyGrid &map);
    };

    float returnYaw(geometry_msgs::Quaternion q);
}