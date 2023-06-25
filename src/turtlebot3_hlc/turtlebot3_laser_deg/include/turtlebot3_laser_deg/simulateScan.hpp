#pragma once

#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

namespace sim_scan
{
    struct PoseDiscrete
    {
        std::uint16_t x;
        std::uint16_t y;
        double yaw;
    };

    class simulateScan
    {
    private:
        const nav_msgs::OccupancyGrid map_;
        const geometry_msgs::Pose robPoseWorldFrame_;
        PoseDiscrete poseMapFrame_;
        geometry_msgs::Pose poseMapCoordinates_;

        const sensor_msgs::LaserScan scan_;
        sensor_msgs::LaserScan simulatedScan_;

    public:
        simulateScan(nav_msgs::OccupancyGrid &map, geometry_msgs::Pose &pose, sensor_msgs::LaserScan &scan_info);
        ~simulateScan();

        PoseDiscrete world_to_map_coordinates(const geometry_msgs::Pose &pose, const nav_msgs::OccupancyGrid &map);

        // TODO: this fnction could be used to check if the
        // robot has spawned on top of an obstacle location
        std::int8_t checkIfObstacle(PoseDiscrete &cellLocation);

        geometry_msgs::Pose map_to_world_coordinates(const PoseDiscrete &poseMapFrame, const nav_msgs::OccupancyGrid &map);

        // function to calculate distance based on two poses and a map
        float poses_to_range(const geometry_msgs::Pose &laserEndPntWrldFrame, const geometry_msgs::Pose &poseWrldFrame);
        
        sensor_msgs::LaserScan createSimScan();
    };
}

std::array<float, 2> polar2cartesian(float rho, float phi)
{
    std::array<float, 2> pointCartesian;
    pointCartesian[0] = rho * std::cos(phi);
    pointCartesian[1] = rho * std::sin(phi);
    return pointCartesian;
}

void bresenham(sim_scan::PoseDiscrete p1, sim_scan::PoseDiscrete p2, std::vector<int> &X, std::vector<int> &Y)
{
    int dx, dy;
    int x1 = p1.x;
    int x2 = p2.x;
    int y1 = p1.y;
    int y2 = p2.y;

    dx = std::abs(x2 - x1);
    dy = std::abs(y2 - y1);

    bool steep;

    if (dy > dx)
    {
        steep = true;
        std::swap(x1, y1);
        std::swap(x2, y2);
    }

    if (x1 > x2)
    {
        std::swap(x1, x2);
        std::swap(y1, y2);
    }

    dx = std::abs(x2 - x1);
    dy = std::abs(y2 - y1);
    int error = 0, ystep;

    int x_n = x1, y_n = y1;
    if (y1 < y2)
        ystep = 1;
    else
        ystep = -1;
    for (int i = 0; i <= dx; i++)
    {
        if (steep)
        {
            X.emplace_back(x_n);
            Y.emplace_back(y_n);
        }
        else
        {
            X.emplace_back(y_n);
            Y.emplace_back(x_n);
        }
        x_n++;
        error = error + dy;

        if (2 * error >= dx)
        {
            y_n = y_n + ystep;
            error = error - dx;
        }
    }
    // original code had a swap of here. gave us the wrong results. included here for information
    // std::swap(X, Y);
}

float distBwPoses(sim_scan::PoseDiscrete &p1, sim_scan::PoseDiscrete &p2)
{
    int dx = std::abs(p2.x - p1.x);
    int dy = std::abs(p2.y - p1.y);
    return std::sqrt((dx*dx) + (dy*dy));
}
