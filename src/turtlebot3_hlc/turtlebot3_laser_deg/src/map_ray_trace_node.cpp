#include <ros/ros.h>
#include "occupancy_grid_utils/ray_tracer.h"
#include "turtlebot3_laser_deg/rayTracer.hpp"
// #include <turtlebot3_laser_deg/rayTracer.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ray_tracing");

    ros::NodeHandle nodeHandle("~");

    ray_tracing::rayTracer rayNode(nodeHandle);

    ros::spin();

    return 0;
}