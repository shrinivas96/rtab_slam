#define _USE_MATH_DEFINES

#include "turtlebot3_laser_deg/simulateScan.hpp"
#include <tf2/LinearMath/Quaternion.h>

namespace sim_scan
{
    simulateScan::simulateScan(nav_msgs::OccupancyGrid &map, 
                               geometry_msgs::Pose &pose, 
                               sensor_msgs::LaserScan &scan_info) : map_(map), robPoseWorldFrame_(pose), scan_(scan_info)
    {
        ROS_INFO_STREAM("Inside the sim scan constructor. Passing on to the coordinate sys.");
        createSimScan();
    }
    
    simulateScan::~simulateScan()
    {
    }

    PoseGridFrame simulateScan::world_to_map_coordinates(geometry_msgs::Pose &poseWrldFrame, nav_msgs::OccupancyGrid &map)
    {
        PoseGridFrame resultingPoseMapFrame;

        // getting some config values related to map and pose
        float gridSize = map.info.resolution;
        geometry_msgs::Pose mapOrgin = map.info.origin;
        geometry_msgs::Point mapOrigins = map.info.origin.position;

        float robPoseWFX = poseWrldFrame.position.x;
        float robPoseWFY = poseWrldFrame.position.y;
        tf2::Quaternion robPoseQuat(poseWrldFrame.orientation.x, poseWrldFrame.orientation.y, poseWrldFrame.orientation.z, poseWrldFrame.orientation.w);
        float robPoseYaw = robPoseQuat.getAngle();      // getAngle() returns the yaw in our case. do not know how though

        // get the world coordinate distance to origin
        float dist2OriginX = robPoseWFX - mapOrgin.position.x;
        float dist2OriginY = robPoseWFY - mapOrgin.position.y;

        // TODO: this should ideally be a retun value, but for now I am just experimenting
        poseMapFrame_.x = std::floor(dist2OriginX/gridSize);
        poseMapFrame_.y = std::floor(dist2OriginY/gridSize);
        poseMapFrame_.yaw = robPoseYaw;

        poseMapCoordinates_.position.x = std::floor(dist2OriginX/gridSize);
        poseMapCoordinates_.position.y = std::floor(dist2OriginY/gridSize);
        poseMapCoordinates_.position.z = 0;
        poseMapCoordinates_.orientation = poseWrldFrame.orientation;

        ROS_INFO_STREAM("Struct Poses are: " << poseMapFrame_.x << " " << poseMapFrame_.y);
        ROS_INFO_STREAM("nav Poses are: " << poseMapCoordinates_.position.x << " " << poseMapCoordinates_.position.y);

        ROS_INFO_STREAM("The pose angle is " << poseMapFrame_.yaw << " " << robPoseYaw);
    }

    sensor_msgs::LaserScan simulateScan::createSimScan()
    {
        sensor_msgs::LaserScan resultScan;
        
        world_to_map_coordinates(robPoseWorldFrame_, map_);
        float convert = 180/M_PI;
        // ROS_INFO_STREAM("The min and max angle are " << scan_.angle_min << " " << scan_.angle_max << " " << scan_.angle_increment);
        // ROS_INFO_STREAM("The min and max angle are " << scan_.angle_min*convert << " " << scan_.angle_max*convert << " " << scan_.angle_increment*convert);
        // float numRays = (scan_.angle_max - scan_.angle_min) / scan_.angle_increment;
        // ROS_INFO_STREAM("Total rays in this one: " << numRays);
        // ROS_INFO_STREAM("Total ranges in the array: " << scan_.ranges.size());
        // ROS_INFO_STREAM("Angle values: ");
        // std::cout << "\n\n";

        // config for scan iteration
        float start_angle = poseMapFrame_.yaw + scan_.angle_min;
        float incr_angle = scan_.angle_increment;
        float end_angle = start_angle + scan_.angle_max;
        int count = 0;

        // this is the potential location where an obstacle might be
        // this pose value will be converted to map frame to check if it is occupied or not
        geometry_msgs::Pose maxRangeWrldFrame;

        // store the x, y distance of max range based on range and angle
        std::array<float, 2> maxRangeCartesian = {0, 0};

        for (float angle=start_angle; angle<=end_angle; angle+=incr_angle)
        {
            // you need to translate this much distance from your pose
            maxRangeCartesian = polar2cartesian(scan_.range_max, angle);

            // the world frame location of a potential obstacle
            // this is robPose+pol2cart(maxrange, angle)
            maxRangeWrldFrame = robPoseWorldFrame_;
            maxRangeWrldFrame.position.x += maxRangeCartesian[0];
            maxRangeWrldFrame.position.y += maxRangeCartesian[1];
        }
        
        // this is the final step where you will simulate the scan for each ray
        for(float angle=start_angle; angle<=end_angle; angle+=incr_angle)
        {
            std::cout << count << " " << angle << " " << angle*convert << "\n";
            count += 1;
        }


        // float angle=scan_.angle_min;
        // for(auto &rangeVal:scan_.ranges)
        // {
        //     std::cout << count << " " << angle << " " << angle*convert << " " << rangeVal << "\n";
        //     angle+=scan_.angle_increment;
        //     count++;
        // }

        
        ROS_INFO_STREAM("Start angle in pose: " << poseMapFrame_.yaw);
        // float start_angle = poseMapFrame_.yaw + scan_.angle_min;
        // float end_angle = start_angle + scan_.angle_max;
        // float total_angle = end_angle - start_angle;
        // float numraysMine = (end_angle - start_angle) / scan_.angle_increment;


        return resultScan;
    }
}