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
        simulatedScan_ = createSimScan();
    }
    
    simulateScan::~simulateScan()
    {
    }

    PoseDiscrete simulateScan::world_to_map_coordinates(const geometry_msgs::Pose &poseWrldFrame, const nav_msgs::OccupancyGrid &map)
    {
        // container for the result to go into
        PoseDiscrete resultingPoseMapFrame;

        // getting some config values related to map
        float gridSize = map.info.resolution;
        geometry_msgs::Pose mapOrgin = map.info.origin;

        // the (x, y, theta) values of the pose
        float robPoseWFX = poseWrldFrame.position.x;
        float robPoseWFY = poseWrldFrame.position.y;
        tf2::Quaternion robPoseQuat(poseWrldFrame.orientation.x, poseWrldFrame.orientation.y, poseWrldFrame.orientation.z, poseWrldFrame.orientation.w);
        float robPoseYaw = robPoseQuat.getAngle();      // getAngle() returns the yaw in our case. do not know how though

        // get the world coordinate distance to origin
        // TODO: this SHould ideally always be positive, but add a check
        float dist2OriginX = robPoseWFX - mapOrgin.position.x;
        float dist2OriginY = robPoseWFY - mapOrgin.position.y;

        resultingPoseMapFrame.x = std::floor(dist2OriginX/gridSize);
        resultingPoseMapFrame.y = std::floor(dist2OriginY/gridSize);
        resultingPoseMapFrame.yaw = robPoseYaw;

        // this is only here for some experimentation, should be removed later
        poseMapCoordinates_.position.x = std::floor(dist2OriginX/gridSize);
        poseMapCoordinates_.position.y = std::floor(dist2OriginY/gridSize);
        poseMapCoordinates_.position.z = 0;
        poseMapCoordinates_.orientation = poseWrldFrame.orientation;

        return resultingPoseMapFrame;
    }

    geometry_msgs::Pose simulateScan::map_to_world_coordinates(const PoseDiscrete &poseMapFrame, const nav_msgs::OccupancyGrid &map)
    {
        // scale the x and y back to world coordinate by multiplying the grid cell size
        geometry_msgs::Pose poseWrldFrame;
        poseWrldFrame.position.x = poseMapFrame.x * map.info.resolution;
        poseWrldFrame.position.y = poseMapFrame.y * map.info.resolution;
        
        tf2::Quaternion poseQuatFromEuler;
        poseQuatFromEuler.setRPY(0, 0, poseMapFrame.yaw);
        poseWrldFrame.orientation.w = poseQuatFromEuler.getW();
        poseWrldFrame.orientation.x = poseQuatFromEuler.getX();
        poseWrldFrame.orientation.y = poseQuatFromEuler.getY();
        poseWrldFrame.orientation.z = poseQuatFromEuler.getZ();
        
        return poseWrldFrame;
    }

    float simulateScan::poses_to_range(const geometry_msgs::Pose &robPoseWrldFrame, const geometry_msgs::Pose &laserEndPntWrldFrame)
    {
        // based on the laser endpoint and robot pose in world coordinates,
        // get the absolute distances in x and y
        float dx = std::abs(robPoseWrldFrame.position.x - laserEndPntWrldFrame.position.x);
        float dy = std::abs(robPoseWrldFrame.position.y - laserEndPntWrldFrame.position.y);
        
        // return hypoteneous distance
        return std::sqrt((dx*dx) + (dy*dy));
    }

    std::int8_t simulateScan::checkIfObstacle(PoseDiscrete & cellLocation)
    {
        // check based on map and cell location if the location is occupied or not.
        // the three possible values you can have are 0, 100, -1
        int mx = cellLocation.x, my = cellLocation.y;
        
        // we need to find the index of an array that corresponds to the x, y location on the grid.
        // the x, y location of the cell has its counting starting from the bottom left of the grid.
        // we think in row, col from top left
        // the origin cell is (0, 0)
        std::size_t index = my * map_.info.width + mx;

        // return decision based on one of 3 values: 0, 1, 2
        std::int8_t result;

        try
        {
            int cellValueAtIndx = map_.data.at(index);
            if(cellValueAtIndx == 0)
                result = 0;
            else if (cellValueAtIndx == 100)
                result = 1;
            else if (cellValueAtIndx == -1)
                result = 2;
            else
                ROS_ERROR("Map contains value %d that is not permissible. Thre possible cell values are 0, 100, -1", cellValueAtIndx);
        }
        catch(const std::exception& e)
        {
            ROS_ERROR("Could not access the value of grid cell at the index. For a detailed error: \n");
            ROS_ERROR_STREAM(e.what());
        }

        return result;
    }

    sensor_msgs::LaserScan simulateScan::createSimScan()
    {
        // TODO: check that the resulting scan has no ranges
        sensor_msgs::LaserScan resultScan;                      // get all the meta data from the original scan, but clear ranges
        resultScan = scan_;
        resultScan.ranges.clear();
        ROS_INFO_STREAM("SS New scan range size: " << resultScan.ranges.size());
        
        poseMapFrame_ = world_to_map_coordinates(robPoseWorldFrame_, map_);         // get the pose in map coordinates, i.e. cell loation

        // config for scan iteration
        // by adding yaw to start angle you have applied the rotation to the scan
        float start_angle = poseMapFrame_.yaw + scan_.angle_min;
        float incr_angle = scan_.angle_increment;
        float end_angle = start_angle + scan_.angle_max;

        ROS_INFO_STREAM("SS Angles: " << start_angle << " " << end_angle);

        geometry_msgs::Pose laserEndPntWrldFrame;               // cell location potentially a laser end point as a pose in world coordinate
        geometry_msgs::Pose maxRangeWrldFrame;                  // this is the potential location where an obstacle might be
        PoseDiscrete maxLaserEndPntMapFrame;                    // potential obstacle in map frame to check if it is occupied or not

        int flag = 0;                                           // set flag variable to check if any of the cells were obstacles
        std::int8_t indxValue;                                  // holds return value from checkObstacle function
        PoseDiscrete cellToCheck;                               // store each cell coordinate as a discrete pose
        float distToObst;                                       // holds distance between robot pose and cell that has an obstacle
        std::vector<int> cellsX, cellsY;                        // cells that will contain the line created from bresenham's algorithm

        std::array<float, 2> maxRangeCartesian = {0, 0};        // store the x, y distance of max range based on range and angle

        int count = 0;

        // TODO: check if the iteration is 360 values or +- 1?
        // for every ray starting from the heading of the robot
        for (float angle=start_angle; angle<=end_angle; angle+=incr_angle)
        {
            // TODO: check that the angular increment is greater than the grid size for a better result?

            flag = 0;                                                           // reset flag to 0 in every new iteration

            maxRangeCartesian = polar2cartesian(scan_.range_max, angle);        // translate this much distance from your pose
            maxRangeWrldFrame = robPoseWorldFrame_;                             // the world frame location of a potential obstacle
            maxRangeWrldFrame.position.x += maxRangeCartesian[0];               // essentially robPose+pol2cart(maxrange, angle)
            maxRangeWrldFrame.position.y += maxRangeCartesian[1];
            maxLaserEndPntMapFrame = world_to_map_coordinates(maxRangeWrldFrame, map_);     // the grid cell location of one laser max end point
            
            // clear cells for this iteration because this is a new line
            cellsX.clear();
            cellsY.clear();

            bresenham(poseMapFrame_, maxLaserEndPntMapFrame, cellsX, cellsY);               // the line between pose and laser end point in map frame

            // for every cell in this line
            for (int i = 0; i < cellsX.size(); i++)
            {
                // the assumption is that we are checking outwards from the robot pose
                // and that is why the first non-zero cell should do it
                // the assumption is that can deal with 0 and -1 as the same

                cellToCheck.x = cellsX.at(i);                                   // initialse the discrete pose to cell location
                cellToCheck.y = cellsY.at(i);
                cellToCheck.yaw = 0.0;

                indxValue = checkIfObstacle(cellToCheck);           // check if that gridcell is occupied or not
                if (indxValue == 0)                                             // if the cell is unoccupied go ahead and see what is next
                    continue;
                else if (indxValue == 1)                                        // this is the case when you have found the obstacle
                {
                    // find approximate distance between pose and the current cell cellToCheck
                    laserEndPntWrldFrame = map_to_world_coordinates(cellToCheck, map_);         // convert cell with obstacle to world frame
                    distToObst = poses_to_range(robPoseWorldFrame_, laserEndPntWrldFrame);      // find distance between current pose and cell
                    resultScan.ranges.emplace_back(distToObst);                                 // add distance to range array
                    
                    flag = 1;               // set flag to 1 and exit
                    break;
                }
                else if (indxValue == 2)
                {
                    // you have stumbled upon a grey area cell in the map. this means inf range, at least for now
                    // TODO: build some check to find if execution ever came here. 
                    // ideally this should never be reached
                    continue;
                }
                // end of all cells in this line
            }
            if (flag == 0)
            {
                // this means the above for loop never went into any occupied cell
                // thus the range is infinite
                resultScan.ranges.emplace_back(INFINITY);
            }
            count++;
            // end of the ray created by angle
        }
        ROS_INFO_STREAM("SS Total count of ranges: " << count << " " << resultScan.ranges.size());
        return resultScan;
    }

    sensor_msgs::LaserScan simulateScan::returnSimulatedScan()
    {
        return simulatedScan_;
    }
}