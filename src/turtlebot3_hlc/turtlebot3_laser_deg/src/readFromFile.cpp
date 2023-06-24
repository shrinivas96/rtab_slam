#include "turtlebot3_laser_deg/readFromFile.hpp"

std::vector<nav_msgs::Odometry> readOdomFromFile(const std::string &file_path)
{
    std::vector<nav_msgs::Odometry> odom_list;

    std::ifstream file(file_path);
    std::string line;
    std::string flag;
    nav_msgs::Odometry odom;

    while (std::getline(file, line))
    {
        if (line.find("header:") != std::string::npos)
        {
            // after header is always seq, so just get that line and save it
            std::getline(file, line);
            std::istringstream iss(line.substr(line.find(":") + 1));
            iss >> odom.header.seq;
        }
        else if (line.find("stamp:") != std::string::npos)
        {
            // after stamp is alwats secs and nsecs
            std::getline(file, line);
            std::istringstream secs_stream(line.substr(line.find("secs:") + 6));
            secs_stream >> odom.header.stamp.sec;

            // again get to the next line
            std::getline(file, line);
            std::istringstream nsecs_stream(line.substr(line.find("nsecs:") + 7));
            nsecs_stream >> odom.header.stamp.nsec;
        }
        else if (line.find("child_frame_id:") != std::string::npos)
        {
            std::istringstream iss(line.substr(line.find(":") + 1));
            iss >> odom.child_frame_id;
        }
        else if (line.find("frame_id:") != std::string::npos)
        {
            // frame id kept after child because frame_id matches both cases, so it needs to check for child frame_id first
            std::istringstream iss(line.substr(line.find(":") + 1));
            iss >> odom.header.frame_id;
        }
        else if (line.find("pose:") != std::string::npos)
        {
            flag = "pose";
            // once you find pose the next two lines are pose and position. then starts the x, y, z
            std::getline(file, line);           // this would contain pose 
            std::getline(file, line);           // this would contain position 

            // the next line should have x values and so on
            std::getline(file, line);
            std::istringstream x_stream(line.substr(line.find("x:") + 2));
            x_stream >> odom.pose.pose.position.x;
            
            std::getline(file, line);
            std::istringstream y_stream(line.substr(line.find("y:") + 2));
            y_stream >> odom.pose.pose.position.y;
            
            std::getline(file, line);
            std::istringstream z_stream(line.substr(line.find("z:") + 2));
            z_stream >> odom.pose.pose.position.z;

            // now starts the lines for orientation
            std::getline(file, line);           // this would contain orientation

            // the next line starts the x, y, z, w
            std::getline(file, line);
            std::istringstream ox_stream(line.substr(line.find("x:") + 2));
            ox_stream >> odom.pose.pose.orientation.x;
            
            std::getline(file, line);
            std::istringstream oy_stream(line.substr(line.find("y:") + 2));
            oy_stream >> odom.pose.pose.orientation.y;
            
            std::getline(file, line);
            std::istringstream oz_stream(line.substr(line.find("z:") + 2));
            oz_stream >> odom.pose.pose.orientation.z;
            
            std::getline(file, line);
            std::istringstream ow_stream(line.substr(line.find("w:") + 2));
            ow_stream >> odom.pose.pose.orientation.w;
        }
        else if (line.find("covariance:") != std::string::npos)
        {
            // the line contains "covariance: [..." thus after finding : +3 to get values
            std::string covArray = line.substr(line.find(":")+3);
            covArray.pop_back();            // remove trailing close bracket character

            std::stringstream cov_stream(covArray);
            float cov_value;
            int count = 0;
            
            while (cov_stream >> cov_value)
            {
                if (count >= 36)
                    break;
                if(cov_stream.peek() == ',')
                    cov_stream.ignore();
                if (flag == "pose")
                    odom.pose.covariance[count] = cov_value;
                else if (flag == "twist")
                    odom.twist.covariance[count] = cov_value;
                count++;
            }
            if (flag == "twist")
                odom_list.push_back(odom);
        }
        else if (line.find("twist:") != std::string::npos)
        {
            flag = "twist";
            // once you find twist the next two lines are twist and linear. then starts the x, y, z
            std::getline(file, line);           // this would contain twist
            std::getline(file, line);           // this would contain linear 

            std::getline(file, line);
            std::istringstream x_stream(line.substr(line.find("x:") + 2));
            x_stream >> odom.twist.twist.linear.x;
            
            std::getline(file, line);
            std::istringstream y_stream(line.substr(line.find("y:") + 2));
            y_stream >> odom.twist.twist.linear.y;
            
            std::getline(file, line);
            std::istringstream z_stream(line.substr(line.find("z:") + 2));
            z_stream >> odom.twist.twist.linear.z;

            // now starts lines for angular values
            std::getline(file, line);           // this would contain angular

            std::getline(file, line);
            std::istringstream ax_stream(line.substr(line.find("x:") + 2));
            ax_stream >> odom.twist.twist.angular.x;
            
            std::getline(file, line);
            std::istringstream ay_stream(line.substr(line.find("y:") + 2));
            ay_stream >> odom.twist.twist.angular.y;
            
            std::getline(file, line);
            std::istringstream az_stream(line.substr(line.find("z:") + 2));
            az_stream >> odom.twist.twist.angular.z;
        }
    }

    file.close();
    return odom_list;
}


std::vector<sensor_msgs::LaserScan> readScanFromFile(const std::string &file_path)
{
    std::vector<sensor_msgs::LaserScan> scan_list;
    std::ifstream file(file_path);
    std::string line;
    sensor_msgs::LaserScan scans;

    while (std::getline(file, line))
    {
        if (line.find("header:") != std::string::npos)
        {
            // after header is always seq, so just get that line and save it
            std::getline(file, line);
            std::istringstream iss(line.substr(line.find(":") + 1));
            iss >> scans.header.seq;
        }
        else if (line.find("stamp:") != std::string::npos)
        {
            // after stamp is alwats secs and nsecs
            std::getline(file, line);
            std::istringstream secs_stream(line.substr(line.find("secs:") + 6));
            secs_stream >> scans.header.stamp.sec;

            // again get to the next line
            std::getline(file, line);
            std::istringstream nsecs_stream(line.substr(line.find("nsecs:") + 7));
            nsecs_stream >> scans.header.stamp.nsec;
        }
        else if (line.find("frame_id:") != std::string::npos)
        {
            std::string scan_frame_id = line.substr(line.find(":") + 3);
            scan_frame_id.pop_back();
            std::istringstream iss(scan_frame_id);
            iss >> scans.header.frame_id;
        }
        else if (line.find("angle_min:") != std::string::npos)
        {
            std::istringstream ang_min_stream(line.substr(line.find(":") + 2));
            ang_min_stream >> scans.angle_min;
        }
        else if (line.find("angle_max:") != std::string::npos)
        {
            std::istringstream ang_max_stream(line.substr(line.find(":") + 2));
            ang_max_stream >> scans.angle_max;
        }
        else if (line.find("angle_increment:") != std::string::npos)
        {
            std::istringstream ang_incr_stream(line.substr(line.find(":") + 2));
            ang_incr_stream >> scans.angle_increment;
        }
        else if (line.find("time_increment:") != std::string::npos)
        {
            std::istringstream time_incr_stream(line.substr(line.find(":") + 2));
            time_incr_stream >> scans.time_increment;
        }
        else if (line.find("scan_time:") != std::string::npos)
        {
            std::istringstream scan_time_stream(line.substr(line.find(":") + 2));
            scan_time_stream >> scans.scan_time;
        }
        else if (line.find("range_min:") != std::string::npos)
        {
            std::istringstream rng_min_stream(line.substr(line.find(":") + 2));
            rng_min_stream >> scans.range_min;
        }
        else if (line.find("range_max:") != std::string::npos)
        {
            std::istringstream rng_max_stream(line.substr(line.find(":") + 2));
            rng_max_stream >> scans.range_max;
        }
        else if (line.find("ranges:") != std::string::npos)
        {
            std::vector<float> myRanges;

            // the line contains "ranges: [..." thus after finding : +3 to get values
            std::string array_ = line.substr(line.find(":")+3);
            array_.pop_back();            // remove trailing close bracket character

            std::string delimiter = ", ";
            size_t pos = 0;
            std::string token;
            int count = 0;
            while ((pos = array_.find(delimiter)) != std::string::npos) {
                token = array_.substr(0, pos); 
                myRanges.push_back(std::stof(token));
                // std::cout << token << std::endl;
                array_.erase(0, pos + delimiter.length());
                count++;
            }

            // last value gets left behind because there is no delimiter anymore. 
            myRanges.push_back(std::stof(array_));
            scans.ranges = myRanges;
        }
        else if (line.find("intensities:") != std::string::npos)
        {
            std::vector<float> myIntensities;

            // the line contains "intensities: [..." thus after finding : +3 to get values
            std::string array_ = line.substr(line.find(":")+3);
            array_.pop_back();            // remove trailing close bracket character
            
            std::string delimiter = ", ";
            size_t pos = 0;
            std::string token;
            int count = 0;
            while ((pos = array_.find(delimiter)) != std::string::npos) {
                token = array_.substr(0, pos); 
                myIntensities.push_back(std::stof(token));
                // std::cout << token << std::endl;
                array_.erase(0, pos + delimiter.length());
                count++;
            }

            myIntensities.push_back(std::stof(array_));
            scans.intensities = myIntensities;
            scan_list.push_back(scans);
        }
    }
    file.close();
    return scan_list;
}