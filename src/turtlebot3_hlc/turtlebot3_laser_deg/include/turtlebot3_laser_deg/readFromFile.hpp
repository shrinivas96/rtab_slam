#pragma once

#include <fstream>
#include <sstream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

std::vector<nav_msgs::Odometry> readOdomFromFile(const std::string &file_path);


std::vector<sensor_msgs::LaserScan> readScanFromFile(const std::string &file_path);