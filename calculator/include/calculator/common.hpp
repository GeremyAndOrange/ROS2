#ifndef COMMON
#define COMMON

#include "rclcpp/rclcpp.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <algorithm>
#include <deque>

using std::placeholders::_1;
using std::placeholders::_2;

struct Coordinate {
    double x;
    double y;
    double z;
};

struct Quaternion {
    double x;
    double y;
    double z;
    double w;
};

struct EulerDegree {
    double roll;
    double pitch;
    double yaw;
};

struct Speed
{
    // [x,y,z]
    double linear[3];
    // [z]
    double angular;
};

struct LocalMap
{
    std::vector<float> lidar_info;
    Coordinate robot_origin_position;
    Quaternion robot_origin_quaternion;
};

struct GlobalMap
{
    std::deque<std::deque<int>> map;
    Coordinate origin;
};

#endif