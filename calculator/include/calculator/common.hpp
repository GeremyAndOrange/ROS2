#ifndef COMMON
#define COMMON

#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;

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

#endif