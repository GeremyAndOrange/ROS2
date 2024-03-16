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

struct Speed {
    // [x,y]
    double linear[2];
    // [z]
    double angular;
};

struct WheelPosition {
    double left;
    double right;
};

enum RobotState {
    work,
    relax,
    outline
};

struct RobotInfo {
    std::string id;
    std::string name;
};

enum Existence {
    yes,
    no
};

struct RobotMap {
    Existence existence;
    Coordinate postion;
};

struct Map {
    int height;
    int width;
    Coordinate origin;
    std::vector<int8_t> data;
};

struct RobotTF {
    Coordinate tf;
};

#endif