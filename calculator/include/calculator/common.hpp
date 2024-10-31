#ifndef COMMON
#define COMMON

#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;

typedef struct _Coordinate {
    double x;
    double y;
    _Coordinate()
    {
        x = 0;
        y = 0;
    }
    _Coordinate(double param_x, double param_y)
    {
        x = param_x;
        y = param_y;
    }
    bool operator==(const _Coordinate& other) const {
        return x == other.x && y == other.y;
    }
    bool operator!=(const _Coordinate& other) const {
        return x != other.x || y != other.y;
    }
} Coordinate;

typedef struct _Quaternion {
    double x;
    double y;
    double z;
    double w;
    _Quaternion()
    {
        x = 0;
        y = 0;
        z = 0;
        w = 0;
    }
    _Quaternion(double param_x, double param_y, double param_z, double param_w)
    {
        x = param_x;
        y = param_y;
        z = param_z;
        w = param_w;
    }
} Quaternion;

typedef struct _EulerDegree {
    double roll;
    double pitch;
    double yaw;
    _EulerDegree()
    {
        roll = 0;
        pitch = 0;
        yaw = 0;
    }
} EulerDegree;

typedef struct _CmlSpeed {
    double linear[2];
    double angular;
    _CmlSpeed()
    {
        linear[0] = 0;
        linear[1] = 0;
        angular = 0;
    }
} CmlSpeed;

typedef enum _RobotState {
    WORK,
    RELAX,
    WAIT,
    ASTERN
} RobotState;

typedef struct _RobotInfo {
    std::string id;
    std::string name;
    Coordinate coor;
    Coordinate tf;
    Quaternion quat;
    RobotState state;
    _RobotInfo()
    {
        id = "";
        name = "";
        state = RELAX;
    }
} RobotInfo;

#endif