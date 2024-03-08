#ifndef MARCO
#define MARCO

// STRING DEFINE
#define SPAWN_ENTITY "ros2 run gazebo_ros spawn_entity.py -entity "
#define ROBOT_CONTROL "ros2 run robot_control robot_node --ros-args"
#define SRCURDF "/home/glj/robot/src/robot_description/urdf/robot.urdf"
#define TEMURDF "/home/glj/robot/src/robot_description/urdf/"

// NUMBER DEFINE
#define ROBOT_NUMBER 99
#define PROPORTIONAL_COEFFICIENT 0.1
#define PRECISION_1 0.1
#define PRECISION_2 0.01
#define PRECISION_3 0.001
#define PRECISION_4 0.0001
#define ANGULAR_SPEED_MAX 3.0
#define LINEAR_SPEED_MIN 0.5

// SENSOR DEFINE
#define LIDAR_RANGE_MAX 3.5         // DEFINE IN .URDF
#define LIDAR_SCAN_NUMBER 360       // DEFINE IN .URDF

// MAP DEFINE
#define MAP_RESOLUTION 0.05
#define MAP_LENGTH 140              // INT(LIDAR_RANGE_MAX/MAP_RESOLUTION)*2

#endif