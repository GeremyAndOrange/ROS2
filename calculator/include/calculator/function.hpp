#ifndef FUNCTION
#define FUNCTION

#include "common.hpp"
#include "marco.hpp"

EulerDegree GetRPY(Quaternion quaternion);

Speed CalSpeed(Coordinate start_point, Coordinate end_point, Quaternion quaternion);

double CalDistance(Coordinate start_point, Coordinate end_point);

#endif