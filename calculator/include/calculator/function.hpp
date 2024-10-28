#ifndef FUNCTION
#define FUNCTION

#include "common.hpp"
#include "marco.hpp"

void GetRPY(Quaternion quaternion, EulerDegree* euler_degree);

double CalDistance(Coordinate start_point, Coordinate end_point);

void CalSpeed(Coordinate start_point, Coordinate end_point, Quaternion quaternion, CmlSpeed* return_speed);

#endif