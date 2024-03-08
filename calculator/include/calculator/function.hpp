#ifndef FUNCTION
#define FUNCTION

#include "common.hpp"
#include "marco.hpp"

void ModifyURDF(const std::string& src_file, const std::string& dst_file, const std::string& robot_name);

EulerDegree GetRPY(Quaternion quaternion);

Quaternion SetRPY(EulerDegree euler_degree);

Speed CalSpeed(Coordinate start_point, Coordinate end_point, Quaternion quaternion);

double CalDistance(Coordinate start_point, Coordinate end_point);

#endif