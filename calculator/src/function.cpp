#include "calculator/function.hpp"

void GetRPY(Quaternion quaternion, EulerDegree* euler_degree)
{
    euler_degree->roll = atan2(2.0 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z), 1.0 - 2.0 * (quaternion.x * quaternion.x + quaternion.y * quaternion.y));
    euler_degree->pitch = asin(2.0 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x));
    euler_degree->yaw = atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y), 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z));
}

void CalSpeed(Coordinate start_point, Coordinate end_point, Quaternion quaternion, CmlSpeed* return_speed)
{
    double delta_x = end_point.x - start_point.x;
    double delta_y = end_point.y - start_point.y;
    double target_yaw_theta = atan2(delta_y, delta_x);

    EulerDegree euler_degree;
    GetRPY(quaternion, &euler_degree);
    double delta_yaw_theta = fmod(target_yaw_theta - euler_degree.yaw + M_PI, 2 * M_PI) - M_PI;

    if (delta_yaw_theta > -ANGULAR_SPEED_MAX && delta_yaw_theta < ANGULAR_SPEED_MAX) {
        return_speed->angular = delta_yaw_theta > 0 ? std::min(2 * delta_yaw_theta, ANGULAR_SPEED_MAX) : std::max(2 * delta_yaw_theta, -ANGULAR_SPEED_MAX);
        return_speed->linear[0] = std::max(0.1 * delta_x, LINEAR_SPEED_MIN);
        return_speed->linear[1] = std::max(0.1 * delta_y, LINEAR_SPEED_MIN);
    }
    else {
        return_speed->angular = delta_yaw_theta > 0 ? std::max(2 * delta_yaw_theta, ANGULAR_SPEED_MAX) : std::min(2 * delta_yaw_theta, -ANGULAR_SPEED_MAX);
        return_speed->linear[0] = 0;
        return_speed->linear[1] = 0;
    }
}

double CalDistance(Coordinate start_point, Coordinate end_point)
{
    double delta_x = end_point.x - start_point.x;
    double delta_y = end_point.y - start_point.y;
    double distance = sqrt((pow(delta_x,2) + pow(delta_y,2)));
    return distance;
}