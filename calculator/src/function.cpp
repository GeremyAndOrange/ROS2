#include "calculator/function.hpp"

void ModifyURDF(const std::string& src_file, const std::string& dst_file, const std::string& robot_name)
{
    std::ifstream src(src_file);
    std::ofstream dst(dst_file);
    std::string line;
    while (std::getline(src, line))
    {
        // remapping robot name
        size_t pos = line.find("<robot name=\"car_robot\">");
        if (pos != std::string::npos)
        {
            line.replace(pos, 24, "<robot name=\"" + robot_name + "\">");
        }

        // remapping node diff_drive
        pos = line.find("<plugin name='diff_drive' filename='libgazebo_ros_diff_drive.so'>");
        if (pos != std::string::npos)
        {
            line.replace(pos, 65, "<plugin name='diff_drive_" + robot_name + "' filename='libgazebo_ros_diff_drive.so'>");
        }
        // remapping node imu_plugin
        pos = line.find("<plugin filename=\"libgazebo_ros_imu_sensor.so\" name=\"imu_plugin\">");
        if (pos != std::string::npos)
        {
            line.replace(pos, 65, "<plugin filename=\"libgazebo_ros_imu_sensor.so\" name=\"imu_plugin_" + robot_name + "\">");
        }
        // remapping node laserscan
        pos = line.find("<plugin name=\"laserscan\" filename=\"libgazebo_ros_ray_sensor.so\">");
        if (pos != std::string::npos)
        {
            line.replace(pos, 64, "<plugin name=\"laserscan_" + robot_name + "\" filename=\"libgazebo_ros_ray_sensor.so\">");
        }

        // remapping topic cmd_vel
        pos = line.find("<remapping>cmd_vel:=cmd_vel</remapping>");
        if (pos != std::string::npos)
        {
            line.replace(pos, 39, "<remapping>cmd_vel:=" + robot_name + "/cmd_vel</remapping>");
        }
        // remapping topic odom
        pos = line.find("<remapping>odom:=odom</remapping>");
        if (pos != std::string::npos)
        {
            line.replace(pos, 33, "<remapping>odom:=" + robot_name + "/odom</remapping>");
        }
        // remapping topic imu
        pos = line.find("<remapping>~/out:=imu</remapping>");
        if (pos != std::string::npos)
        {
            line.replace(pos, 32, "<remapping>~/out:=" + robot_name + "/imu</remapping>");
        }
        // remmaping topic scan
        pos = line.find("<remapping>~/out:=scan</remapping>");
        if (pos != std::string::npos)
        {
            line.replace(pos, 34, "<remapping>~/out:=" + robot_name + "/scan</remapping>");
        }
        dst << line << "\n";
    }
}

EulerDegree GetRPY(Quaternion quaternion)
{
    EulerDegree euler_degree;
    euler_degree.roll = atan2(2.0 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z), 1.0 - 2.0 * (quaternion.x * quaternion.x + quaternion.y * quaternion.y));
    euler_degree.pitch = asin(2.0 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x));
    euler_degree.yaw = atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y), 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z));
    return euler_degree;
}

Quaternion SetRPY(EulerDegree euler_degree)
{
    Quaternion quaternion;
    double cy = cos(euler_degree.yaw * 0.5);
    double sy = sin(euler_degree.yaw * 0.5);
    double cr = cos(euler_degree.roll * 0.5);
    double sr = sin(euler_degree.roll * 0.5);
    double cp = cos(euler_degree.pitch * 0.5);
    double sp = sin(euler_degree.pitch * 0.5);

    quaternion.w = cy * cr * cp + sy * sr * sp;
    quaternion.x = cy * sr * cp - sy * cr * sp;
    quaternion.y = cy * cr * sp + sy * sr * cp;
    quaternion.z = sy * cr * cp - cy * sr * sp;
    return quaternion;
}

Speed CalSpeed(Coordinate start_point, Coordinate end_point, Quaternion quaternion)
{
    double delta_x = end_point.x - start_point.x;
    double delta_y = end_point.y - start_point.y;
    double target_yaw_theta = atan2(delta_y, delta_x);

    EulerDegree euler_degree = GetRPY(quaternion);
    double delta_yaw_theta = fmod(target_yaw_theta - euler_degree.yaw + M_PI, 2 * M_PI) - M_PI;

    Speed return_speed;
    return_speed.angular = std::min(20 * PROPORTIONAL_COEFFICIENT * delta_yaw_theta, ANGULAR_SPEED_MAX);
    return_speed.linear[0] = std::max(PROPORTIONAL_COEFFICIENT * delta_x, LINEAR_SPEED_MIN);
    return_speed.linear[1] = std::max(PROPORTIONAL_COEFFICIENT * delta_y, LINEAR_SPEED_MIN);
    return_speed.linear[2] = 0;
    return return_speed;
}

double CalDistance(Coordinate start_point, Coordinate end_point)
{
    double delta_x = end_point.x - start_point.x;
    double delta_y = end_point.y - start_point.y;
    double distance = sqrt((pow(delta_x,2) + pow(delta_y,2)));
    return distance;
}