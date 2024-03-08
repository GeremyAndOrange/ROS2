#ifndef CONTROL
#define CONTROL

#include "robot_control/robot_interfaces.hpp"
#include "calculator/marco.hpp"
#include "calculator/function.hpp"

class RobotNode : public rclcpp::Node
{
// topic function by Pub/Sub
// server function by Get/Send  
public:
    // 构造函数
    RobotNode(std::string name);
    // 获取里程计信息
    void SubOdomInfo(const nav_msgs::msg::Odometry::SharedPtr info);
    // 请求任务主函数
    void GetTask();
    // 发送自身位置
    void SendPosition();
    // 请求任务回调函数
    void GetTaskCallBack(rclcpp::Client<interfaces::srv::GetTask>::SharedFuture response);
    // 状态检测函数
    void CheckState();
    // 运动控制-1
    void PubMotionControl();
    // 运动控制-2
    void ClearMotionControl();
    // 接收雷达信息
    void SublidarInfo(const sensor_msgs::msg::LaserScan::SharedPtr info);
    // 构建局部地图
    void LocalMapBuild();

private:
    // parameter
    std::string id;
    Coordinate position;
    Quaternion quaternion;
    std::vector<Coordinate> path;
    LocalMap lidar_scan;
    unsigned int state;     // {1:working,2:relaxing,3:outline}

    // service
    rclcpp::Client<interfaces::srv::GetTask>::SharedPtr TaskService;

    // topic
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr PublisherMotionControl;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr PublisherMapInfo;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr SubscriptionOdomInfo;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr SubscriptionLidarInfo;

    // timer
    rclcpp::TimerBase::SharedPtr CheckStateTimer;
    rclcpp::TimerBase::SharedPtr LocalMapBuildTimer;
};

#endif