#ifndef MANAGER
#define MANAGER

#include "manager/manager_interfaces.hpp"
#include "calculator/marco.hpp"
#include "calculator/function.hpp"

struct RobotPosition
{
    // id
    unsigned int id;
    // [x,y,z]
    Coordinate position;
};


class ManagerNode : public rclcpp::Node
{
// topic function by Pub/Sub
// server function by Get/Send  
public:
    // 构造函数
    ManagerNode(std::string name);
    // 生成行为体函数
    void NewRobot(const interfaces::srv::NewRobot::Request::SharedPtr request, const interfaces::srv::NewRobot::Response::SharedPtr response);
    // 回复任务
    void SendTask(const interfaces::srv::GetTask::Request::SharedPtr request, const interfaces::srv::GetTask::Response::SharedPtr response);
    // 接收局部地图
    void SubLocalMap(const nav_msgs::msg::OccupancyGrid info);
    // 更新局部地图
    void UpdateMap(const nav_msgs::msg::OccupancyGrid info, int start_x, int start_y);

    //debug
    void pub();

private:
    // parameter
    unsigned int number;
    unsigned int id[ROBOT_NUMBER];
    std::vector<RobotPosition> RobotPositionList;
    GlobalMap map;

    // service
    rclcpp::Service<interfaces::srv::NewRobot>::SharedPtr new_robot_service;
    rclcpp::Service<interfaces::srv::GetTask>::SharedPtr send_task_service;

    // topic
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr SubscriptionLidarInfo;

    // timer

    // debug
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr debug;
    rclcpp::TimerBase::SharedPtr debugTimer;
};

#endif