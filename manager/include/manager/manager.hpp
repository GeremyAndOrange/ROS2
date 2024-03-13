#ifndef MANAGER
#define MANAGER

#include "manager/manager_interfaces.hpp"
#include "calculator/marco.hpp"
#include "calculator/function.hpp"

class ManagerNode : public rclcpp::Node
{
public:
    // 构造函数
    ManagerNode(std::string name);
    // 生成行为体函数
    void NewRobot(const interfaces::srv::NewRobot::Request::SharedPtr request, const interfaces::srv::NewRobot::Response::SharedPtr response);
    // 回复任务
    void SendTask(const interfaces::srv::GetTask::Request::SharedPtr request, const interfaces::srv::GetTask::Response::SharedPtr response);

private:
    // parameter
    unsigned int number;
    unsigned int id[ROBOT_NUMBER];

    // service
    rclcpp::Service<interfaces::srv::NewRobot>::SharedPtr new_robot_service;
    rclcpp::Service<interfaces::srv::GetTask>::SharedPtr send_task_service;

    // topic

    // timer

};

#endif