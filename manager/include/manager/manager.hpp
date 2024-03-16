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
    // 初始化
    void Initial();
    // 生成机器
    void NewRobot(const interfaces::srv::NewRobot::Request::SharedPtr request, const interfaces::srv::NewRobot::Response::SharedPtr response);
    // 回复任务
    void SendTask(const interfaces::srv::GetTask::Request::SharedPtr request, const interfaces::srv::GetTask::Response::SharedPtr response);

private:
    // property
    RobotMap group[ROBOT_NUMBER];

    // service
    rclcpp::Service<interfaces::srv::NewRobot>::SharedPtr NewRobotService;
    rclcpp::Service<interfaces::srv::GetTask>::SharedPtr SendTaskService;

    // topic

    // timer

};

#endif