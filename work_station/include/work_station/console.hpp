#ifndef CONSOLE
#define CONSOLE

#include "work_station/console_interfaces.hpp"
#include "rclcpp/rclcpp.hpp"

class ConsoleNode : public rclcpp::Node
{
public:
    // 构造函数
    ConsoleNode(std::string name);
    // 初始化
    void Initial();

private:
    // property

    // service

    // topic

    // timer

};

#endif