#ifndef CONSOLE
#define CONSOLE

#include "rclcpp/rclcpp.hpp"

class ConsoleNode : public rclcpp::Node
{
public:
    ConsoleNode(std::string name);
    ~ConsoleNode();
    // UI Start
    void Initial();
};
#endif