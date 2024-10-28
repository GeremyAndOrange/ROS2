#ifndef MANAGER
#define MANAGER

#include "manager/manager_interfaces.hpp"
#include "calculator/marco.hpp"
#include "calculator/function.hpp"

class ManagerNode : public rclcpp::Node
{
public:
    ManagerNode(std::string name);
    ~ManagerNode();

private:
    void Initial();

private:
    void ExpansionMap();
    void AggregateTask(std::string RobotId);
    void RRTAlgorithm();

private:
    void SubMap(const nav_msgs::msg::OccupancyGrid::SharedPtr info);

private:
    void SendTask(const interfaces::srv::GetTask::Request::SharedPtr request, const interfaces::srv::GetTask::Response::SharedPtr response);

private:
    // property
    std::mutex MapMutex;
    std::map<std::string, RobotInfo> RobotGroup;
    nav_msgs::msg::OccupancyGrid StoredMap;
    nav_msgs::msg::OccupancyGrid ExpansionedMap;

    // service
    rclcpp::Service<interfaces::srv::GetTask>::SharedPtr SendTaskService;

    // topic
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr SubscriptionGLobalMap;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr PublisherExpansionedMap;

    // timer
    
};

#endif