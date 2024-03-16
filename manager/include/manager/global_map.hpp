#ifndef GLOBALMAP
#define GLOBALMAP

#include "manager/global_map_interfaces.hpp"
#include "calculator/marco.hpp"
#include "calculator/function.hpp"

class GlobalMap : public rclcpp::Node
{
public:
    // 构造函数
    GlobalMap(std::string name);
    // 初始化
    void Initial();
    // 监听tf
    void SubTF(const tf2_msgs::msg::TFMessage::SharedPtr info);
    // 接收子图
    void SubMap(const nav_msgs::msg::OccupancyGrid::SharedPtr info);
    // 发送地图
    void PubMap();

private:
    // property
    Map map;
    RobotTF tfGroup[ROBOT_NUMBER];

    // service

    // topic
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr PublisherMap;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr SubscriptionSubmap;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr SubscriptionTF;

    // timer
    rclcpp::TimerBase::SharedPtr MapTimer;
};

#endif