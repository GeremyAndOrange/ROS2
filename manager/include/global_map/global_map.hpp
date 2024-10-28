#ifndef GLOBALMAP
#define GLOBALMAP

#include "global_map/global_map_interfaces.hpp"
#include "calculator/marco.hpp"
#include "calculator/function.hpp"

class GlobalMap : public rclcpp::Node
{
public:
    GlobalMap(std::string name);
    ~GlobalMap();

private:
    void Initial();

private:
    void SubMap(const nav_msgs::msg::OccupancyGrid::SharedPtr info);

private:
    void PubMap();

private:
    // property
    std::mutex MapMutex; 
    nav_msgs::msg::OccupancyGrid StoredMap;

    // ptr
    std::unique_ptr<tf2_ros::Buffer> TfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> TfListener;

    // topic
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr PublisherGlobalMap;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr SubscriptionSubmap;

    // timer
    rclcpp::TimerBase::SharedPtr MapTimer;
};

#endif