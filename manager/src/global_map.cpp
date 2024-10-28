#include "global_map/global_map.hpp"

GlobalMap::GlobalMap(std::string name) : Node(name)
{
    RCLCPP_INFO(this->get_logger(), "This is globalMap node.");
    this->Initial();
}

GlobalMap::~GlobalMap()
{

}

void GlobalMap::Initial()
{
    // ptr
    TfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    TfListener = std::make_shared<tf2_ros::TransformListener>(*TfBuffer);
    
    // service

    // topic
    this->PublisherGlobalMap = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/globalmap",10);
    this->SubscriptionSubmap = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/submap",10,std::bind(&GlobalMap::SubMap,this,_1));

    // timer
    this->MapTimer = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&GlobalMap::PubMap,this));
}

void GlobalMap::SubMap(const nav_msgs::msg::OccupancyGrid::SharedPtr info)
{
    std::lock_guard<std::mutex> lock(this->MapMutex);
    this->StoredMap = *info;
}

void GlobalMap::PubMap()
{
    nav_msgs::msg::OccupancyGrid TransformMap;
    std::lock_guard<std::mutex> lock(this->MapMutex);
    if (StoredMap.data.empty()) {
        return;
    }

    TransformMap = this->StoredMap;
    TransformMap.header.frame_id = "world";

    this->PublisherGlobalMap->publish(TransformMap);
}