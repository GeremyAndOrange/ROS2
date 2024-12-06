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
    try {
        geometry_msgs::msg::TransformStamped WorldToSubmap;
        WorldToSubmap = TfBuffer->lookupTransform("world", this->StoredMap.header.frame_id, tf2::TimePointZero);

        TransformMap.info.origin.position.x = this->StoredMap.info.origin.position.x + WorldToSubmap.transform.translation.x;
        TransformMap.info.origin.position.y = this->StoredMap.info.origin.position.y + WorldToSubmap.transform.translation.y;
        TransformMap.info.origin.orientation.x = this->StoredMap.info.origin.orientation.x + WorldToSubmap.transform.rotation.x;
        TransformMap.info.origin.orientation.y = this->StoredMap.info.origin.orientation.y + WorldToSubmap.transform.rotation.y;
        TransformMap.info.origin.orientation.z = this->StoredMap.info.origin.orientation.z + WorldToSubmap.transform.rotation.z;
        TransformMap.info.origin.orientation.w = this->StoredMap.info.origin.orientation.w + WorldToSubmap.transform.rotation.w;
    }
    catch(const tf2::TransformException &error) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "TF Exception: %s", error.what());
        return;
    }

    this->PublisherGlobalMap->publish(TransformMap);
}