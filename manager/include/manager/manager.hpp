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
    bool ScanBoundary(Coordinate PointInMap, std::vector<Coordinate>& BoundaryPoints);
    bool AggregateTask(const std::vector<Coordinate>& BoundaryPoints, std::vector<Coordinate>& TaskPoints);
    void DijsktraAlgorithm(Coordinate SourcePoint, Coordinate TargetPoint, std::vector<double>& TaskPathPlanning);

private:
    bool IsValidPoint(Coordinate point);
    Coordinate GenerateRandomNode();
    Coordinate FindNewNode(Coordinate LastNode, Coordinate NextNode);

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

    // ptr
    std::unique_ptr<tf2_ros::Buffer> TfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> TfListener;

    // service
    rclcpp::Service<interfaces::srv::GetTask>::SharedPtr SendTaskService;

    // topic
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr SubscriptionGLobalMap;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr PublisherExpansionedMap;

    // timer
    
};

#endif