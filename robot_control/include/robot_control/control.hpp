#ifndef CONTROL
#define CONTROL

#include "robot_control/robot_interfaces.hpp"
#include "calculator/marco.hpp"
#include "calculator/function.hpp"

class RobotNode : public rclcpp::Node
{ 
public:
    RobotNode(std::string name);
    ~RobotNode();

private:
    void CheckState();
    void StateChange();
    void GetParameter();
    void InitialVariable();

private:
    void UpdatePathInfo();
    void TfBroadcast();
    Coordinate CoorTrans(Coordinate self, Coordinate tf);
    void TfPoint(const geometry_msgs::msg::PointStamped& point, geometry_msgs::msg::PointStamped& TransformedPoint);

private:
    void GetTask();
    void GetTaskCallBack(rclcpp::Client<interfaces::srv::GetTask>::SharedFuture response);

private:
    void CheckCollision(const sensor_msgs::msg::LaserScan::SharedPtr info);
    void PubMotionControl();

private:
    void SubOdomInfo(const nav_msgs::msg::Odometry::SharedPtr info);

private:
    //property
    RobotInfo robot;
    std::vector<Coordinate> path;

    // ptr
    std::unique_ptr<tf2_ros::Buffer> TfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> TfListener;
    std::shared_ptr<tf2_ros::TransformBroadcaster> TfBroadcaster;

    // service
    rclcpp::Client<interfaces::srv::GetTask>::SharedPtr TaskService;

    // topic
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr PublisherMotionControl;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr PublisherPathPoints;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr SubscriptionOdomInfo;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr SubscriptionLaserInfo;

    // timer
    rclcpp::TimerBase::SharedPtr CheckStateTimer;
    rclcpp::TimerBase::SharedPtr StateChangeTimer;
    rclcpp::TimerBase::SharedPtr TfTimer;
};

#endif