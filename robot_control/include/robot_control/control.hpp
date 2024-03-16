#ifndef CONTROL
#define CONTROL

#include "robot_control/robot_interfaces.hpp"
#include "calculator/marco.hpp"
#include "calculator/function.hpp"

class RobotNode : public rclcpp::Node
{ 
public:
    // 构造函数
    RobotNode(std::string name);
    // 获取参数
    void GetParameter();
    // 发布tf
    void Pubtf();
    // 获取里程计信息
    void SubOdomInfo(const nav_msgs::msg::Odometry::SharedPtr info);
    // 请求任务/发送自身位置
    void GetTask(int type);
    // 请求任务回调函数
    void GetTaskCallBack(rclcpp::Client<interfaces::srv::GetTask>::SharedFuture response);
    // 状态检测函数
    void CheckState();
    // 运动控制
    void PubMotionControl(int state);
    // 关节数据
    void PubJointState();

private:
    // property
    RobotInfo robot;
    RobotState state;     // {1:working,2:relaxing,3:outline}
    Coordinate origin;
    Coordinate position;
    WheelPosition wheel;
    Quaternion quaternion;
    std::vector<Coordinate> path;

    // service
    rclcpp::Client<interfaces::srv::GetTask>::SharedPtr TaskService;

    // topic
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr PublisherMotionControl;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr PublisherJointState;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr SubscriptionOdomInfo;

    // timer
    rclcpp::TimerBase::SharedPtr CheckStateTimer;
    rclcpp::TimerBase::SharedPtr JointStateTimer;
    rclcpp::TimerBase::SharedPtr PubTfTimer;
};

#endif