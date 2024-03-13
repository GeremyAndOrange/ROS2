#include "robot_control/control.hpp"

// 构造函数,有一个参数为节点名称
RobotNode::RobotNode(std::string name) : Node(name)
{
    // get id
    this->declare_parameter<std::string>("id", "-1");
    this->id = this->get_parameter("id").as_string();
    RCLCPP_INFO(this->get_logger(), ("This is robot_" + this->id).c_str());

    // parameter
    this->state = 2;
    this->left_wheel_postion = 0.0;
    this->right_wheel_position = 0.0;

    // service
    this->TaskService = this->create_client<interfaces::srv::GetTask>("get_task");

    // topic
    this->PublisherJointState = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",10);
    this->SubscriptionOdomInfo = this->create_subscription<nav_msgs::msg::Odometry>("odom",10,std::bind(&RobotNode::SubOdomInfo,this,_1));

    // timer
    this->CheckStateTimer = this->create_wall_timer(std::chrono::milliseconds(50),std::bind(&RobotNode::CheckState,this));
    this->JointStateTImer = this->create_wall_timer(std::chrono::milliseconds(50),std::bind(&RobotNode::PubJointState,this));
}

void RobotNode::SubOdomInfo(const nav_msgs::msg::Odometry::SharedPtr info)
{
    this->position.x = info->pose.pose.position.x;
    this->position.y = info->pose.pose.position.y;
    this->position.z = info->pose.pose.position.z;

    this->quaternion.x = info->pose.pose.orientation.x;
    this->quaternion.y = info->pose.pose.orientation.y;
    this->quaternion.z = info->pose.pose.orientation.z;
    this->quaternion.w = info->pose.pose.orientation.w;
}

void RobotNode::PubMotionControl(const int state)
{
    if (state != 1 || state != 0) return;
    Speed speed = CalSpeed(this->position,this->path[0],this->quaternion);
    geometry_msgs::msg::Twist info;
    info.angular.z = speed.angular * state;
    info.linear.x = speed.linear[0] * state;
    info.linear.y = speed.linear[1] * state;
    info.linear.z = speed.linear[2] * state;

    this->PublisherMotionControl = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    this->PublisherMotionControl->publish(info);
}

void RobotNode::CheckState()
{
    this->GetTask(1);
    switch (this->state)
    {
    case 1:
        this->PubMotionControl(1);
        if (CalDistance(this->position,this->path[0]) < PRECISION_1)
        {
            this->path.erase(this->path.begin());
            this->PubMotionControl(0);
            this->state = 2;
        }
        
        break;
    case 2:
        if (!this->path.empty())
        {
            this->PubMotionControl(1);
            this->state = 1;
        }
        else
        {
            this->GetTask(0);
        }
        break;
    case 3:
        break;
    default:
        break;
    }
}

void RobotNode::GetTask(const int type)
{
    // type {0: GetTask(), 1: SendPosition()}
    auto request = std::make_shared<interfaces::srv::GetTask_Request>();
    request->state = 2;
    request->id = std::stoi(this->id);
    request->x = this->position.x;
    request->y = this->position.y;
    if (type == 1) this->TaskService->async_send_request(request,std::bind(&RobotNode::GetTaskCallBack,this,_1));
    if (type == 0) this->TaskService->async_send_request(request);
}

void RobotNode::GetTaskCallBack(rclcpp::Client<interfaces::srv::GetTask>::SharedFuture response)
{
    Coordinate coordinate;
    auto result = response.get();
    for (size_t i = 0; i < result->path.size(); i += 2)
    {
        coordinate.x = result->path[i];
        coordinate.y = result->path[i+1];
        coordinate.z = 0;
        this->path.push_back(coordinate);
    }
}

void RobotNode::PubJointState()
{
    sensor_msgs::msg::JointState info;
    info.header.stamp = this->now();
    info.header.frame_id = "";
    info.name.push_back("left_wheel_joint");
    info.name.push_back("right_wheel_joint");
    if (this->path.empty())
    {
        info.velocity.push_back(0.0);
        info.velocity.push_back(0.0);
        info.position.push_back(this->left_wheel_postion);
        info.position.push_back(this->right_wheel_position);
    }
    else
    {
        Speed speed = CalSpeed(this->position,this->path[0],this->quaternion);
        double linear_speed = sqrt(pow(speed.linear[0],2) + pow(speed.linear[1],2));
        double angular_speed = speed.angular * DISTANCE_WHEEL / 2;
        info.velocity.push_back(linear_speed - angular_speed);
        info.velocity.push_back(linear_speed + angular_speed);
        this->left_wheel_postion += info.velocity[0] * (1/50);
        this->right_wheel_position += info.velocity[1] * (1/50);
        info.position.push_back(this->left_wheel_postion);
        info.position.push_back(this->right_wheel_position);

    }
    this->PublisherJointState->publish(info);
}