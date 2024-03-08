#include "robot_control/control.hpp"

// 构造函数,有一个参数为节点名称
RobotNode::RobotNode(std::string name) : Node(name)
{
    // get id
    this->declare_parameter<std::string>("id", "-1");
    this->id = this->get_parameter("id").as_string();
    RCLCPP_INFO(this->get_logger(), ("This is robot_" + this->id).c_str());

    // initial
    // parameter
    this->state = 2;

    // service
    this->TaskService = this->create_client<interfaces::srv::GetTask>("get_task");

    // topic
    this->SubscriptionOdomInfo = this->create_subscription<nav_msgs::msg::Odometry>("robot_" + this->id + "/odom",10,std::bind(&RobotNode::SubOdomInfo,this,_1));
    this->SubscriptionLidarInfo = this->create_subscription<sensor_msgs::msg::LaserScan>("robot_" + this->id + "/scan", rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::BestEffort), std::bind(&RobotNode::SublidarInfo,this,_1));
    // timer
    this->CheckStateTimer = this->create_wall_timer(std::chrono::milliseconds(50),std::bind(&RobotNode::CheckState,this));
    this->LocalMapBuildTimer = this->create_wall_timer(std::chrono::milliseconds(50),std::bind(&RobotNode::LocalMapBuild,this));
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

void RobotNode::PubMotionControl()
{
    Speed speed = CalSpeed(this->position,this->path[0],this->quaternion);
    geometry_msgs::msg::Twist info;
    info.angular.z = speed.angular;
    info.linear.x = speed.linear[0];
    info.linear.y = speed.linear[1];
    info.linear.z = speed.linear[2];

    this->PublisherMotionControl = this->create_publisher<geometry_msgs::msg::Twist>("robot_" + this->id + "/cmd_vel", 10);
    this->PublisherMotionControl->publish(info);
}

void RobotNode::ClearMotionControl()
{
    geometry_msgs::msg::Twist info;
    info.angular.z = 0;
    info.linear.x = 0;
    info.linear.y = 0;
    info.linear.z = 0;
    this->PublisherMotionControl = this->create_publisher<geometry_msgs::msg::Twist>("robot_" + this->id + "/cmd_vel", 10);
    this->PublisherMotionControl->publish(info);
}

void RobotNode::CheckState()
{
    this->SendPosition();
    switch (this->state)
    {
    case 1:
        this->PubMotionControl();
        if (CalDistance(this->position,this->path[0]) < PRECISION_1)
        {
            this->path.erase(this->path.begin());
            this->ClearMotionControl();
            this->state = 2;
        }
        
        break;
    case 2:
        if (!this->path.empty())
        {
            this->PubMotionControl();
            this->state = 1;
        }
        else
        {
            this->GetTask();
        }
        break;
    case 3:
        break;
    default:
        break;
    }
}

void RobotNode::GetTask()
{
    auto request = std::make_shared<interfaces::srv::GetTask_Request>();
    request->state = 2;
    request->id = std::stoi(this->id);
    request->x = this->position.x;
    request->y = this->position.y;
    this->TaskService->async_send_request(request,std::bind(&RobotNode::GetTaskCallBack,this,_1));
}

void RobotNode::SendPosition()
{
    auto request = std::make_shared<interfaces::srv::GetTask_Request>();
    request->state = 1;
    request->id = std::stoi(this->id);
    request->x = this->position.x;
    request->y = this->position.y;
    this->TaskService->async_send_request(request);
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

void RobotNode::SublidarInfo(const sensor_msgs::msg::LaserScan::SharedPtr info)
{
    this->lidar_scan.lidar_info = info->ranges;
    this->lidar_scan.robot_origin_position = this->position;
    this->lidar_scan.robot_origin_quaternion = this->quaternion;
}

void RobotNode::LocalMapBuild()
{
    std::vector<int8_t> map(MAP_LENGTH * MAP_LENGTH);
    for (size_t i = 0; i < MAP_LENGTH; i++)
    {
        for (size_t j = 0; j < MAP_LENGTH; j++)
        {
            float coor_x = int(j-MAP_LENGTH/2) * MAP_RESOLUTION;
            float coor_y = int(i-MAP_LENGTH/2) * MAP_RESOLUTION;
            float coor_theta = atan2(coor_y,coor_x);
            int cal_theta = int((coor_theta - GetRPY(this->quaternion).yaw)*(180.0/M_PI));
            cal_theta = cal_theta < 0 ? (cal_theta%360 + 360) : cal_theta%360;
            float distance = sqrt((pow(coor_x,2) + pow(coor_y,2)));
            int index = i * MAP_LENGTH + j;
            if (distance <= this->lidar_scan.lidar_info[cal_theta])
            {
                map[index] = 0;
            }
            else if (distance > this->lidar_scan.lidar_info[cal_theta] && distance < this->lidar_scan.lidar_info[cal_theta] + MAP_RESOLUTION * 3)
            {
                map[index] = 100;
            }
            else
            {
                map[index] = -1;
            }
        }
    }

    nav_msgs::msg::OccupancyGrid info;
    info.data = map;
    info.info.origin.position.x = this->lidar_scan.robot_origin_position.x - LIDAR_RANGE_MAX;
    info.info.origin.position.y = this->lidar_scan.robot_origin_position.y - LIDAR_RANGE_MAX;

    this->PublisherMapInfo = this->create_publisher<nav_msgs::msg::OccupancyGrid>("local_map", 10);
    this->PublisherMapInfo->publish(info);
}