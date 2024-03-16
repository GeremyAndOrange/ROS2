#include "robot_control/control.hpp"

// 构造函数,有一个参数为节点名称
RobotNode::RobotNode(std::string name) : Node(name)
{
    // parameter
    this->GetParameter();

    // property
    this->state = relax;

    // service
    this->TaskService = this->create_client<interfaces::srv::GetTask>("/get_task");

    // topic
    this->PublisherJointState = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",10);
    this->SubscriptionOdomInfo = this->create_subscription<nav_msgs::msg::Odometry>("odom",10,std::bind(&RobotNode::SubOdomInfo,this,_1));

    // timer
    this->CheckStateTimer = this->create_wall_timer(std::chrono::milliseconds(50),std::bind(&RobotNode::CheckState,this));
    this->JointStateTimer = this->create_wall_timer(std::chrono::milliseconds(50),std::bind(&RobotNode::PubJointState,this));
    this->PubTfTimer = this->create_wall_timer(std::chrono::milliseconds(50),std::bind(&RobotNode::Pubtf,this));
}

void RobotNode::GetParameter()
{
    this->declare_parameter<std::string>("name", "robot");
    this->declare_parameter<double>("origin_x",0.0);
    this->declare_parameter<double>("origin_y",0.0);
    this->robot.name = this->get_parameter("name").as_string();
    this->robot.id = this->robot.name.substr(this->robot.name.find('_') + 1);
    this->origin.x = this->get_parameter("origin_x").as_double();
    this->origin.y = this->get_parameter("origin_y").as_double();
}

void RobotNode::Pubtf()
{
    tf2_ros::TransformBroadcaster TFbroadcaster(this);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.child_frame_id = this->robot.name + "_submap";
    transform.transform.translation.x = this->origin.x;
    transform.transform.translation.y = this->origin.y;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 1.0;
    transform.header.stamp = this->get_clock()->now();
    transform.header.stamp.sec = this->get_clock()->now().seconds() + 0.5;
    TFbroadcaster.sendTransform(transform);
}

void RobotNode::SubOdomInfo(const nav_msgs::msg::Odometry::SharedPtr info)
{
    this->position.x = info->pose.pose.position.x;
    this->position.y = info->pose.pose.position.y;

    this->quaternion.x = info->pose.pose.orientation.x;
    this->quaternion.y = info->pose.pose.orientation.y;
    this->quaternion.z = info->pose.pose.orientation.z;
    this->quaternion.w = info->pose.pose.orientation.w;
}

void RobotNode::PubMotionControl(int state)
{
    if (state != 1 && state != 0) return;
    Speed speed = CalSpeed(this->position,this->path[0],this->quaternion);
    geometry_msgs::msg::Twist info;
    info.angular.z = speed.angular * state;
    info.linear.x = speed.linear[0] * state;
    info.linear.y = speed.linear[1] * state;

    this->PublisherMotionControl = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    this->PublisherMotionControl->publish(info);
}

void RobotNode::CheckState()
{
    this->GetTask(1);
    switch (this->state)
    {
    case work:
        this->PubMotionControl(1);
        if (CalDistance(this->position,this->path[0]) < PRECISION_1) {
            this->path.erase(this->path.begin());
            this->PubMotionControl(0);
            this->state = relax;
        }
        
        break;
    case relax:
        if (!this->path.empty()) {
            this->PubMotionControl(1);
            this->state = work;
        }
        else
        {
            this->GetTask(0);
        }
        break;
    case outline:
        break;
    default:
        break;
    }
}

void RobotNode::GetTask(int type)
{
    // type {0: GetTask(), 1: SendPosition()}
    auto request = std::make_shared<interfaces::srv::GetTask_Request>();
    request->id = std::stoi(this->robot.id);
    request->x = this->position.x;
    request->y = this->position.y;
    if (type == 0) this->TaskService->async_send_request(request,std::bind(&RobotNode::GetTaskCallBack,this,_1));
    if (type == 1) this->TaskService->async_send_request(request);
}

void RobotNode::GetTaskCallBack(rclcpp::Client<interfaces::srv::GetTask>::SharedFuture response)
{
    Coordinate coordinate;
    auto result = response.get();
    for (size_t i = 0; i < result->path.size(); i += 2) {
        coordinate.x = result->path[i];
        coordinate.y = result->path[i+1];
        this->path.push_back(coordinate);
    }
}

void RobotNode::PubJointState()
{
    sensor_msgs::msg::JointState info;
    info.header.stamp = this->get_clock()->now();
    info.header.frame_id = "";
    info.name.push_back(this->robot.name + "_left_wheel_joint");
    info.name.push_back(this->robot.name + "_right_wheel_joint");
    if (this->path.empty()) {
        info.velocity.push_back(0.0);
        info.velocity.push_back(0.0);
        info.position.push_back(this->wheel.left);
        info.position.push_back(this->wheel.right);
    } else {
        Speed speed = CalSpeed(this->position,this->path[0],this->quaternion);
        double linear_speed = sqrt(pow(speed.linear[0],2) + pow(speed.linear[1],2));
        double angular_speed = speed.angular * DISTANCE_WHEEL / 2;
        info.velocity.push_back(linear_speed - angular_speed);
        info.velocity.push_back(linear_speed + angular_speed);
        this->wheel.left += info.velocity[0] * (1/50);
        this->wheel.right += info.velocity[1] * (1/50);
        info.position.push_back(this->wheel.left);
        info.position.push_back(this->wheel.right);

    }
    this->PublisherJointState->publish(info);
}