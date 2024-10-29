#include "robot_control/control.hpp"

// 构造函数,有一个参数为节点名称
RobotNode::RobotNode(std::string name) : Node(name)
{
    this->GetParameter();
    this->InitialVariable();
}

RobotNode::~RobotNode()
{
    
}

void RobotNode::InitialVariable()
{
    // ptr
    TfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    TfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    TfListener = std::make_shared<tf2_ros::TransformListener>(*TfBuffer);
    
    // service
    this->TaskService = this->create_client<interfaces::srv::GetTask>("/get_task");

    // topic
    this->PublisherMotionControl = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    this->SubscriptionOdomInfo = this->create_subscription<nav_msgs::msg::Odometry>("odom",10,std::bind(&RobotNode::SubOdomInfo,this,_1));

    // timer
    this->CheckStateTimer = this->create_wall_timer(std::chrono::milliseconds(50),std::bind(&RobotNode::CheckState,this));
    this->StateChangeTimer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&RobotNode::StateChange,this));
    this->TfTimer = this->create_wall_timer(std::chrono::milliseconds(50),std::bind(&RobotNode::TfBroadcast,this));
}

void RobotNode::GetParameter()
{
    this->declare_parameter<std::string>("name", "robot");
    this->declare_parameter<double>("origin_x",0.0);
    this->declare_parameter<double>("origin_y",0.0);
    this->robot.name = this->get_parameter("name").as_string();
    this->robot.id = this->robot.name.substr(this->robot.name.find('_') + 1);
    this->robot.tf.x = this->get_parameter("origin_x").as_double();
    this->robot.tf.y = this->get_parameter("origin_y").as_double();
}

void RobotNode::SubOdomInfo(const nav_msgs::msg::Odometry::SharedPtr info)
{
    this->robot.coor.x = info->pose.pose.position.x;
    this->robot.coor.y = info->pose.pose.position.y;

    this->robot.quat.x = info->pose.pose.orientation.x;
    this->robot.quat.y = info->pose.pose.orientation.y;
    this->robot.quat.z = info->pose.pose.orientation.z;
    this->robot.quat.w = info->pose.pose.orientation.w;
}

void RobotNode::PubMotionControl()
{
    CmlSpeed speed;
    CalSpeed(CoorTrans(this->robot.coor,this->robot.tf),this->path[0],this->robot.quat,&speed);
    geometry_msgs::msg::Twist info;
    info.angular.z = speed.angular;
    info.linear.x = speed.linear[0];
    info.linear.y = speed.linear[1];

    this->PublisherMotionControl->publish(info);
}

void RobotNode::UpdatePathInfo()
{
    if (CalDistance(CoorTrans(this->robot.coor,this->robot.tf), this->path[0]) < PRECISION_1) {
        this->path.erase(this->path.begin());
        if (this->path.empty()) {
            geometry_msgs::msg::Twist info; 
            info.angular.z = 0;
            info.linear.x = 0;
            info.linear.y = 0;
            this->PublisherMotionControl->publish(info);

            this->robot.state = RELAX;
        }
    }
}

void RobotNode::CheckState()
{
    if (this->robot.state == RELAX) {
        this->GetTask();
    }

    if (this->robot.state == WORK) {
        this->UpdatePathInfo();
        this->PubMotionControl();   
    }
}

void RobotNode::GetTask()
{
    auto request = std::make_shared<interfaces::srv::GetTask_Request>();
    request->id = this->robot.id;
    request->coor_x = this->robot.coor.x;
    request->coor_y = this->robot.coor.y;
    request->tf_x = this->robot.tf.x;
    request->tf_y = this->robot.tf.y;
    this->TaskService->async_send_request(request,std::bind(&RobotNode::GetTaskCallBack,this,_1));
    this->robot.state = WAIT;
    this->StateChangeTimer->reset();
}

void RobotNode::GetTaskCallBack(rclcpp::Client<interfaces::srv::GetTask>::SharedFuture response)
{
    Coordinate coordinate;
    auto result = response.get();

    if (result->is_path) {
        for (size_t i = 0; i < result->path.size(); i += 2) {
            coordinate.x = result->path[i];
            coordinate.y = result->path[i+1];
            this->path.push_back(coordinate);
        }
        this->robot.state = WORK;
        this->StateChangeTimer->cancel();
    }
}

void RobotNode::StateChange()
{
    if (this->robot.state == WAIT) {
        this->robot.state = RELAX;
    }
}

Coordinate RobotNode::CoorTrans(Coordinate self, Coordinate tf)
{
    Coordinate transformation;
    transformation.x = self.x + tf.x;
    transformation.y = self.y + tf.y;
    return transformation;
}

void RobotNode::TfBroadcast()
{
    geometry_msgs::msg::TransformStamped WorldToSubmap;
    WorldToSubmap.header.frame_id = "world";
    WorldToSubmap.child_frame_id = "robot_" + this->robot.id + "_submap";
    WorldToSubmap.header.stamp = this->get_clock()->now();

    WorldToSubmap.transform.translation.x = -this->robot.tf.x;
    WorldToSubmap.transform.translation.y = -this->robot.tf.y;
    WorldToSubmap.transform.translation.z = 0.0;

    WorldToSubmap.transform.rotation.x = 0.0;
    WorldToSubmap.transform.rotation.y = 0.0;
    WorldToSubmap.transform.rotation.z = 0.0;
    WorldToSubmap.transform.rotation.w = 1.0;

    TfBroadcaster->sendTransform(WorldToSubmap);
}

void RobotNode::TfPoint(const geometry_msgs::msg::PointStamped& point, geometry_msgs::msg::PointStamped& TransformedPoint)
{
    try {
        TfBuffer->transform(point,TransformedPoint,"world");
    }
    catch(const tf2::TransformException &error) {
        RCLCPP_ERROR(this->get_logger(), "TF Exception: %s", error.what());
    }
}