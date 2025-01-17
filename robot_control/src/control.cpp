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
    this->PublisherPathPoints = this->create_publisher<visualization_msgs::msg::Marker>("path_points", 10);
    this->SubscriptionOdomInfo = this->create_subscription<nav_msgs::msg::Odometry>("odom",10,std::bind(&RobotNode::SubOdomInfo,this,_1));
    this->SubscriptionLaserInfo = this->create_subscription<sensor_msgs::msg::LaserScan>("scan",10,std::bind(&RobotNode::CheckCollision,this,_1));

    // timer
    this->CheckStateTimer = this->create_wall_timer(std::chrono::milliseconds(50),std::bind(&RobotNode::CheckState,this));
    this->StateChangeTimer = this->create_wall_timer(std::chrono::seconds(3), std::bind(&RobotNode::StateChange,this));
    this->TfTimer = this->create_wall_timer(std::chrono::milliseconds(20),std::bind(&RobotNode::TfBroadcast,this));
    this->CartoCoorTimer = this->create_wall_timer(std::chrono::milliseconds(40),std::bind(&RobotNode::GetCoorInfo,this));
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

void RobotNode::GetCoorInfo()
{
    geometry_msgs::msg::TransformStamped BaseFootPrintToWorld;
    try {
        BaseFootPrintToWorld = TfBuffer->lookupTransform("world", "robot_" + robot.id + "_base_footprint", tf2::TimePointZero);

        this->robot.coor.x = BaseFootPrintToWorld.transform.translation.x;
        this->robot.coor.y = BaseFootPrintToWorld.transform.translation.y;

        this->robot.quat.x = BaseFootPrintToWorld.transform.rotation.x;
        this->robot.quat.y = BaseFootPrintToWorld.transform.rotation.y;
        this->robot.quat.z = BaseFootPrintToWorld.transform.rotation.z;
        this->robot.quat.w = BaseFootPrintToWorld.transform.rotation.w;
    }
    catch(const tf2::TransformException &error) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "TF Exception: %s", error.what());
        return;
    }
}

void RobotNode::SubOdomInfo(const nav_msgs::msg::Odometry::SharedPtr info)
{
    if (info) return;
    // this->robot.coor.x = info->pose.pose.position.x;
    // this->robot.coor.y = info->pose.pose.position.y;
    
    // this->robot.quat.x = info->pose.pose.orientation.x;
    // this->robot.quat.y = info->pose.pose.orientation.y;
    // this->robot.quat.z = info->pose.pose.orientation.z;
    // this->robot.quat.w = info->pose.pose.orientation.w;
}       

void RobotNode::CheckCollision(const sensor_msgs::msg::LaserScan::SharedPtr info)
{
    // tf laser->basefootprint
    EulerDegree LaserDeg;
    try {
        geometry_msgs::msg::TransformStamped LaserToBaseFootprint;
        LaserToBaseFootprint = TfBuffer->lookupTransform("robot_" + this->robot.id + "_base_footprint", "robot_" + this->robot.id + "_laser_link", tf2::TimePointZero);
        auto rotation = LaserToBaseFootprint.transform.rotation;
        Quaternion quaternion = {rotation.x, rotation.y, rotation.z, rotation.w};
        GetRPY(quaternion, &LaserDeg);
    }
    catch(const tf2::TransformException &error) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "TF Exception: %s", error.what());
        return;
    }

    // calculate valid range
    double RealAnlgeMin = info->angle_min - LaserDeg.yaw;

    int IndexMin = std::ceil((-std::asin(ROBOT_WIDTH / (info->range_min + 2 * COLLISION_RANGE)) - RealAnlgeMin) / info->angle_increment);
    int IndexMax = std::ceil(( std::asin(ROBOT_WIDTH / (info->range_min + 2 * COLLISION_RANGE)) - RealAnlgeMin) / info->angle_increment);
    // calculate is checked
    int CollisionWarnLevel = 0;     // 0 safe; 1 warn; 2 error
    for (int i = IndexMin; i < IndexMax; i++) {
        if (info->ranges[i] > info->range_min && info->ranges[i] < info->range_min + 2 * COLLISION_RANGE) {
            CollisionWarnLevel = 1;
        }
        if (info->ranges[i] > info->range_min && info->ranges[i] < info->range_min + COLLISION_RANGE) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "IMMEDIATELY COLLIDE, DISTANCE %f.", info->ranges[i]);
            CollisionWarnLevel = 2;
            break;
        }
    }

    // calculate cmd_vel
    if (CollisionWarnLevel == 2) {
        this->path.second.clear();
        this->robot.state = ASTERN;

        geometry_msgs::msg::Twist info;
        info.angular.z = 0;
        info.linear.y = 0;
        info.linear.x = -0.2;
        this->PublisherMotionControl->publish(info);
    }
    else {
        if (CollisionWarnLevel == 0 && this->robot.state == ASTERN) {
            this->robot.state = RELAX;
        }
    }
}

void RobotNode::PubMotionControl()
{
    CmlSpeed speed;
    CalSpeed(CoorTrans(this->robot.coor,this->robot.tf),this->path.second[0],this->robot.quat,&speed);
    geometry_msgs::msg::Twist info;
    info.angular.z = speed.angular;
    info.linear.x = speed.linear[0];
    info.linear.y = speed.linear[1];

    this->PublisherMotionControl->publish(info);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "robot_" + this->robot.id;
    marker.id = std::stoi(this->robot.id) + 2;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0;

    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    marker.points.clear();
    for (const auto& point : this->path.second) {
        geometry_msgs::msg::Point AddedPoint;
        AddedPoint.x = point.x;
        AddedPoint.y = point.y;
        AddedPoint.z = 0;
        marker.points.push_back(AddedPoint);
    }

    this->PublisherPathPoints->publish(marker);
}

void RobotNode::UpdatePathInfo()
{
    if (CalDistance(CoorTrans(this->robot.coor,this->robot.tf), this->path.second[0]) < 5 * PRECISION_2) {
        this->path.second.erase(this->path.second.begin());
        if (this->path.second.empty()) {
            this->robot.state = WAIT;
            this->StateChangeTimer->reset();
        }
    }
}

void RobotNode::PubStopControl()
{
    geometry_msgs::msg::Twist info; 
    info.angular.z = 0;
    info.linear.x = 0;
    info.linear.y = 0;
    this->PublisherMotionControl->publish(info);
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

    if (this->robot.state == WAIT) {
        this->PubStopControl();
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
        if (this->robot.state != WORK) {
            this->path.second.clear();
            for (size_t i = 0; i < result->path.size(); i += 2) {
                coordinate.x = result->path[i];
                coordinate.y = result->path[i+1];
                this->path.second.push_back(coordinate);
                RCLCPP_INFO(this->get_logger(), "PATH POINT. %f, %f", coordinate.x, coordinate.y);
            }
            this->path.first = false;
            this->robot.state = WORK;
            this->StateChangeTimer->cancel();
        }
    }
    else {
        geometry_msgs::msg::Twist info; 
        info.angular.z = 0;
        info.linear.x = 0;
        info.linear.y = 0;
        this->PublisherMotionControl->publish(info);

        this->robot.state = RELAX;
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
    geometry_msgs::msg::TransformStamped OdomToSubmap;
    try {
        OdomToSubmap = TfBuffer->lookupTransform("robot_" + robot.id + "_submap", "robot_" + robot.id + "_odom", tf2::TimePointZero);
    }
    catch(const tf2::TransformException &error) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "TF Exception: %s", error.what());
        return;
    }

    geometry_msgs::msg::TransformStamped WorldToSubmap;
    WorldToSubmap.header.frame_id = "world";
    WorldToSubmap.child_frame_id = "robot_" + this->robot.id + "_submap";
    WorldToSubmap.header.stamp = this->get_clock()->now();

    WorldToSubmap.transform.translation.x = -this->robot.tf.x - OdomToSubmap.transform.translation.x;
    WorldToSubmap.transform.translation.y = -this->robot.tf.y - OdomToSubmap.transform.translation.y;
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
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "TF Exception: %s", error.what());
    }
}