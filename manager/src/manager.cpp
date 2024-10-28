#include "manager/manager.hpp"

// 构造函数,有一个参数为节点名称
ManagerNode::ManagerNode(std::string name) : Node(name)
{
    RCLCPP_INFO(this->get_logger(), "This is manager node.");
    this->Initial();
}

ManagerNode::~ManagerNode()
{
    
}

void ManagerNode::Initial()
{
    // service
    this->SendTaskService = this->create_service<interfaces::srv::GetTask>("/get_task",std::bind(&ManagerNode::SendTask,this,_1,_2));

    // topic
    this->SubscriptionGLobalMap = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/globalmap",10,std::bind(&ManagerNode::SubMap,this,_1));
    this->PublisherExpansionedMap = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/expansion_map",10);

    // timer
    
}

void ManagerNode::SendTask(const interfaces::srv::GetTask::Request::SharedPtr request, const interfaces::srv::GetTask::Response::SharedPtr response)
{
    // save robot position
    RobotInfo robot;
    robot.id = request->id;
    robot.tf.x = request->tf_x;
    robot.tf.y = request->tf_y;
    robot.coor.x = request->coor_x;
    robot.coor.y = request->coor_y;
    RobotGroup[robot.id] = robot;

    // calculate expansion map
    ExpansionMap();

    // calculate task points
    AggregateTask(robot.id);

    // calculate nearest task point

    // calculate path planning

    // send path


    std::vector<double> target_path = {robot.coor.x + robot.tf.x + 4.0, robot.coor.y + robot.tf.y, robot.coor.x + robot.tf.x, robot.coor.y + robot.tf.y};
    // send task path
    response->is_path = true;
    response->path = target_path;
}

void ManagerNode::SubMap(const nav_msgs::msg::OccupancyGrid::SharedPtr info)
{
    std::lock_guard<std::mutex> lock(this->MapMutex);
    this->StoredMap = *info;
}

void ManagerNode::ExpansionMap()
{
    std::lock_guard<std::mutex> lock(this->MapMutex);
    this->ExpansionedMap = this->StoredMap;

    uint32_t width = this->StoredMap.info.width;
    uint32_t height = this->StoredMap.info.height;
    double_t resolution = this->StoredMap.info.resolution;

    int ExpansionRadius = int(std::round(EXPANSION_RADIUS / resolution));
    for (size_t y = 0; y < height; y++) {
        for (size_t x = 0; x < width; x++) {
            if (this->StoredMap.data[y * width + x] > 65) {
                for (int dy = -ExpansionRadius; dy <= ExpansionRadius; ++dy) {
                    for (int dx = -ExpansionRadius; dx <= ExpansionRadius; ++dx) {
                        if (this->StoredMap.data[(y+dy) * width + (x+dx)] < 65) {
                            if ((x+dx) > 0 && (x+dx) < width && (y+dy) > 0 && (y+dy) < height) {
                                this->ExpansionedMap.data[(y+dy) * width + (x+dx)] = 75;
                            }
                        }
                    }
                }
            }
        }
    }
    
    this->PublisherExpansionedMap->publish(this->ExpansionedMap);
}

void ManagerNode::AggregateTask(std::string RobotId)
{
    RobotInfo Self = this->RobotGroup[RobotId];
}

void ManagerNode::RRTAlgorithm()
{

}