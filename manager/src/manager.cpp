#include "manager/manager.hpp"

// 构造函数,有一个参数为节点名称
ManagerNode::ManagerNode(std::string name) : Node(name)
{
    RCLCPP_INFO(this->get_logger(), "This is manager node.");
    // parameter
    for (int i = 0; i < ROBOT_NUMBER; i++) this->id[i] = 0;
    this->number = 0;

    // service
    this->new_robot_service = this->create_service<interfaces::srv::NewRobot>("new_robot_service",std::bind(&ManagerNode::NewRobot,this,_1,_2));
    this->send_task_service = this->create_service<interfaces::srv::GetTask>("get_task",std::bind(&ManagerNode::SendTask,this,_1,_2));

    // topic
    this->SubscriptionLidarInfo = this->create_subscription<nav_msgs::msg::OccupancyGrid>("local_map",10,std::bind(&ManagerNode::SubLocalMap,this,_1));

    // timer

    // debug
    this->debug = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map",10);
    this->debugTimer = this->create_wall_timer(std::chrono::milliseconds(500),std::bind(&ManagerNode::pub,this));
}

void ManagerNode::NewRobot(const interfaces::srv::NewRobot::Request::SharedPtr request, const interfaces::srv::NewRobot::Response::SharedPtr response)
{
    if (request->is_create && this->number < ROBOT_NUMBER)
    {
        // robot number
        this->number += 1;

        // new robot's id
        for (int i = 0; i < ROBOT_NUMBER; i++)
        {
            if (this->id[i] == 0)
            {
                this->id[i] = 1;
                response->id = i;
                break;
            }
        }
        
        // create new robot
        std::string robot_name = "robot_" + std::to_string(response->id);
        std::string command = std::string(ROBOT_CONTROL) + " --remap __node:=" + robot_name + " --param id:=\"'" + std::to_string(response->id) +"'\" &";
        int result = std::system(command.c_str());
        RCLCPP_INFO(this->get_logger(), std::to_string(result).c_str());

        // create urdf
        std::string dst_file = TEMURDF + robot_name + ".urdf";
        std::ofstream urdf_file(dst_file);
        ModifyURDF(SRCURDF,dst_file,robot_name);

        // load to gazebo
        std::string spawn_command = SPAWN_ENTITY + robot_name + " -file " + dst_file + " -x " + std::to_string(request->gen_x) + " -y " + std::to_string(request->gen_y) + " &";
        result = std::system(spawn_command.c_str());
        RCLCPP_INFO(this->get_logger(), std::to_string(result).c_str());
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Failed to create robot");
    }
}

void ManagerNode::SendTask(const interfaces::srv::GetTask::Request::SharedPtr request, const interfaces::srv::GetTask::Response::SharedPtr response)
{
    std::vector<double> target_path = {4.0,0.0,0.0,0.0};
    // save robot position
    RobotPosition robot_position;
    robot_position.id = request->id;
    robot_position.position.x = request->x;
    robot_position.position.y = request->y;
    robot_position.position.z = 0;
    this->RobotPositionList.push_back(robot_position);
    // send task path
    if (request->state == 2)
    {
        response->path = target_path;
    }
}

void ManagerNode::SubLocalMap(const nav_msgs::msg::OccupancyGrid info)
{
    if (this->map.map.empty())
    {
        this->map.origin.x = info.info.origin.position.x;
        this->map.origin.y = info.info.origin.position.y;
        this->map.map.resize(MAP_LENGTH, std::deque<int>(MAP_LENGTH, -1));
        UpdateMap(info, 0, 0);
    }
    else
    {
        int delta_x = int((info.info.origin.position.x - this->map.origin.x) / MAP_RESOLUTION);
        int delta_y = int((info.info.origin.position.y - this->map.origin.y) / MAP_RESOLUTION);
        if (delta_y <= 0)
        {
            this->map.map.insert(this->map.map.begin(), -delta_y, std::deque<int>(this->map.map[0].size(), -1));
            this->map.origin.x = info.info.origin.position.x;
            delta_y = 0;
        }
        else
        {
            if (delta_y + MAP_LENGTH > int(this->map.map.size()))
            {
                this->map.map.insert(this->map.map.end(), delta_y + MAP_LENGTH - int(this->map.map.size()), std::deque<int>(this->map.map[0].size(), -1));
            }
        }
        if (delta_x <= 0)
        {
            for (auto& row : this->map.map)
            {
                row.insert(row.begin(), -delta_x, -1);
            }
            this->map.origin.y = info.info.origin.position.y;
            delta_x = 0;
        }
        else
        {
            if (delta_x + MAP_LENGTH > int(this->map.map[0].size()))
            {
                for (auto& row : this->map.map)
                {
                    row.insert(row.end(), delta_x + MAP_LENGTH - int(this->map.map[0].size()), -1);
                }
            }
        }
        UpdateMap(info, delta_x, delta_y);
    }
}

void ManagerNode::UpdateMap(const nav_msgs::msg::OccupancyGrid info, int start_x, int start_y)
{
    for (int i = 0; i < MAP_LENGTH; ++i)
    {
        for (int j = 0; j < MAP_LENGTH; ++j)
        {
            this->map.map[start_y + i][start_x + j] = info.data[i * MAP_LENGTH + j];
        }
    }
}

void ManagerNode::pub()
{
    nav_msgs::msg::OccupancyGrid info;
    std::vector<int8_t> flat_map;
    for (const auto& row : this->map.map) {
        for (int val : row) {
            flat_map.push_back(static_cast<int8_t>(val));
        }
    }
    info.info.height = this->map.map[0].size();
    info.info.width = this->map.map.size();
    info.info.resolution = MAP_RESOLUTION;
    info.info.origin.position.x = this->map.origin.x;
    info.info.origin.position.y = this->map.origin.y;
    info.data = flat_map;
    info.header.frame_id = "map";
    info.header.stamp = rclcpp::Node::now();
    this->debug->publish(info);
}