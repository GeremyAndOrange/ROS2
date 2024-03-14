#include "manager/manager.hpp"

// 构造函数,有一个参数为节点名称
ManagerNode::ManagerNode(std::string name) : Node(name)
{
    RCLCPP_INFO(this->get_logger(), "This is manager node.");
    // parameter
    for (int i = 0; i < ROBOT_NUMBER; i++) this->id[i] = 0;
    this->number = 0;

    // service
    this->new_robot_service = this->create_service<interfaces::srv::NewRobot>("/new_robot_service",std::bind(&ManagerNode::NewRobot,this,_1,_2));
    this->send_task_service = this->create_service<interfaces::srv::GetTask>("/get_task",std::bind(&ManagerNode::SendTask,this,_1,_2));

    // topic
    

    // timer

}

void ManagerNode::NewRobot(const interfaces::srv::NewRobot::Request::SharedPtr request, const interfaces::srv::NewRobot::Response::SharedPtr response)
{
    if (request->is_create && this->number < ROBOT_NUMBER)
    {
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
        // update urdf file
        std::string urdf_command = "ros2 launch robot_description urdf.launch.py robot_name:=robot_" + std::to_string(response->id);
        int result = std::system(urdf_command.c_str());
        RCLCPP_INFO(this->get_logger(), "Update urdf file result: %d",result);
        // create new robot
        std::string robot_command = "ros2 launch robot_description robot.launch.py robot_name:=robot_" + std::to_string(response->id)
                                  + " x:=" + std::to_string(request->gen_x)
                                  + " y:=" + std::to_string(request->gen_y)
                                  + " &";
        result = std::system(robot_command.c_str());
        RCLCPP_INFO(this->get_logger(), "Created robot result: %d",result);
    }
}

void ManagerNode::SendTask(const interfaces::srv::GetTask::Request::SharedPtr request, const interfaces::srv::GetTask::Response::SharedPtr response)
{
    std::vector<double> target_path = {4.0,0.0,0.0,0.0};
    // send task path
    if (request->state == 2)
    {
        response->path = target_path;
    }
}