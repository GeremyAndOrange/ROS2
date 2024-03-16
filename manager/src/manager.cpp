#include "manager/manager.hpp"

// 构造函数,有一个参数为节点名称
ManagerNode::ManagerNode(std::string name) : Node(name)
{
    RCLCPP_INFO(this->get_logger(), "This is manager node.");
    // property
    this->Initial();

    // service
    this->NewRobotService = this->create_service<interfaces::srv::NewRobot>("/new_robot_service",std::bind(&ManagerNode::NewRobot,this,_1,_2));
    this->SendTaskService = this->create_service<interfaces::srv::GetTask>("/get_task",std::bind(&ManagerNode::SendTask,this,_1,_2));

    // topic
    

    // timer

}

void ManagerNode::Initial()
{
    for (size_t i = 0; i < ROBOT_NUMBER; i++) {
        this->group[i].existence = no;
        this->group[i].postion.x = 0.0;
        this->group[i].postion.y = 0.0;
    }
    
}

void ManagerNode::NewRobot(const interfaces::srv::NewRobot::Request::SharedPtr request, const interfaces::srv::NewRobot::Response::SharedPtr response)
{
    if (request->is_create && this->group[ROBOT_NUMBER].existence != yes) {
        // new robot id
        for (int i = 0; i < ROBOT_NUMBER; i++) {
            if (this->group[i].existence == no) {
                this->group[i].existence = yes;
                response->id = i;
                break;
            }
        }
        // update urdf file
        std::string urdf_command = "ros2 launch robot_description config.launch.py robot_name:=robot_" + std::to_string(response->id);
        std::system(urdf_command.c_str());
        // create new robot
        std::string robot_command = " ros2 launch robot_description robot.launch.py robot_name:=robot_" + std::to_string(response->id)
                                  + " origin_x:=" + std::to_string(request->gen_x)
                                  + " origin_y:=" + std::to_string(request->gen_y)
                                  + " &";
        std::system(robot_command.c_str());
    }
}

void ManagerNode::SendTask(const interfaces::srv::GetTask::Request::SharedPtr request, const interfaces::srv::GetTask::Response::SharedPtr response)
{
    this->group[request->id].postion.x = request->x;
    this->group[request->id].postion.y = request->y;
    std::vector<double> target_path = {request->x + 4.0, request->y, request->x, request->y};
    // send task path
    response->path = target_path;
}