#include "robot_control/control.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // 产生一个RobotNode的节点
    auto node = std::make_shared<RobotNode>("RobotNode");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}