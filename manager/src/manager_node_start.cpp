#include "manager/manager.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // 产生一个ManagerNode的节点
    auto node = std::make_shared<ManagerNode>("ManagerNode");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}