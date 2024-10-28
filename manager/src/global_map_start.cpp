#include "global_map/global_map.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // 产生一个GlobalMap的节点
    auto node = std::make_shared<GlobalMap>("GlobalMapNode");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}