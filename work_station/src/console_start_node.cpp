#include <QApplication>
#include "work_station/console.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    // 产生一个ConsoleNode的节点
    auto node = std::make_shared<ConsoleNode>("ConsoleNode");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}