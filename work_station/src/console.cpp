#include "work_station/console.hpp"
#include "console_gui/console_gui.h"

// 构造函数,有一个参数为节点名称
ConsoleNode::ConsoleNode(std::string name) : Node(name)
{
    Initial();
}

ConsoleNode::~ConsoleNode()
{

}

void ConsoleNode::Initial()
{
    ConsoleGui dlg;
    dlg.exec();
}