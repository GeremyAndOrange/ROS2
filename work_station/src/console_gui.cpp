#include "console_gui/console_gui.h"

ConsoleGui::ConsoleGui(QWidget* parent)
    : QDialog(parent)
{
    ui.setupUi(this);
    ConnectSlots();
    InitialParameter();
}

ConsoleGui::~ConsoleGui()
{

}

void ConsoleGui::ConnectSlots()
{
    connect(ui.StartSystem, SIGNAL(clicked()), this, SLOT(OnSystemStartClicked()));
}

void ConsoleGui::InitialParameter()
{
    NodeList.clear();
    ui.GridMapGraphic->setScene(&scene);
}

void ConsoleGui::OnSystemStartClicked()
{
    auto node = std::make_shared<ManagerNode>("ManagerNode");
    NodeList.insert("ManagerNode", node);
    executor.add_node(node);

    executor.spin_some();
}

/*--------------------------*/

GridItem::GridItem(int x, int y, int type)
    : QGraphicsRectItem(x*5, y*5, 5, 5)
{
    switch (type) {
        case 0: setBrush(Qt::white); break;
        case 1: setBrush(Qt::black); break;
        case 2: setBrush(Qt::gray); break;
        default: break;
    }
}

GridItem::~GridItem()
{
    
}
