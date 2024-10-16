#include "console_gui/console_gui.hpp"

ConsoleGui::ConsoleGui(QWidget* parent)
    : QDialog(parent)
{
    ui.setupUi(this);
    ui.GridMapGraphic->setScene(&scene);

    connect(ui.StartSystem, SIGNAL(clicked()), this, SLOT(OnSystemStartClicked()));
}

ConsoleGui::~ConsoleGui()
{

}

void ConsoleGui::OnSystemStartClicked()
{
    // start manager/gazebo/global_map
    std::string manager_command = "ros2 launch robot_description environment.launch.py &";
    std::system(manager_command.c_str());
    
    int RobotNumber = ui.RobotNumber->toPlainText().toInt();
    for (int i = 0; i < RobotNumber; i++) {
        // update urdf file
        std::string urdf_command = "ros2 launch robot_description config.launch.py robot_name:=robot_" + std::to_string(i);
        std::system(urdf_command.c_str());
        // create new robot
        std::string robot_command = " ros2 launch robot_description robot.launch.py robot_name:=robot_" + std::to_string(i)
                                  + " origin_x:=" + std::to_string(0)
                                  + " origin_y:=" + std::to_string(i)
                                  + " &";
        std::system(robot_command.c_str());
    }

    ui.InfoText->setText("System Starting Finished\n");
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
