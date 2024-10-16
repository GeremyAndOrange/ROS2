#ifndef CONSOLE_GUI
#define CONSOLE_GUI

// QT
#include <QVector>
#include <QThread>
#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include "ui_console_gui.h"
// ConsoleNode
#include "work_station/console_interfaces.hpp"
#include "rclcpp/rclcpp.hpp"
// ManagerNode
#include "manager/manager.hpp"

class ConsoleGui : public QDialog
{
    Q_OBJECT
public:
    ConsoleGui(QWidget* parent=0);
    ~ConsoleGui();

private:
    void ConnectSlots();
    void InitialParameter();

private slots:
    void OnSystemStartClicked();

private:
    Ui::Console ui;

    // Parameter
    QGraphicsScene scene;
    QVector<QVector<int>> GridMap;

    // NodeManager
    rclcpp::executors::MultiThreadedExecutor executor;
    QMap<std::string, std::shared_ptr<rclcpp::Node>> NodeList;
};

/*--------------------------*/

class GridItem : public QGraphicsRectItem
{
public:
    GridItem(int x, int y, int type);
    ~GridItem();
};

#endif