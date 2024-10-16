#ifndef CONSOLE_GUI
#define CONSOLE_GUI

// QT
#include <QVector>
#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include "ui_console_gui.h"
// ConsoleNode
#include "work_station/console_interfaces.hpp"
#include "rclcpp/rclcpp.hpp"

class ConsoleGui : public QDialog
{
    Q_OBJECT
public:
    ConsoleGui(QWidget* parent=0);
    ~ConsoleGui();

private slots:
    void OnSystemStartClicked();    

private:
    Ui::Console ui;

    // Parameter
    QGraphicsScene scene;
    QVector<QVector<int>> GridMap;
};

/*--------------------------*/

class GridItem : public QGraphicsRectItem
{
public:
    GridItem(int x, int y, int type);
    ~GridItem();
};

#endif