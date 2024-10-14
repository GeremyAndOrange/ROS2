#ifndef CONSOLE_GUI
#define CONSOLE_GUI

#include "ui_console_gui.h"

class ConsoleGui : public QDialog
{
    Q_OBJECT
public:
    ConsoleGui(QWidget* parent=0);
    ~ConsoleGui();

private:

private:
    Ui::Dialog ui;
};

#endif