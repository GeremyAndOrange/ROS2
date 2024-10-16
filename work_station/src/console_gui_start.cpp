#include "console_gui/console_gui.hpp"

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    ConsoleGui dlg;
    dlg.show();
    return app.exec();
}