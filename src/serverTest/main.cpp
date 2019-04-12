#include <QCoreApplication>

#include "module_ground_station.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    ModuleGroundStation *groundStation = new ModuleGroundStation();
    bool listening = groundStation->StartTCPServer();

    return a.exec();
}
