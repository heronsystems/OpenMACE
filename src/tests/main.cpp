#include <QCoreApplication>


#include <maps/tests/maps_tests.h>

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);


    runMapsTests();

    return a.exec();
}
