#include <Eigen/Geometry>
#include "base/pose/rotation_3D.h"
#include "base/unit_tests/unittests_orientation.h"

#include <QCoreApplication>
#include <iostream>
#include <QFile>
#include <QTextStream>
#include <QStringList>


const char kPathSeperator =
        #ifdef _WIN32
        '\\';
#else
        '/';
#endif
using namespace std;

int main(int argc, char *argv[])
{
    runOrientationTests();

    return 0;
}
