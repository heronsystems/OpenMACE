#include <QCoreApplication>
#include <iostream>
#include <QFile>
#include <QTextStream>
#include <QStringList>

#include "base/pose/cartesian_position_2D.h"
#include "base/pose/cartesian_position_3D.h"

#include "base/pose/geodetic_position_2D.h"
#include "base/pose/geodetic_position_3D.h"
#include "base/misc/data_test.h"

using namespace mace ;

const char kPathSeperator =
        #ifdef _WIN32
        '\\';
#else
        '/';
#endif

using namespace mace::pose;
int main(int argc, char *argv[])
{

    std::cout << "HI" << std::endl;

    CartesianPosition_2D point1(2,3);
    point1.data.normalize();

    CartesianPosition_2D point2(1,1);
    CartesianPosition_2D newPoint2D = point1 + point2;
    CartesianPosition_3D newPoint3D = point1 + point2;

    CartesianPosition_3D testPoint3D(0,1,2);
    CartesianPosition_3D copyTest(testPoint3D);
    CartesianPosition_3D addPoints = testPoint3D + copyTest;

    std::cout<<"Pausing here"<<std::endl;

    return 0;
}
