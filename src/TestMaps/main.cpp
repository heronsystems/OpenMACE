#include <QCoreApplication>
#include <iostream>
#include <QFile>
#include <QTextStream>
#include <QStringList>

#include "base/pose/cartesian_position_2D.h"
#include "base/pose/cartesian_position_3D.h"

#include "base/pose/geodetic_position_2D.h"
#include "base/pose/geodetic_position_3D.h"

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
    CartesianPosition_2D point2(1,1);
    CartesianPosition_2D newPoint2D = point1 + point2;
    CartesianPosition_3D newPoint3D = point1 + point2;

    CartesianPosition_3D testPoint3D(0,1,2);
    CartesianPosition_3D copyTest(testPoint3D);

    GeodeticPosition_2D geoPoint1(1,2);
    GeodeticPosition_2D geoPoint2(2,3);
    GeodeticPosition_3D geoPoint3 = geoPoint1 + geoPoint2;

    std::cout<<"Pausing here"<<std::endl;

    return 0;
}
