#include <Eigen/Geometry>

#include <QCoreApplication>
#include <iostream>
#include <QFile>
#include <QTextStream>
#include <QStringList>

#include "maps/data_2d_grid.h"
#include "base/unit_tests/unittests_orientation.h"

const char kPathSeperator =
        #ifdef _WIN32
        '\\';
#else
        '/';
#endif
using namespace std;

int main(int argc, char *argv[])
{
    double value = 0;

    mace::pose::Rotation_3D orientationTest(0,M_PI_2,0.2);
    Eigen::Matrix3d rotationMatrix = orientationTest.m_QRotation.toRotationMatrix();
    double rxRoll, rxPitch, rxYaw;
    orientationTest.getDiscreteEuler(rxRoll, rxPitch, rxYaw);

    double txRoll, txPitch, txYaw;
    txYaw = atan2(-rotationMatrix(1,0),rotationMatrix(0,0));
    txPitch = asin(rotationMatrix(2,0));
    txRoll = atan2(-rotationMatrix(2,1),rotationMatrix(2,2));

    double qw, qx, qy, qz;
    qw = orientationTest.m_QRotation.w();
    qx = orientationTest.m_QRotation.x();
    qy = orientationTest.m_QRotation.y();
    qz = orientationTest.m_QRotation.z();

    std::cout<<"Let us pause here"<<std::endl;

    return 0;
}
