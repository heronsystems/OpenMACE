#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <QCoreApplication>
#include <iostream>
#include <QFile>
#include <QTextStream>
#include <QStringList>

#include "maps/data_2d_grid.h"
#include "base/pose/pose_components.h"
#include "base/unit_tests/unittests_orientation.h"
#include "base/pose/dynamics_aid.h"

const char kPathSeperator =
        #ifdef _WIN32
        '\\';
#else
        '/';
#endif
using namespace std;
using namespace Eigen;

int main(int argc, char *argv[])
{
    double value = 0;


        mace::pose::GeodeticPosition_3D swarmOrigin3(35.6208548, -78.8033786, 15.0);
        mace::pose::GeodeticPosition_2D swarmOrigin2(35.6208548, -78.8033786);

        mace::pose::CartesianPosition_3D vehicleOrigin(10, 10, 10.0);
        mace::pose::GeodeticPosition_3D target;
        mace::pose::DynamicsAid::LocalPositionToGlobal(&swarmOrigin2, &vehicleOrigin, &target);

//    Eigen::Vector2d currentPosition(1,1);
//    Vector3d translation(1,1,2);
//    Matrix3d rotation = Matrix3d::Identity();

//    Transform<double, 3, Affine> t =
//            Transform<double, 3, Affine>::Identity();
//    t.translate(translation);

//    Transform<double, 3, Affine> tInvert;
//    tInvert.linear() = rotation.transpose();
//    tInvert.translation() = translation * -1;

//    currentPosition = t.rotation()*currentPosition + t.translation();
//    currentPosition = tInvert.rotation()*currentPosition + tInvert.translation();


//    mace::pose::Rotation_3D orientationTest(0,M_PI_2,0.2);
//    Eigen::Matrix3d rotationMatrix = orientationTest.m_QRotation.toRotationMatrix();
//    double rxRoll, rxPitch, rxYaw;
//    orientationTest.getDiscreteEuler(rxRoll, rxPitch, rxYaw);

//    double txRoll, txPitch, txYaw;
//    txYaw = atan2(-rotationMatrix(1,0),rotationMatrix(0,0));
//    txPitch = asin(rotationMatrix(2,0));
//    txRoll = atan2(-rotationMatrix(2,1),rotationMatrix(2,2));

//    double qw, qx, qy, qz;
//    qw = orientationTest.m_QRotation.w();
//    qx = orientationTest.m_QRotation.x();
//    qy = orientationTest.m_QRotation.y();
//    qz = orientationTest.m_QRotation.z();

    std::cout<<"Let us pause here"<<std::endl;

    return 0;
}
