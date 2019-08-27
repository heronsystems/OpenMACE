#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "base/state_space/space_information.h"
#include "base/state_space/discrete_motion_validity_check.h"
#include "base/state_space/special_validity_check.h"
#include "base/state_space/cartesian_2D_space.h"

#include <QCoreApplication>
#include <iostream>
#include <QFile>
#include <QTextStream>
#include <QStringList>

#include "maps/data_2d_grid.h"
#include "maps/occupancy_definition.h"

#include "base/pose/pose_components.h"
#include "base/unit_tests/unittests_orientation.h"
#include "base/pose/dynamics_aid.h"

#include "planners/virtual_potential_fields/potential_fields.h"

const char XValue =
        #ifdef _WIN32
        '\\';
#else
        '/';
#endif

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


    mace::pose::GeodeticPosition_3D swarmOrigin3(35.6208548, -78.8033786, 15.0);
    mace::pose::GeodeticPosition_3D swarmOrigin2(35.6209758, -78.8031681, 25.0);

    mace::pose::CartesianPosition_3D localPosition;
    mace::pose::DynamicsAid::GlobalPositionToLocal(&swarmOrigin3, &swarmOrigin2, &localPosition);

    mace::maps::OccupiedResult fillData = mace::maps::OccupiedResult::NOT_OCCUPIED;

    mace::maps::Data2DGrid<mace::maps::OccupiedResult>* exampleMap = new mace::maps::Data2DGrid<mace::maps::OccupiedResult>(&fillData);
    exampleMap->updateGridSize(-43,43,-13,13,1,1);
//    mace::pose::CartesianPosition_2D originPosition(-15,0);
//    exampleMap->updateOriginPosition(originPosition);

    mace::maps::OccupiedResult* value;

    //construct the two horizontal portions
    unsigned int xIndex = 0, yIndex = 0;
    for(xIndex = 1; xIndex < exampleMap->getSizeX() - 1; xIndex++)
    {
        value = exampleMap->getCellByPosIndex(xIndex,yIndex);
        *value = mace::maps::OccupiedResult::ENVIRONMENT_BOUNDARY;
    }
    yIndex = exampleMap->getSizeY() - 1;
    for(xIndex = 1; xIndex < exampleMap->getSizeX() - 1; xIndex++)
    {
        value = exampleMap->getCellByPosIndex(xIndex,yIndex);
        *value = mace::maps::OccupiedResult::ENVIRONMENT_BOUNDARY;
    }

//    //construct the two vertical portions
    xIndex = 0;
    for(yIndex = 0; yIndex < exampleMap->getSizeY(); yIndex++)
    {
        value = exampleMap->getCellByPosIndex(xIndex,yIndex);
        *value = mace::maps::OccupiedResult::ENVIRONMENT_BOUNDARY;
    }
    xIndex = exampleMap->getSizeX() - 1;
    for(yIndex = 0; yIndex < exampleMap->getSizeY(); yIndex++)
    {
        value = exampleMap->getCellByPosIndex(xIndex,yIndex);
        *value = mace::maps::OccupiedResult::ENVIRONMENT_BOUNDARY;
    }

    //establish boundary around pillar one
    value = exampleMap->getCellByPos(15,0);
    *value = mace::maps::OccupiedResult::OCCUPIED;

    //establish boundary around pillar two
    value = exampleMap->getCellByPos(-15,0);
    *value = mace::maps::OccupiedResult::OCCUPIED;


    mace::state_space::SpaceInformationPtr spaceInfo;

    mace::PotentialFields pf(spaceInfo,exampleMap);

    pf.printGrid();

    mace::pose::CartesianPosition_2D currentPosition(-30,10);
    mace::pose::CartesianPosition_2D targetPosition(30,10);

    VPF_ResultingForce resultingForce = pf.computeArtificialForceVector(&currentPosition, &targetPosition);
    double heading = wrapTo2Pi(atan2(resultingForce.getForceY(), resultingForce.getForceX()));
    double speedY = sin(heading) * 2.0;
    double speedX = cos(heading) * 2.0;
    unsigned int index = exampleMap->indexFromPos(-43,13);

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
