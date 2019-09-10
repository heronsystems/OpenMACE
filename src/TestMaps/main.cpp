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

#include "planners/fast_marching/ndgridmap/fmcell.h"
#include "planners/fast_marching/ndgridmap/ndgridmap.hpp"
#include "planners/signed_distance_fields/collision_map.hpp"


#include "planners/fast_marching/ndgridmap/fmcell.h"
#include "planners/fast_marching/ndgridmap/ndgridmap.hpp"
#include "planners/fast_marching/console/console.h"

#include "planners/fast_marching/fm2/fm2.hpp"
#include "planners/fast_marching/fm2/fm2star.hpp"
#include "planners/fast_marching/datastructures/fmfibheap.hpp"
#include "planners/fast_marching/datastructures/fmpriorityqueue.hpp"

#include "planners/fast_marching/io/maploader.hpp"
#include "planners/fast_marching/io/gridplotter.hpp"
#include "planners/fast_marching/io/gridwriter.hpp"

const char kPathSeperator =
        #ifdef _WIN32
        '\\';
#else
        '/';
#endif
using namespace std;
using namespace Eigen;

typedef nDGridMap<FMCell, 3> FMGrid3D;
typedef array<unsigned int, 3> Coord3D;

sdf_tools::COLLISION_CELL _free_cell(0.0);
sdf_tools::COLLISION_CELL _obst_cell(1.0);
sdf_tools::COLLISION_CELL _oob_cell(INFINITY);

double velMapping(double d, double max_v)
{
    double vel;

    if( d <= 0.25)
        vel = 2.0 * d * d;
    else if(d > 0.25 && d <= 0.75)
        vel = 1.5 * d - 0.25;
    else if(d > 0.75 && d <= 1.0)
        vel = - 2.0 * (d - 1.0) * (d - 1.0) + 1;
    else
        vel = 1.0;

    return vel * max_v;
}

int main(int argc, char *argv[])
{

    /*
    unsigned int idx;
    double max_vel = 2;
    vector<unsigned int> obs;
    Eigen::Vector3d pt;
    vector<int64_t> pt_idx;
    double flow_vel;
    double _resolution = 0.5;
    Eigen::Vector3d _map_origin;
    _map_origin<<0.0,0.0,0.0;

    mace::maps::OccupiedResult fillData = mace::maps::OccupiedResult::NOT_OCCUPIED;
    mace::maps::Data2DGrid<mace::maps::OccupiedResult> staticMap(&fillData, -10,10,-10,10,0.5,0.5);
    mace::maps::OccupiedResult* value;

    //establish boundary around pillar one
    value = staticMap.getCellByPos(15,0);
    *value = mace::maps::OccupiedResult::OCCUPIED;

    //establish boundary around pillar two
    value = staticMap.getCellByPos(-15,0);
    *value = mace::maps::OccupiedResult::OCCUPIED;

    //establish boundary around pillar two
    value = staticMap.getCellByPos(-10,7);
    *value = mace::maps::OccupiedResult::OCCUPIED;

    //establish boundary around pillar two
    value = staticMap.getCellByPos(10,7);
    *value = mace::maps::OccupiedResult::OCCUPIED;

    //establish boundary around pillar two
    value = staticMap.getCellByPos(-2,9);
    *value = mace::maps::OccupiedResult::OCCUPIED;

    //establish boundary around pillar two
    value = staticMap.getCellByPos(4,7);
    *value = mace::maps::OccupiedResult::OCCUPIED;

    unsigned int size_x = 4;
    unsigned int size_y = 4;
    unsigned int size_z = 4;

    Coord3D dimsize {size_x, size_y, size_z};
    FMGrid3D grid_fmm(dimsize,0.5);

    sdf_tools::CollisionMapGrid* collision_map_local = new sdf_tools::CollisionMapGrid(_resolution, size_x, size_y, size_z, _free_cell,INFINITY);
    Vector3d addPt(0.5, 0.5, 0.5);
    collision_map_local->SetValue3d(addPt, _obst_cell);
    sdf_generation::DistanceField EDT = collision_map_local->ExtractDistanceField(INFINITE);

    for(unsigned int k = 0; k < size_z; k++)
    {
        for(unsigned int j = 0; j < size_y; j++)
        {
            for(unsigned int i = 0; i < size_x; i++)
            {
                idx = k * size_y * size_x + j * size_x + i;
                pt << (i + 0.5) * _resolution + _map_origin(0),
                        (j + 0.5) * _resolution + _map_origin(1),
                        (k + 0.5) * _resolution + _map_origin(2);

                VoxelGrid::GRID_INDEX index = collision_map_local->LocationToGridIndex((i + 0.5) * _resolution + _map_origin(0),
                                                                          (j + 0.5) * _resolution + _map_origin(1),
                                                                          (k + 0.5) * _resolution + _map_origin(2));

                if(collision_map_local->IndexInBounds(index))
                {
                    double d = sqrt(EDT.GetImmutable(index).first.distance_square) * _resolution;
                    flow_vel = velMapping(d, max_vel);
                }
                else
                    flow_vel = max_vel;

                if( k == 0 || k == (size_z - 1) || j == 0 || j == (size_y - 1) || i == 0 || i == (size_x - 1) )
                    flow_vel = 0.0;


                grid_fmm[idx].setOccupancy(flow_vel);
                if (grid_fmm[idx].isOccupied())
                    obs.push_back(idx);
            }
        }
    }

    grid_fmm.setOccupiedCells(std::move(obs));
    grid_fmm.setLeafSize(_resolution);
    */

    //    Eigen::Vector2d repulsionVector(1, 2);
    //    Eigen::Vector2d rotatedRepulsionVector (1, 2);
    //    double angle = acos(rotatedRepulsionVector.dot(repulsionVector) / (rotatedRepulsionVector.norm() * repulsionVector.norm()));

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



    console::info("Parsing input arguments.");
    string filename = "/home/majorpr13/OpenMACE/map.png";

    // A bit of shorthand.
    constexpr unsigned int ndims2 = 2; // Setting two dimensions.
    typedef nDGridMap<FMCell, ndims2> FMGrid2D;
    typedef typename std::vector< std::array<double, ndims2> > Path2D; // A bit of short-hand.

    // Loading grid.
    FMGrid2D grid_fm2;

    // Solvers declaration.
    std::vector<Solver<FMGrid2D>*> solvers;
    solvers.push_back(new FM2<FMGrid2D>("FM2_Dary"));
    solvers.push_back(new FM2<FMGrid2D, FMFibHeap<FMCell> >("FM2_Fib"));
    solvers.push_back(new FM2<FMGrid2D, FMPriorityQueue<FMCell> >("FM2_SFMM"));
    solvers.push_back(new FM2Star<FMGrid2D>("FM2*_Dary_Dist", DISTANCE));
    solvers.push_back(new FM2Star<FMGrid2D>("FM2*_Dary_Time"));
    solvers.push_back(new FM2Star<FMGrid2D, FMFibHeap<FMCell> >("FM2*_Fib_Time"));
    solvers.push_back(new FM2Star<FMGrid2D, FMFibHeap<FMCell> >("FM2*_Fib_Dist", DISTANCE));
    solvers.push_back(new FM2Star<FMGrid2D, FMPriorityQueue<FMCell> >("FM2*_SFMM_Time"));
    solvers.push_back(new FM2Star<FMGrid2D, FMPriorityQueue<FMCell> >("FM2*_SFMM_Dist", DISTANCE));

    // Executing every solver individually over the same grid.
    for (Solver<FMGrid2D>* s :solvers)
    {
        // For FM2 and its variations, it is better to completely reinitialize the grid.
        //if(!MapLoader::loadMapFromText(filename.c_str(), grid_fm2)) // Loading from text file.
            //exit(1);
        MapLoader::loadMapFromImg(filename.c_str(), grid_fm2); // Loading from image.
        s->setEnvironment(&grid_fm2);
        s->setInitialAndGoalPoints({30, 20}, {375, 280}); // Init and goal points directly set.
        s->compute();
        cout << "\tElapsed "<< s->getName() <<" time: " << s->getTime() << " ms" << '\n';

        Path2D path;
        vector<double> path_vels;
        s->as<FM2<FMGrid2D>>()->computePath(&path, &path_vels);
        GridPlotter::plotArrivalTimesPath(grid_fm2, path);
    }

    // Preventing memory leaks.
    for (auto & s : solvers)
        delete s;


    std::cout<<"Let us pause here"<<std::endl;

    return 0;
}
