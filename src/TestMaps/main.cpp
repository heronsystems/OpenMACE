#include <QCoreApplication>
#include <iostream>
#include <QFile>
#include <QTextStream>
#include <QStringList>
#include <vector>
#include <array>

//#include "opencv2/opencv.hpp"
//#include "opencv2/imgproc.hpp"
//#include "opencv2/highgui.hpp"

//#include "graphs/signed_distance_fields/collision_map.hpp"

//#include "planners/fast_marching/ndgridmap/fmcell.h"
//#include "planners/fast_marching/ndgridmap/ndgridmap.hpp"
//#include "planners/fast_marching/fm2/fm2.hpp"
//#include "planners/graph_planning_node.h"
//#include "planners/fast_marching/fm2/fm2star.hpp"
//#include "planners/fast_marching/fm/fmm.hpp"
//#include "planners/fast_marching/datastructures/fmpriorityqueue.hpp"

#include "base/unit_tests/unittests_position.h"

/*
#include "planners/fast_marching/console/console.h"

#include "planners/fast_marching/datastructures/fmfibheap.hpp"

#include "planners/fast_marching/io/maploader.hpp"
#include "planners/fast_marching/io/gridplotter.hpp"
#include "planners/fast_marching/io/gridwriter.hpp"
*/
const char kPathSeperator =
        #ifdef _WIN32
        '\\';
#else
        '/';
#endif
using namespace std;

//typedef nDGridMap<FMCell, 2> FMGrid2D;
//typedef std::vector<std::array<double, 2>> Path2D;

//typedef array<unsigned int, 3> Coord3D;

//sdf_tools::COLLISION_CELL _free_cell(0.0);
//sdf_tools::COLLISION_CELL _obst_cell(1.0);
//sdf_tools::COLLISION_CELL _oob_cell(INFINITY);

/*
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

void loadMyMapFromImg(nDGridMap<FMCell, 2>& grid) {

    typedef uint8_t Pixel;

    std::vector<unsigned int> obs;

    cv::Mat image = cv::imread("C:\\GitHub\\OpenMACE\\F3_Sample.png", cv::IMREAD_GRAYSCALE);
    cv::Mat flipVertical = image;
    cv::flip(image, flipVertical, 0);
    std::array<unsigned int, 2> dimsize;
    unsigned int width = image.size().width;
    unsigned int height = image.size().height;

    dimsize[0] = width;
    dimsize[1] = height;
    grid.resize(dimsize);


    // Naive pixel access
    // Loop over all rows
    for (int r = 0; r < flipVertical.rows; r++)
    {
      // Loop over all columns
      for ( int c = 0; c < flipVertical.cols; c++)
      {
        // Obtain pixel at (r, c)
        Pixel pixel = flipVertical.at<Pixel>(r, c);
        double occupancy = pixel/255;
        unsigned int idx = width*r+c;
        grid[idx].setOccupancy(occupancy);
        if (grid[idx].isOccupied())
            obs.push_back(idx);
      }

    }
    grid.setOccupiedCells(std::move(obs));
}

void plotMyArrivalTimesPath(nDGridMap<FMCell, 2>& grid, const Path2D& path, std::string name = "") {
    std::array<unsigned int,2> d = grid.getDimSizes();
    double max_val = grid.getMaxValue();
    cv::Mat image = cv::imread("C:\\GitHub\\OpenMACE\\F3_Sample.png", cv::IMREAD_GRAYSCALE);
    unsigned int width = d[0];
    unsigned int height = d[1];

    std::cout<<"The path has been computed as having 1: "<<std::endl;
    cv::Mat result = image;
    for (int r = 0; r < height; r++)
    {
      // Loop over all columns
      for ( int c = 0; c < width; c++)
      {
        result.at<uint8_t>(r, c) = grid[width*r+c].getValue()/max_val*255;
      }

    }

    std::cout<<"The path has been computed as having 2: "<<std::endl;

    for (unsigned int i = 0; i< path.size(); i++)
        result.at<uint8_t>(static_cast<unsigned int>(path[i][1]), (static_cast<unsigned int>(path[i][0]))) = 255;

    std::cout<<"The path has been computed as having 3: "<<std::endl;

    cv::Mat colorResult;
    cv::applyColorMap(result, colorResult, cv::COLORMAP_JET);

    cv::imshow("Result Map Image",colorResult);
    cv::waitKey(0);
}
*/

int main(int argc, char *argv[])
{
    UNUSED(argc); UNUSED(argv);

    //runPositionTests();
    mace::pose::GeodeticPosition_3D swarmOrigin(-35.3631970,149.1653205,584);
    mace::pose::GeodeticPosition_3D vehicleOrigin(-35.363261,149.165230,583.83);

    double bearingTo = swarmOrigin.polarBearingTo(&vehicleOrigin);
    double translationalDistance = swarmOrigin.distanceBetween2D(&vehicleOrigin);
    double altitudeDifference = swarmOrigin.deltaAltitude(&vehicleOrigin);

    double distanceTranslateX = translationalDistance * cos(correctForAcuteAngle(bearingTo));
    double distanceTranslateY = translationalDistance * sin(correctForAcuteAngle(bearingTo));
    correctSignFromPolar(distanceTranslateX, distanceTranslateY, bearingTo);
    Eigen::Vector3d test(distanceTranslateY, distanceTranslateX, altitudeDifference);
    std::cout<<"Waiting here."<<std::endl;
//    m_swarmTOvehicleHome.translation() = m_vehicleHomeTOswarm.translation() * -1;
    /*
    Path2D path;

    string filename = "map.jpg";
    mace::planners_graph::GraphNode graphNode;

    // Loading grid.
    FMGrid2D grid_fm2;

    loadMyMapFromImg(grid_fm2); // Loading from image
    std::vector<Solver<FMGrid2D>*> solvers;
    //solvers.push_back(new FM2Star<FMGrid2D, FMPriorityQueue<FMCell>>("FM2*_SFMM_Dist", DISTANCE));
    solvers.push_back(new FM2Star<FMGrid2D, FMPriorityQueue<FMCell> >("FM2*_SFMM_Time"));

    // Solvers declaration.
//    FM2<FMGrid2D>* solver = new FM2<FMGrid2D>("FM2_Dary");

//    FM2<FMGrid2D>* solver = new FM2<FMGrid2D, FMFibHeap<FMCell> >("FM2_Fib"));
//    FM2<FMGrid2D>* solver = new FM2<FMGrid2D, FMPriorityQueue<FMCell> >("FM2_SFMM"));
//    FM2<FMGrid2D>* solver = new FM2Star<FMGrid2D>("FM2*_Dary_Dist", DISTANCE));
//    FM2<FMGrid2D>* solver = new FM2Star<FMGrid2D>("FM2*_Dary_Time"));
//    FM2<FMGrid2D>* solver = new FM2Star<FMGrid2D, FMFibHeap<FMCell> >("FM2*_Fib_Time"));
//    FM2<FMGrid2D>* solver = new FM2Star<FMGrid2D, FMFibHeap<FMCell> >("FM2*_Fib_Dist", DISTANCE));
//    FM2<FMGrid2D>* solver = new FM2Star<FMGrid2D, FMPriorityQueue<FMCell> >("FM2*_SFMM_Time"));

    for (Solver<FMGrid2D>* s :solvers)
    {
        s->setEnvironment(&grid_fm2);
        s->setInitialAndGoalPoints({315, 20}, {315, 650}); // Init and goal points directly set.

        s->compute();
        std::cout << "\tElapsed "<< s->getName() <<" time: " << s->getTime() << " ms" << '\n';

        vector<double> path_vels;
        s->as<FM2<FMGrid2D>>()->computePath(&path, &path_vels);
        std::cout<<"The path has been computed as having: "<<std::endl;
        plotMyArrivalTimesPath(grid_fm2,path);

    }

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

    return 0;
}
