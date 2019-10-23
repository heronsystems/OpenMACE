#include <QCoreApplication>
#include <iostream>
#include <QFile>
#include <QTextStream>
#include <QStringList>
#include <vector>
#include <array>

#include "maps/costmaps/layers/costmap_base_layer.h"
#include "maps/costmaps/layers/costmap_inflation_layer.h"
#include "maps/iterators/line_map_iterator.h"
#include "maps/iterators/grid_map_iterator.h"
#include "base/geometry/polygon_cartesian.h"

#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include "graphs/signed_distance_fields/collision_map.hpp"

#include "planners/fast_marching/ndgridmap/fmcell.h"
#include "planners/fast_marching/ndgridmap/ndgridmap.hpp"
#include "planners/fast_marching/fm2/fm2.hpp"
#include "planners/graph_planning_node.h"
#include "planners/fast_marching/fm2/fm2star.hpp"
#include "planners/fast_marching/fm/fmm.hpp"
#include "planners/fast_marching/datastructures/fmpriorityqueue.hpp"
#include "graphs/signed_distance_fields/collision_map.hpp"

#include "base/unit_tests/unittests_position.h"
#include "base/geometry/rotate_2d.h"
#include "maps/data_2d_grid.h"
#include "maps/occupancy_definition.h"


#include "planners/fast_marching/console/console.h"

#include "planners/fast_marching/datastructures/fmfibheap.hpp"

const char kPathSeperator =
        #ifdef _WIN32
        '\\';
#else
        '/';
#endif
using namespace std;

typedef nDGridMap<FMCell, 2> FMGrid2D;
typedef std::vector<std::array<double, 2>> Path2D;

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

void loadPlanningMapFromCostmap(const mace::costmap::Costmap_BaseLayer &obstacleLayer,const sdf_tools::CollisionMapGrid &collision_map, nDGridMap<FMCell, 2>& grid)
{
    std::vector<unsigned int> obs;

    mace::maps::GridMapIterator gridMapItr(&obstacleLayer);
    const uint8_t* value;

    std::array<unsigned int, 2> dimsize;
    dimsize[0] = obstacleLayer.getSizeX();
    dimsize[1] = obstacleLayer.getSizeY();
    grid.resize(dimsize);
    double velocity = 0.0;
    auto EDT = collision_map.ExtractDistanceField(mace::costmap::Costmap2D::LETHAL_OBSTACLE);

    for(; !gridMapItr.isPastEnd(); ++gridMapItr)
    {
        double xPos,yPos;
        obstacleLayer.getPositionFromIndex(*gridMapItr,xPos,yPos);
        double distance = std::sqrt(EDT.GetImmutable(xPos,yPos,0.0).first.distance_square) * 0.5;
        velocity = velMapping(distance,2.0);

        value = obstacleLayer[*gridMapItr];

        if(*value > mace::costmap::Costmap2D::FREE_SPACE)
        {

            grid[*gridMapItr].setOccupancy(0.0);
            obs.push_back(*gridMapItr);
        }
        else
            grid[*gridMapItr].setOccupancy(velocity);

    } //end of for loop iterator
    grid.setOccupiedCells(std::move(obs));
    grid.setLeafSize(0.5);

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
        {
            std::cout<<"The grid is occupied so: "<<std::to_string(occupancy)<<std::endl;
            obs.push_back(idx);
        }
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

    cv::destroyAllWindows();
    cv::Mat colorResult;
    cv::applyColorMap(result, colorResult, cv::COLORMAP_JET);

    cv::imshow("Result Map Image",colorResult);
    cv::waitKey(0);
}


int main(int argc, char *argv[])
{
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

    double rotatedX,rotatedY,originX =1.0,originY=1.0,angle =45.0;

    Eigen::Transform<double,2,Eigen::Affine> worldToMap = Eigen::Transform<double, 2, Eigen::Affine>::Identity();;
    worldToMap.translation() = Eigen::Vector2d(10,10);
    worldToMap.rotate(0.785398);

    mace::geometry::rotatePoint_2D(rotatedX, rotatedY, originX, originY, angle, 0.0, 0.0);

    Eigen::Vector2d worldCoordinates(0.0,0.0);
    Eigen::Vector2d mapCoordinates;
    mapCoordinates = (worldToMap.rotation() * worldCoordinates) + worldToMap.translation();

    std::cout<<"Waiting here."<<std::endl;

    mace::pose::CartesianPosition_2D vertex1(-59,-13);
    mace::pose::CartesianPosition_2D vertex2(28,-13);
    mace::pose::CartesianPosition_2D vertex3(28,13);
    mace::pose::CartesianPosition_2D vertex4(-59,13);

    mace::geometry::Polygon_Cartesian boundingPolygon;
    boundingPolygon.appendVertex(vertex1);
    boundingPolygon.appendVertex(vertex2);
    boundingPolygon.appendVertex(vertex3);
    boundingPolygon.appendVertex(vertex4);

    mace::costmap::Costmap_BaseLayer staticLayer("static_layer",mace::costmap::Costmap2D::FREE_SPACE,-59,28,-13,13,0.5);
    staticLayer.outlineBoundary(boundingPolygon,mace::costmap::Costmap2D::LETHAL_OBSTACLE);

    uint8_t* value;

    value = staticLayer.getCellByPos(-30.0,0.0);
    *value = mace::costmap::Costmap2D::LETHAL_OBSTACLE;

    value = staticLayer.getCellByPos(0.0,0.0);
    *value = mace::costmap::Costmap2D::LETHAL_OBSTACLE;

    mace::costmap::Costmap_InflationLayer inflationLayer("inflation_layer",mace::costmap::Costmap2D::FREE_SPACE,-59,28,-13,13,0.5);
    inflationLayer.setScribedRadii(1.0,2.0);
    inflationLayer.setInflationParameters(2.5,2.0);
    inflationLayer.enableLayer(true);
    inflationLayer.updateCosts(staticLayer,0,0,staticLayer.getSizeX()-1, staticLayer.getSizeY()-1);

    //Construct a collision map
    sdf_tools::CollisionMapGrid collisionGrid(0.5,staticLayer.getSizeX(),staticLayer.getSizeY(),1,mace::costmap::Costmap2D::FREE_SPACE,mace::costmap::Costmap2D::LETHAL_OBSTACLE);

    mace::maps::GridMapIterator gridMapItr(&inflationLayer);

    std::array<unsigned int, 2> dimsize;
    dimsize[0] = staticLayer.getSizeX();
    dimsize[1] = staticLayer.getSizeY();
    for(; !gridMapItr.isPastEnd(); ++gridMapItr)
    {
        double xPos, yPos;
        staticLayer.getPositionFromIndex(*gridMapItr,xPos,yPos);
        value = staticLayer[*gridMapItr];
        if(*value > mace::costmap::Costmap2D::FREE_SPACE)
            collisionGrid.SetValue(xPos,yPos,0.0,mace::costmap::Costmap2D::LETHAL_OBSTACLE);
    } //end of for loop iterator


    Path2D path;

    // Loading grid.
    FMGrid2D grid_fm2;
//    loadMyMapFromImg(grid_fm2);
    loadPlanningMapFromCostmap(staticLayer, collisionGrid, grid_fm2); // Loading from image
    std::vector<Solver<FMGrid2D>*> solvers;

    // Solvers declaration.
//    solvers.push_back(new FM2<FMGrid2D>("FM2_Dary"));
//    FM2<FMGrid2D>* solver = new FM2<FMGrid2D, FMFibHeap<FMCell> >("FM2_Fib"));
//    FM2<FMGrid2D>* solver = new FM2<FMGrid2D, FMPriorityQueue<FMCell> >("FM2_SFMM"));
//    solvers.push_back(new FM2Star<FMGrid2D>("FM2*_Dary_Dist", DISTANCE));
//    solvers.push_back(new FM2Star<FMGrid2D>("FM2*_Dary_Time"));
//    solvers.push_back(new FM2Star<FMGrid2D, FMFibHeap<FMCell> >("FM2*_Fib_Time"));
    solvers.push_back(new FM2Star<FMGrid2D, FMFibHeap<FMCell> >("FM2*_Fib_Dist", DISTANCE));
//    solvers.push_back(new FM2Star<FMGrid2D, FMPriorityQueue<FMCell>>("FM2*_SFMM_Time",DISTANCE));

    for (Solver<FMGrid2D>* s :solvers)
    {
        s->setEnvironment(&grid_fm2);
        s->setInitialAndGoalPoints({8, 8}, {160, 41}); // Init and goal points directly set.

        s->compute();
        std::cout << "\tElapsed "<< s->getName() <<" time: " << s->getTime() << " ms" << '\n';

        vector<double> path_vels;
        s->as<FM2<FMGrid2D>>()->computePath(&path, &path_vels);
        std::cout<<"The path has been computed as having: "<<std::endl;
        plotMyArrivalTimesPath(grid_fm2,path);
    }

    return 0;
}
