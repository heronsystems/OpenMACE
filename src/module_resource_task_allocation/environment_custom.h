#ifndef ENVIRONMENT_CUSTOM_H
#define ENVIRONMENT_CUSTOM_H

#include <map>
#include <vector>
#include <limits>
#include <cmath>
#include <memory>
#include <iostream>

//#include "voropp/voro_index.hh"
//#include "voropp/container.hh"
//#include "voropp/v_compute.hh"
//#include "voropp/c_loops.hh"
#include <tuple>

#include "data_generic_command_item_topic/command_item_topic_components.h"
#include "base/pose/cartesian_position_2D.h"
#include "base/geometry/cell_2DC.h"
#include "maps/bounded_2D_grid.h"

#include "data/timer.h"


namespace mace {
namespace geometry{


/**
 * @brief The GridDirection enum to denote how we sort cell nodes
 */
enum GridDirection {
    NORTH_SOUTH,
    EAST_WEST,
    CLOSEST_POINT
};


/**
 * @brief The Environment_Map class holds the environment boundary verticies, bounding rectangle, and Nodes on a 2D grid
 */
class Environment_Map
{
public:
    /**
     * @brief Environment_Map constructor
     * @param boundingPolygon Polygon defining the environment boundary
     * @param gridSpacing Spacing between grid points
     * @param globalOrigin Global origin for environment
     */
    Environment_Map(const Polygon_Cartesian &boundingPolygon, const double &gridSpacing, const bool &globalInstance);

    ~Environment_Map()
    {
        clearDataGrid();
    }

    /**
     * @brief computeVoronoi Given the bounding box and current vehicle positions, compute a voronoi diagram
     * @param cellVec Container for vector of cells to be assigned by vehicle distances
     * @param sitePositions Positions of sites (in x,y coordinates)
     * @return Success or Failure
     */
    bool computeVoronoi(std::vector<Cell_2DC> &cellVec, const std::vector<mace::pose::CartesianPosition_2D> &sitePositions);

    /**
     * @brief Environment_Map::computeBalancedVoronoi Use the number of vehicles and their positions to create a balanced Voronoi partition
     * @param vehicles Map of vehicles and their positions
     * @return Success or Failure
     */
    bool computeBalancedVoronoi(const std::map<int, mace::pose::CartesianPosition_2D> &vehicles);

    /**
     * @brief getCells Return the cells that make up our Voronoi partition
     * @return Cells making up the voronoi partition
     */
    std::map<int, Cell_2DC> getCells() { return cells; }

    /**
     * @brief getBoundaryVerts Return the vector of points that make up the boundary
     * @return Vector of points making up a boundary
     */
    std::vector<mace::pose::CartesianPosition_2D> getBoundaryVerts() { return boundaryVerts ;}

    /**
     * @brief getBoundingBox Return the bounding polygon
     * @return Polygon representing the boundary
     */
    Polygon_Cartesian getBoundingPolygon() { return m_boundary; }


    /**
     * @brief printCellInfo Print stats about a cell to the console
     * @param cell Cell to print stats for
     */
    void printCellInfo(const Cell_2DC &cell);

private:

    /**
     * @brief sortCellVertices Sort the vertices of a cell in CCW fashion
     * @param cell Cell to update vertex ordering
     */
    void sortCellVerticesCCW(Cell_2DC &cell);


    /**
     * @brief clearDataGrid Clear and delete the data grid pointer
     */
    void clearDataGrid()
    {
        if(!m_globalInstance) {
            delete m_dataGrid;
            m_dataGrid = NULL;
        }
    }

private:


    /**
     * @brief m_dataGrid data structure holding a standardized grid of data
     * that is constrained by a Polygon_2DC boundary.
     */
    mace::maps::Bounded2DGrid* m_dataGrid;

    /**
     * @brief m_boundary data structure holding a vector of Cartesian_2D points
     * that define the boundary in which the environment is valid for vehicle
     * movement.
     */
    mace::geometry::Polygon_Cartesian m_boundary;

    /**
     * @brief boundaryVerts Vertices that make up the environment boundary
     */
    std::vector<mace::pose::CartesianPosition_2D> boundaryVerts;

    /**
     * @brief cells Container for cells corresponding to each vehicle
     */
    std::map<int, Cell_2DC> cells;

    bool m_globalInstance;
};


} //end of namespace geometry
} //end of namespace mace

#endif // ENVIRONMENT_CUSTOM_H
