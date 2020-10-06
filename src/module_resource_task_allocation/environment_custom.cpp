#include <environment_custom.h>

#include <algorithm>

#include <polySplit/polysplit.h>

#include <Eigen/Dense>

//using namespace voro;

namespace mace {
namespace geometry{

// This function returns a random double between 0 and 1
double rnd() {return double(rand())/RAND_MAX;}

/**
 * @brief Environment_Map constructor
 * @param verts Vector of vertices that make up the environment boundary
 * @param gridSpacing Spacing between grid points
 */
Environment_Map::Environment_Map(const Polygon_Cartesian &boundingPolygon, const double &gridSpacing, const bool &globalInstance) :
    m_boundary(boundingPolygon), m_globalInstance(globalInstance) {

    UNUSED(gridSpacing);

//    // Only generated and insert nodes if the RTA instance is a local instance (i.e. onboard a vehicle that needs to generate nodes)
//    if(!globalInstance) {
//        m_dataGrid = new mace::maps::Bounded2DGrid(m_boundary, gridSpacing, gridSpacing);
//    }
}

/**
 * @brief Environment_Map::computeBalancedVoronoi Use the number of vehicles and their positions to create a balanced Voronoi partition
 * @param vehicles Map of vehicles and their positions
 * @return Success or Failure
 */
bool Environment_Map::computeBalancedVoronoi(const std::map<int, CartesianPosition_2D> &vehicles) {
    UNUSED(vehicles);

    bool success = false;
//    if(m_boundary.getVector().size() > 2) {
//        // Step 1): Use the number of vehicles to create evenly spaced points in environment
//        int numVehicles = vehicles.size();

//        if(numVehicles > 1 && !m_globalInstance) {
//            //TODO Pat: Address issue of global and local instance
//            numVehicles = 1;
//            std::cout << "*_*_*_*_*_*_*_* TESTING: IN RTA - Local instance with multiple vehicles *_*_*_*_*_*_*_*" << std::endl;
//            // TODO: If this fires (i.e. local instances somehow have knowledge of multiple vehicles), then we need logic
//            //          to handle this. Basically, local instances should only have knowledge of their own position for RTA
//            //          positions. Local instances should only have knowledge of their operational boundary, so there's no
//            //          need to partition the received boundaries
//        }

//        PolySplit polygon;
//        polygon.initPolygon(m_boundary, numVehicles);

//        std::vector<Position<CartesianPosition_2D> > centroids = polygon.getCentroids();
//        if(centroids.size() != numVehicles) {
//            std::cout << "Balanced Voronoi: Number of vehicles does not match number of available polygons." << std::endl;
//        }
//        else {
//            std::vector<Cell_2DC> cellsVec;
//            std::vector<Polygon_Cartesian> polygons = polygon.getPolygons();
//            for(auto polygon : polygons) {
//                std::vector<Position<CartesianPosition_2D> > coords;
//                std::vector<Position<CartesianPosition_2D> > vertices = polygon.getVector();
//                for(auto vertex : vertices) {
//                    Position<CartesianPosition_2D> tmp;
//                    tmp.setXPosition(vertex.getXPosition());
//                    tmp.setYPosition(vertex.getYPosition());
//                    coords.push_back(tmp);
//                }

//                // Create cell and add to list:
//                Cell_2DC cell(coords, "2D Cartesian Polygon");
//                // Sort the cell vertices:
//                sortCellVerticesCCW(cell); // TODO: Optimize
//                // Only generated and insert nodes if the RTA instance is a local instance (i.e. onboard a vehicle that needs to generate nodes)
//                if(!m_globalInstance) {
//                    std::list<mace::pose::Position<mace::pose::CartesianPosition_2D>*> nodeList = m_dataGrid->getBoundedDataList();
//                    cell.insertNodes(nodeList, true);
//                }

//                cellsVec.push_back(cell);
//            }

//            if(cellsVec.size() > 0) {
//                success = true;
//            }

//            // Step 3): Assign cells to vehicle IDs based on distance to vehicle/site
//            std::vector<int> usedCellIndices;
//            for(auto vehicle : vehicles) {
//                double dist = std::numeric_limits<double>::max();
//                Cell_2DC tmpCell;
//                int counter = 0;
//                int index = counter;
//                for(auto cell : cellsVec) {
//                    double tmpDist = cell.getCenter().distanceTo(vehicle.second);
//                    if(tmpDist < dist && std::find(usedCellIndices.begin(), usedCellIndices.end(), counter) == usedCellIndices.end()) {
//                        dist = tmpDist;
//                        tmpCell = cell;
//                        index = counter;
//                    }

//                    counter++;
//                }
//                usedCellIndices.push_back(index);
//                cells.insert(std::make_pair(vehicle.first, tmpCell));


//                //! MTB - Removing
//                //! @pnolan Issue 139
//                //!printCellInfo(cells.at(1));
//                //!printCellInfo(cells.at(1));
//                //!

//            }
//        }
//    }


    // Return success or failure:
    return success;
}

/**
 * @brief computeVoronoi Given the bounding box and current vehicle positions, compute a voronoi diagram
 * @param cellVec Container for vector of cells to be assigned by vehicle distances
 * @param sitePositions Positions of sites (in x,y coordinates)
 * @return Success or Failure
 */
bool Environment_Map::computeVoronoi(std::vector<Cell_2DC> &cellVec, const std::vector<CartesianPosition_2D> &sitePositions) {
    UNUSED(cellVec);
    UNUSED(sitePositions);

    return false;

//    bool success = false;

//    // Set up constants for the container geometry
//    double x_min, y_min, x_max, y_max;
//    const double z_min = -0.5, z_max = 0.5;

//    m_boundary.getBoundingValues(x_min, y_min, x_max, y_max);

//    double x,y,z;
//    voronoicell_neighbor c;

//    // Set up the number of blocks that the container is divided into
//    const int n_x=1, n_y=1, n_z=1;

//    // Create a container with the geometry given above, and make it
//    // non-periodic in each of the three coordinates. Allocate space for
//    // eight particles within each computational block
//    container con(x_min,x_max,y_min,y_max,z_min,z_max,n_x,n_y,n_z,
//                 false,false,false,8);

//    // Add sites/particles into the container only if they are in the environment
//    int voronoiCounter = 0;

//    for(auto particle : sitePositions) {
//        voronoiCounter++;
//        con.put(voronoiCounter, particle.getXPosition(), particle.getYPosition(), 0);
//    }

//    // Loop over all sites in the container and compute each Voronoi
//    // cell
//    c_loop_all cl(con);
//    int dimension = 0;
//    if(cl.start()) do if(con.compute_cell(c,cl)) {
//        dimension+=1;
//    } while (cl.inc());

//    // Initialze containers:
//    std::vector<std::vector<int> > face_connections(dimension);
//    std::vector<std::vector<double> > vertex_positions(dimension);

//    int counter = 0;
//    if(cl.start()) do if(con.compute_cell(c,cl)) {
//        // Grab cell position:
//        cl.pos(x,y,z);

//        // Initialize vectors for vertex index and points:
//        std::vector<int> f_vert;
//        std::vector<double> v;

//        // Grab vertex indeces and points
//        c.face_vertices(f_vert);
//        c.vertices(x,y,z,v);

//        // Add to vector containers:
//        face_connections[counter] = f_vert;
//        vertex_positions[counter] = v;

//        // Initialize Cell variables:
//        int count = 1;
//        double xVal = NAN;
//        double yVal = NAN;
//        std::vector<Position<CartesianPosition_2D> > coords;
//        // Loop over vertex positions and determine which is the x coordinate, and which is y coordinate:
//        for(auto vertexCoord : vertex_positions[counter]) {
//            if(count == 1 ) {
//                // X coord:
//                xVal = vertexCoord;
//                count++;
//            }
//            else if(count == 2) {
//                // Y coord:
//                yVal = vertexCoord;
//                count++;
//            }
//            else {
//                // Z coord: add our point, reset counter
//                // Add point to our vector:
//                if(vertexCoord < 0) {
//                    Position<CartesianPosition_2D> tmp;
//                    tmp.setXPosition(xVal);
//                    tmp.setYPosition(yVal);
//                    coords.push_back(tmp);
//                }
//                count = 1;
//            }
//        }

//        // Increment counter:
//        counter += 1;

//        // Create cell and add to map:
//        Cell_2DC cell(coords, "2D Cartesian Polygon");
//        // Sort the cell vertices:
//        sortCellVerticesCCW(cell); // TODO: Optimize
//        // Only generated and insert nodes if the RTA instance is a local instance (i.e. onboard a vehicle that needs to generate nodes)
//        if(!m_globalInstance) {
//            std::list<mace::pose::Position<mace::pose::CartesianPosition_2D>*> nodeList = m_dataGrid->getBoundedDataList();
//            cell.insertNodes(nodeList, true);
//        }

//        cellVec.push_back(cell);

//      } while (cl.inc()); // Increment to the next cell:

//      if(cellVec.size() > 0) {
//          success = true;
//      }
//      return success;
}


/**
 * @brief sortCellVertices Sort the vertices of a cell in CCW fashion
 * @param cell Cell to update vertex ordering
 */
void Environment_Map::sortCellVerticesCCW(Cell_2DC &cell) {
    // 1) Create empty cell vertices vector
    // 2) Calculate polar angles and put into vector
    // 3) Find smallest angle
    // 4) Use index to add to cell vertices vector
    // 5) Remove smallest angle, find new smallest angle
    // 6) Use index to add to cell vertices vector

    //  1) Calculate angle between site and all vertices
    std::vector<double> angles;
    std::vector<CartesianPosition_2D> sortedVerts;
    for(auto cellVert : cell.getVector()) {
        double angle = atan2(cellVert.getYPosition() - cell.getCenter().getYPosition(), cellVert.getXPosition() - cell.getCenter().getXPosition()) * (180/M_PI);
        angles.push_back(angle);
    }

    while(sortedVerts.size() < cell.getVector().size()) {
        std::vector<double>::iterator result = std::min_element(std::begin(angles), std::end(angles));
        int index = std::distance(std::begin(angles), result);
        // Add vertex at this index to our new vector:
        sortedVerts.push_back(cell.getVector().at(index));

        // Set angle to MAX double value:
        angles.at(index) = std::numeric_limits<double>::max();
    }

    // Set our vertices to sorted vertices:
    cell.replaceVector(sortedVerts);
}

/**
 * @brief printCellInfo Print stats about a cell to the console
 * @param cell Cell to print stats for
 */
void Environment_Map::printCellInfo(const Cell_2DC &cell) {
    std::cout << " ******************** Printing Cell ******************** " << std::endl;

    // Print boundary vertices:
    std::cout << "      **** Boundary vertices: " << std::endl;
    std::vector<CartesianPosition_2D> boundaryVerts = cell.getVector();
    int tmpVertCounter = 0;
    for(auto vertex : boundaryVerts) {
        tmpVertCounter++;
        std::cout << "Vertex " << tmpVertCounter << ": (" << vertex.getXPosition() << ", " << vertex.getYPosition() << ")" << std::endl;
    }
    std::cout << std::endl << std::endl;

    // Print bounding rectangle:
    std::cout << "      **** Boundary rectangle vertices: " << std::endl;
    std::vector<CartesianPosition_2D> boundingRect = cell.getBoundingRect().getVector();
    int tmpBoundingVertCounter = 0;
    for(auto vertex : boundingRect) {
        tmpBoundingVertCounter++;
        std::cout << "Vertex " << tmpBoundingVertCounter << ": (" << vertex.getXPosition() << ", " << vertex.getYPosition() << ")" << std::endl;
    }
    std::cout << std::endl << std::endl;

    // Print center:
    std::cout << "      **** Center: " << std::endl;
    CartesianPosition_2D center = cell.getCenter();
    std::cout << "Center: (" << center.getXPosition() << ", " << center.getYPosition() << ")" << std::endl;

    // Print node stats:
    std::cout << "      **** Node stats: " << std::endl;
    std::vector<CartesianPosition_2D*> nodes = cell.getNodes();
    std::cout << "Num nodes: " << nodes.size() << std::endl;

    std::cout << " __________________ End Printing Cell __________________ " << std::endl;
}

} //end of namespace geometry
} //end of namespace mace
