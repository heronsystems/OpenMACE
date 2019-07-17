#ifndef POLYSPLIT_H
#define POLYSPLIT_H

#include <polySplit/line.h>
#include <polySplit/vector.h>
#include <polySplit/polygon.h>

#include <environment_custom.h>

/**
 * @brief The PolySplit class handles splitting a polygon into equal areas and can return the centroids of each area
 */
class PolySplit
{
public:
    /**
     * @brief PolySplit Class default constructor
     */
    PolySplit() {}

    /**
     * @brief initPolygon Given the boundary vertices, initialize the polygon for splitting
     * @param boundary Boundary vertices
     * @param numVehicles Number of vehicles we are splitting the polygon for
     */
    void initPolygon(const mace::geometry::Polygon_Cartesian &boundary, const int &numVehicles);

    /**
     * @brief getCentroids Return the centroids of the areas split from the environment boundary
     * @return Vector of points corresponding to area centroids
     */
    std::vector<mace::geometry::Position<mace::geometry::CartesianPosition_2D> > getCentroids() const;

    /**
     * @brief getPolygons Return the equal area polygons
     * @return Vector of polygons
     */
    std::vector<mace::geometry::Polygon_Cartesian> getPolygons() const;

private:
    /**
     * @brief polygon Polygon container
     */
    Polygon polygon;

    /**
     * @brief splitPolygons Container for the polygons that are split from the original environment
     */
    std::vector<Polygon> splitPolygons;

    /**
     * @brief splitCentroids Centroids for the split polygons
     */
    std::vector<Vector> splitCentroids;
};

#endif // POLYSPLIT_H
