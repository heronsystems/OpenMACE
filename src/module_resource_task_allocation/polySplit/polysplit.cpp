#include "polysplit.h"

/**
 * @brief initPolygon Given the boundary vertices, initialize the polygon for splitting
 * @param boundary boundary polygon that defines the environment
 * @param numVehicles Number of vehicles we are splitting the polygon for
 */
void PolySplit::initPolygon(const mace::geometry::Polygon_Cartesian &boundary, const int &numVehicles)
{
    polygon.clear();
    std::vector<mace::pose::CartesianPosition_2D> boundaryVector = boundary.getVector();

    for(auto vertex : boundaryVector) {
        polygon.push_back(Vector(vertex.getXPosition(), vertex.getYPosition()));        
    }

    Polygon polygonToSplit = polygon;
    for(int i = 0; i < numVehicles; i++) {
        Polygon poly1, poly2;
        Line cut;
//        double squareToCut = polygonToSplit.countSquare() / (boundaryVector.size()-i);
        double squareToCut = polygonToSplit.countSquare() / (numVehicles-i);
        polygonToSplit.split(squareToCut, poly1, poly2, cut);

        // Set polygon to keep based on areas::
//        if(poly1.countSquare() <= poly2.countSquare() || (boundaryVector.size() - i) == 1) {
        if(poly1.countSquare() <= poly2.countSquare() || (numVehicles - i) == 1) {
            splitPolygons.push_back(poly1);
            // Push centroid:
            splitCentroids.push_back(poly1.countCenter());
            // Set next polygon to split:
            polygonToSplit = poly2;

        }
        else {
            splitPolygons.push_back(poly2);
            // Push centroid:
            splitCentroids.push_back(poly2.countCenter());
            // Set next polygon to split:
            polygonToSplit = poly1;
        }
    }
}

/**
 * @brief getCentroids Return the centroids of the areas split from the environment boundary
 * @return Vector of points corresponding to area centroids
 */
std::vector<mace::pose::CartesianPosition_2D> PolySplit::getCentroids() const {
    std::vector<mace::pose::CartesianPosition_2D> centroids;
    for(auto centroid : splitCentroids) {
        mace::pose::CartesianPosition_2D tmpPt;
        tmpPt.setXPosition(centroid.x);
        tmpPt.setYPosition(centroid.y);
        centroids.push_back(tmpPt);
    }
    return centroids;
}


/**
 * @brief getPolygons Return the equal area polygons
 * @return Vector of polygons
 */
std::vector<mace::geometry::Polygon_Cartesian> PolySplit::getPolygons() const {
    std::vector<mace::geometry::Polygon_Cartesian> polygons;
    for(auto polygon : splitPolygons) {
        mace::geometry::Polygon_Cartesian tmpPoly;
        std::vector<mace::pose::CartesianPosition_2D> verts;
        std::vector<Vector> tmpVerts = polygon.getVectors();
        for(auto vert : tmpVerts) {
            mace::pose::CartesianPosition_2D tmpPos;
            tmpPos.setXPosition(vert.x);
            tmpPos.setYPosition(vert.y);
            verts.push_back(tmpPos);
        }
        tmpPoly.replaceVector(verts);
        polygons.push_back(tmpPoly);
    }

    return polygons;
}














