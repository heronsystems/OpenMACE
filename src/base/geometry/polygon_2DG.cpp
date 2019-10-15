#include "polygon_2DG.h"

namespace mace{
namespace geometry {

Polygon_2DG::Polygon_2DG(const std::string &descriptor):
    PolygonBase(descriptor)
{

}

Polygon_2DG::Polygon_2DG(const std::vector<GeodeticPosition_2D> &vector, const std::string &descriptor):
    PolygonBase(vector, descriptor)
{
    updateBoundingBox();
}

Polygon_2DG::Polygon_2DG(const Polygon_2DG &copy):
    PolygonBase(copy)
{
    updateBoundingBox();
}

std::vector<bool> Polygon_2DG::contains(std::vector<GeodeticPosition_2D> &checkVector, const bool &onLineCheck)
{
    std::vector<bool> rtnCheck;

    if(polygonSize() < 3)
        return rtnCheck;

    GeodeticPosition_3D origin3D(m_vertex.at(0).getLatitude(),m_vertex.at(0).getLongitude(),0.0);

    std::vector<CartesianPosition_2D> checkLocalVector;
    for(size_t i = 0; i < checkVector.size(); i++)
    {
        mace::pose::CartesianPosition_3D vertex3D;
        mace::pose::GeodeticPosition_3D checkVertex(checkVector[i].getLatitude(),checkVector[i].getLongitude(),0.0);
        mace::pose::DynamicsAid::GlobalPositionToLocal(&origin3D,&checkVertex,&vertex3D);
        mace::pose::CartesianPosition_2D vertex2D;
        vertex2D.setXPosition(vertex3D.getXPosition());
        vertex2D.setYPosition(vertex3D.getYPosition());
        checkLocalVector.push_back(vertex2D);
    }

    return m_localPolygon.contains(checkLocalVector,onLineCheck);
}

bool Polygon_2DG::contains(const double &latitude, const double &longitude, const bool &onLineCheck) const
{
    CartesianPosition_2D localVertex;
    GeodeticPosition_2D origin = m_vertex.at(0);
    GeodeticPosition_2D point(latitude,longitude);
    return contains(point,onLineCheck);
}

bool Polygon_2DG::contains(const GeodeticPosition_2D &point, const bool &onLineCheck) const
{
    CartesianPosition_3D localVertex;
    GeodeticPosition_3D checkPoint(point.getLatitude(),point.getLongitude(),0.0);
    GeodeticPosition_3D origin(m_vertex.at(0).getLatitude(),m_vertex.at(0).getLongitude(),0.0);
    mace::pose::DynamicsAid::GlobalPositionToLocal(&origin,&checkPoint,&localVertex);

    return m_localPolygon.contains(localVertex.getXPosition(), localVertex.getYPosition(),onLineCheck);
}

void Polygon_2DG::updateBoundingBox()
{
    m_localPolygon.clearPolygon();
    GeodeticPosition_3D origin(m_vertex.at(0).getLatitude(), m_vertex.at(0).getLongitude(),0.0);
    for(size_t i = 0; i < polygonSize(); i++)
    {
        CartesianPosition_3D localVertex3D;
        GeodeticPosition_3D vertex(m_vertex.at(i).getLatitude(), m_vertex.at(i).getLongitude(),0.0);
        mace::pose::DynamicsAid::GlobalPositionToLocal(&origin,&vertex,&localVertex3D);
        CartesianPosition_2D localVertex2D;
        m_localPolygon.appendVertex(localVertex2D);
    }

    if (m_vertex.size() >= 3)
    {
        xMax = m_vertex[0].getLongitude();
        xMin = xMax;
        yMax = m_vertex[0].getLatitude();
        yMin = yMax;

        const size_t num = this->m_vertex.size();
        for(size_t i = 1; i < num; i++)
        {
            if(m_vertex[i].getLongitude() > xMax)
                xMax = m_vertex[i].getLongitude();
            else if(m_vertex[i].getLongitude() < xMin)
                xMin = m_vertex[i].getLongitude();
            if(m_vertex[i].getLatitude() > yMax)
                yMax = m_vertex[i].getLatitude();
            else if(m_vertex[i].getLatitude() < yMin)
                yMin = m_vertex[i].getLatitude();
        }
    }
}

void Polygon_2DG::getBoundingValues(double &minX, double &minY, double &maxX, double &maxY) const
{
    minX = xMin;
    minY = yMin;
    maxX = xMax;
    maxY = yMax;
}

Polygon_2DG Polygon_2DG::getBoundingRect() const
{
    Polygon_2DG polygon("Bounding Polygon");

    GeodeticPosition_2D LL("Lower Left",xMin,yMin);
    GeodeticPosition_2D UL("Upper Left",xMin,yMax);
    GeodeticPosition_2D UR("Upper Right",xMax,yMax);
    GeodeticPosition_2D LR("Lower Right",xMax,yMin);

    polygon.appendVertex(LL);
    polygon.appendVertex(UL);
    polygon.appendVertex(UR);
    polygon.appendVertex(LR);

    return polygon;
}


GeodeticPosition_2D Polygon_2DG::getCenter() const
{

    GeodeticPosition_2D center2DG;
    center2DG.setName("Center");

    GeodeticPosition_3D center3DG;

    CartesianPosition_2D center2DC = m_localPolygon.getCenter();
    CartesianPosition_3D center3DC(center2DC.getXPosition(),center2DC.getYPosition(),0.0);
    GeodeticPosition_3D origin(m_vertex.at(0).getLatitude(), m_vertex.at(0).getLongitude(), 0.0);
    mace::pose::DynamicsAid::LocalPositionToGlobal(&origin,&center3DC,&center3DG);
    center2DG.setLatitude(center3DG.getLatitude());
    center2DG.setLongitude(center3DG.getLongitude());
    return center2DG;
}

GeodeticPosition_2D Polygon_2DG::getTopLeft() const
{
    GeodeticPosition_2D UL("Upper Left", xMin, yMax);
    return UL;
}

GeodeticPosition_2D Polygon_2DG::getTopRight() const
{
    GeodeticPosition_2D UL("Upper Right", xMax, yMax);
    return UL;
}

GeodeticPosition_2D Polygon_2DG::getBottomRight() const
{
    GeodeticPosition_2D LR("Lower Right", xMax, yMin);
    return LR;
}


GeodeticPosition_2D Polygon_2DG::getBottomLeft() const
{
    GeodeticPosition_2D BL("Bottom Left", xMin, yMin);
    return BL;
}

void Polygon_2DG::getCorners(GeodeticPosition_2D &topLeft, GeodeticPosition_2D &bottomRight) const
{
    topLeft = getTopLeft();
    bottomRight = getBottomRight();
}

void Polygon_2DG::applyCoordinateShift(const double &distance, const double &bearing)
{
    for (size_t i = 0; i < polygonSize(); i++)
    {
        m_vertex.at(i).applyPositionalShiftFromPolar(distance,bearing);
    }
    updateBoundingBox();
}


} //end of namespace geometry
} //end of namespace mace
