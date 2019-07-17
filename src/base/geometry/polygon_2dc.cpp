#include "polygon_cartesian.h"

namespace mace{
namespace geometry {

Polygon_Cartesian::Polygon_Cartesian(const std::string &descriptor):
    PolygonBase(descriptor)
{

}

Polygon_Cartesian::Polygon_Cartesian(const std::vector<CartesianPosition_2D> &vector, const std::string &descriptor):
    PolygonBase(vector, descriptor)
{
    updateBoundingBox();
}

Polygon_Cartesian::Polygon_Cartesian(const Polygon_Cartesian &copy):
    PolygonBase(copy)
{
    updateBoundingBox();
}

std::vector<bool> Polygon_Cartesian::contains(std::vector<CartesianPosition_2D> &checkVector, const bool &onLineCheck)
{
    std::vector<bool> rtnVector;
    const size_t size = checkVector.size();

    if (size >= 3){
        for(unsigned int i = 0; i < size; i++)
        {
            rtnVector.push_back(contains(checkVector[i].getXPosition(),checkVector[i].getYPosition(),onLineCheck));
        }
    }

    return rtnVector;
}

bool Polygon_Cartesian::contains(const double &x, const double &y, const bool &onLineCheck) const
{
    const size_t num = this->m_vertex.size();

    if (num < 3)
        return false;

    if(onLineCheck == true)
    {
        for(size_t i = 0; (i+1) < num; i++)
        {
            if((i+1) < num){
                if(isOnLine(m_vertex[i],m_vertex[i+1],x,y))
                    return true;
            }
        }

        /* this condition checks the last vertice to the first
         * this was moved outside the for loop to avoid checking
         * this condition at every vertice
         */

        if(isOnLine(m_vertex[num - 1],m_vertex[0],x,y))
            return true;
    }

    int counter = 0;
    for(size_t i = 0; i < num; i++)
    {
        if(m_vertex[i].getYPosition() <= y)
        {
            if(m_vertex[(i+1)%num].getYPosition() > y)
            {
                double value = isLeftOfInf(m_vertex[i],m_vertex[(i+1)%num],x,y);
                if(value > 0)
                    ++counter;
            }
        }
        else{
            if(m_vertex[(i+1)%num].getYPosition() <= y)
            {
                double value = isLeftOfInf(m_vertex[i],m_vertex[(i+1)%num],x,y);
                if(value < 0)
                    --counter;
            }
        }
    }

    return counter != 0;
}

bool Polygon_Cartesian::contains(const CartesianPosition_2D &point, const bool &onLineCheck) const
{
    return contains(point.getXPosition(), point.getYPosition(), onLineCheck);
}

void Polygon_Cartesian::updateBoundingBox()
{
    if (m_vertex.size() >= 3)
    {
        xMax = m_vertex[0].getXPosition();
        xMin = xMax;
        yMax = m_vertex[0].getYPosition();
        yMin = yMax;

        const size_t num = this->m_vertex.size();
        for(size_t i = 1; i < num; i++)
        {
            if(m_vertex[i].getXPosition() > xMax)
                xMax = m_vertex[i].getXPosition();
            else if(m_vertex[i].getXPosition() < xMin)
                xMin = m_vertex[i].getXPosition();
            if(m_vertex[i].getYPosition() > yMax)
                yMax = m_vertex[i].getYPosition();
            else if(m_vertex[i].getYPosition() < yMin)
                yMin = m_vertex[i].getYPosition();
        }
    }
}

void Polygon_Cartesian::getBoundingValues(double &minX, double &minY, double &maxX, double &maxY) const
{
    minX = xMin;
    minY = yMin;
    maxX = xMax;
    maxY = yMax;
}

Polygon_Cartesian Polygon_Cartesian::getBoundingRect() const
{
    Polygon_Cartesian polygon("Bounding Polygon");

    CartesianPosition_2D LL("Lower Left",xMin,yMin);
    CartesianPosition_2D UL("Upper Left",xMin,yMax);
    CartesianPosition_2D UR("Upper Right",xMax,yMax);
    CartesianPosition_2D LR("Lower Right",xMax,yMin);

    polygon.appendVertex(LL);
    polygon.appendVertex(UL);
    polygon.appendVertex(UR);
    polygon.appendVertex(LR);

    return polygon;
}


pose::CartesianPosition_2D Polygon_Cartesian::getCenter() const
{
    CartesianPosition_2D center("Center");
    size_t size = polygonSize();
    for (size_t i = 0; i < size; i++)
    {
        center += m_vertex[i];
    }
    center.setXPosition(center.getXPosition()/size);
    center.setYPosition(center.getYPosition()/size);
    return center;
}

CartesianPosition_2D Polygon_Cartesian::getTopLeft() const
{
    CartesianPosition_2D UL("Upper Left",xMin,yMax);
    return UL;
}

CartesianPosition_2D Polygon_Cartesian::getTopRight() const
{
    CartesianPosition_2D UL("Upper Right",xMax,yMax);
    return UL;
}

CartesianPosition_2D Polygon_Cartesian::getBottomRight() const
{
    CartesianPosition_2D LR("Lower Right",xMax,yMin);
    return LR;
}


CartesianPosition_2D Polygon_Cartesian::getBottomLeft() const
{
    CartesianPosition_2D BL("Bottom Left",xMin,yMin);
    return BL;
}

void Polygon_Cartesian::getCorners(CartesianPosition_2D &topLeft, CartesianPosition_2D &bottomRight) const
{
    topLeft = getTopLeft();
    bottomRight = getBottomRight();
}

CoordinateFrame Polygon_Cartesian::getVertexCoordinateFrame() const
{
    return CoordinateFrame::CF_LOCAL_ENU;
}

void Polygon_Cartesian::applyCoordinateShift(const double &distance, const double &bearing)
{
    for (size_t i = 0; i < polygonSize(); i++)
    {
        m_vertex.at(i).applyPositionalShiftFromPolar(distance,bearing);
    }
    updateBoundingBox();
}


} //end of namespace geometry
} //end of namespace mace
