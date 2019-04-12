#ifndef POLYGON_2DC_H
#define POLYGON_2DC_H

#include "base_polygon.h"
#include "list"
#include "base/pose/cartesian_position_2D.h"

namespace mace{
namespace geometry {

using namespace pose;

class Polygon_2DC : public PolygonBase<Position<CartesianPosition_2D>>
{
public:

    Polygon_2DC(const std::string &descriptor = "2D Cartesian Polygon");

    Polygon_2DC(const std::vector<Position<CartesianPosition_2D>> &vector, const std::string &descriptor = "2D Cartesian Polygon");

    Polygon_2DC(const Polygon_2DC &copy);


    ~Polygon_2DC() = default;

    //!
    //! \brief getBoundingRect
    //! \return
    //!
    Polygon_2DC getBoundingRect() const;


    void getBoundingValues(double &xMin, double &minY, double &maxX, double &maxY) const;

    //!
    //! \brief contains
    //! \param point
    //! \param onLineCheck
    //! \return
    //!
    bool contains(const Position<CartesianPosition_2D> &point, const bool &onLineCheck = false) const;

    //!
    //! \brief contains
    //! \param x
    //! \param y
    //! \param onLineCheck
    //! \return
    //!
    bool contains(const double &x, const double &y, const bool &onLineCheck = false) const;

    //!
    //! \brief contains
    //! \param checkVector
    //! \param onLineCheck
    //! \return
    //!
    std::vector<bool> contains(std::vector<Position<CartesianPosition_2D>> &checkVector, const bool &onLineCheck = false);

    //!
    //! \brief getCenter
    //! \return
    //!
    Position<CartesianPosition_2D> getCenter() const;    

    std::vector<int> findUndefinedVertices() const override
    {
        int index = 0;
        std::vector<int> nullItems;
        for(std::vector<Position<CartesianPosition_2D>>::const_iterator it = m_vertex.begin(); it != m_vertex.end(); ++it) {
            if(!it->hasXBeenSet() && !it->hasYBeenSet())
            {
                //This should see that the value is null
                nullItems.push_back(index);
            }
            index++;
        }
        return nullItems;
    }

public:
    Position<CartesianPosition_2D> getTopLeft() const override;
    Position<CartesianPosition_2D> getTopRight() const override;

    Position<CartesianPosition_2D> getBottomLeft() const override;
    Position<CartesianPosition_2D> getBottomRight() const override;

    void getCorners(Position<CartesianPosition_2D> &topLeft, Position<CartesianPosition_2D> &bottomRight) const override;

    mace::pose::CoordinateFrame getVertexCoordinateFrame() const override;

    void applyCoordinateShift(const double &distance, const double &bearing);

public:
    double getXMin() const
    {
        return xMin;
    }

    double getYMin() const
    {
        return yMin;
    }

    double getXMax() const
    {
        return xMax;
    }

    double getYMax() const
    {
        return yMax;
    }

protected:
    void updateBoundingBox() override;

    /** Assignment Operators **/
public:

    //!
    //! \brief operator =
    //! \param rhs
    //! \return
    //!
    Polygon_2DC& operator = (const Polygon_2DC &rhs)
    {
        PolygonBase::operator =(rhs);
        this->xMin = rhs.xMin;
        this->yMin = rhs.yMin;
        this->xMax = rhs.xMax;
        this->yMax = rhs.yMax;
        return *this;
    }

    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const Polygon_2DC &rhs) const
    {
        if(!PolygonBase<Position<CartesianPosition_2D>>::operator ==(rhs))
        {
            return false;
        }
        if(this->xMin != rhs.xMin)
        {
            return false;
        }
        if(this->xMax != rhs.xMax)
        {
            return false;
        }
        if(this->yMin != rhs.yMin)
        {
            return false;
        }
        if(this->yMax != rhs.yMax)
        {
            return false;
        }
        return true;
    }

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return
    //!
    bool operator != (const Polygon_2DC &rhs) const {
        return !(*this == rhs);
    }

private:
    double xMin, xMax;
    double yMin, yMax;
};

} //end of namespace geometry
} //end of namespace mace

#endif // POLYGON_2DC_H
