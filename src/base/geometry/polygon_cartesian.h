#ifndef POLYGON_CARTESIAN_H
#define POLYGON_CARTESIAN_H

#include "base_polygon.h"
#include "list"
#include "base/pose/abstract_cartesian_position.h"

namespace mace{
namespace geometry {

using namespace pose;

class Polygon_Cartesian : public PolygonBase<CartesianPosition_2D>
{
public:

    Polygon_Cartesian(const std::string &descriptor = "2D Cartesian Polygon");

    Polygon_Cartesian(const std::vector<Abstract_CartesianPosition> &vector, const std::string &descriptor = "2D Cartesian Polygon");

    Polygon_Cartesian(const Polygon_Cartesian &copy);


    ~Polygon_Cartesian() override = default;

    //!
    //! \brief getBoundingRect
    //! \return
    //!
    Polygon_Cartesian getBoundingRect() const;


    void getBoundingValues(double &xMin, double &minY, double &maxX, double &maxY) const;

    //!
    //! \brief contains
    //! \param point
    //! \param onLineCheck
    //! \return
    //!
    bool contains(const CartesianPosition_2D &point, const bool &onLineCheck = false) const;

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
    std::vector<bool> contains(std::vector<CartesianPosition_2D> &checkVector, const bool &onLineCheck = false);

    //!
    //! \brief getCenter
    //! \return
    //!
    CartesianPosition_2D getCenter() const;

    std::vector<int> findUndefinedVertices() const override
    {
        int index = 0;
        std::vector<int> nullItems;
        for(std::vector<CartesianPosition_2D>::const_iterator it = m_vertex.begin(); it != m_vertex.end(); ++it) {
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
    CartesianPosition_2D getTopLeft() const override;
    CartesianPosition_2D getTopRight() const override;

    CartesianPosition_2D getBottomLeft() const override;
    CartesianPosition_2D getBottomRight() const override;

    void getCorners(CartesianPosition_2D &topLeft, CartesianPosition_2D &bottomRight) const override;

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
    Polygon_Cartesian& operator = (const Polygon_Cartesian &rhs)
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
    bool operator == (const Polygon_Cartesian &rhs) const
    {
        if(!PolygonBase<CartesianPosition_2D>::operator ==(rhs))
        {
            return false;
        }
        if(fabs(this->xMin - rhs.xMin) > std::numeric_limits<double>::epsilon())
        {
            return false;
        }
        if(fabs(this->xMax - rhs.xMax) > std::numeric_limits<double>::epsilon())
        {
            return false;
        }
        if(fabs(this->yMin - rhs.yMin) > std::numeric_limits<double>::epsilon())
        {
            return false;
        }
        if(fabs(this->yMax - rhs.yMax) > std::numeric_limits<double>::epsilon())
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
    bool operator != (const Polygon_Cartesian &rhs) const {
        return !(*this == rhs);
    }

private:
    double xMin, xMax;
    double yMin, yMax;
};

} //end of namespace geometry
} //end of namespace mace

#endif // POLYGON_2DC_H
