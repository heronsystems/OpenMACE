#ifndef BASE_GRID_MAP_H
#define BASE_GRID_MAP_H

#include <cmath>

#include "common/class_forward.h"

#include "base/geometry/rotate_2d.h"
#include "base/pose/cartesian_position_2D.h"

namespace mace {
namespace maps {

struct mapSize {
    double x_min;
    double x_max;
    double y_min;
    double y_max;
    double x_res;
    double y_res;
};

typedef void(*CallbackFunctionPtr_UpdatedGridSize)(void*, mapSize&);

MACE_CLASS_FORWARD(BaseGridMap);

class BaseGridMap
{
public:
  

public:

    //!
    //! \brief BaseGridMap
    //! \param x_min
    //! \param x_max
    //! \param y_min
    //! \param y_max
    //! \param x_res
    //! \param y_res
    //! \param fill_value
    //!
    BaseGridMap(const double &x_length = 10.0, const double &y_length = 10.0,
                const double &x_res = 0.5, const double &y_res = 0.5,
                const pose::CartesianPosition_2D &position = pose::CartesianPosition_2D(),
                const double &rotationAngleDegrees = 0.0);

    BaseGridMap(const double &x_min, const double &x_max,
                const double &y_min, const double &y_max,
                const double &x_res, const double &y_res,
                const pose::CartesianPosition_2D &position,
                const double &rotationAngleDegrees);

    BaseGridMap(const BaseGridMap &copy);

    //!
    //! \brief ~Dynamic2DGrids
    //!
    virtual ~BaseGridMap() = default;

    void updateOriginPosition(const pose::CartesianPosition_2D &position);

    void getPositionFromIndex(const unsigned int &index, double &x, double &y) const
    {
        double tmpX, tmpY;
        unsigned int yIndex = (index/xSize);
        tmpY = yMin + yIndex * yResolution;

        unsigned int xIndex = (index % xSize);
        tmpX = xMin + xIndex * xResolution;

        // Rotate point about origin before passing back:
        if(this->originPosition.areAllPositionsValid()) {
            double originX = this->getOriginPosition().getXPosition();
            double originY = this->getOriginPosition().getYPosition();
            geometry::rotatePoint_2D(x, y, tmpX, tmpY, this->getRotationAngleDegrees(), originX, originY);
        }
        else {
            geometry::rotatePoint_2D(x, y, tmpX, tmpY, this->getRotationAngleDegrees());
        }

    }

    void getMinPositionFromIndex(const unsigned int &index, double &x, double &y) const
    {
        getPositionFromIndex(index,x,y);
    }

    void getMaxPositionFromIndex(const unsigned int &index, double &x, double &y) const
    {
        getPositionFromIndex(index,x,y);
        x+=xResolution;
        y+=yResolution;
    }

    void getIndexDecomposed(const unsigned int &index, unsigned int &xIndex, unsigned int &yIndex) const
    {
        yIndex = (index/xSize);
        xIndex = (index % xSize);
    }

    //!
    //! \brief indexFromXPos
    //! \param x
    //! \return
    //!
    unsigned int indexFromXPos(const double &x) const
    {
        // TODO-Pat: @Ken: I think the comment below is a valid assumption. Also, rotating just the y portion of the point doesn't make sense
        //                  - Every usage of this method is preceded by a rotation

        // No need to rotate point, it is assumed that this y position is already rotated to a standard cartesian frame

        double resultX = (x - xMin) / xResolution;
        resultX = (std::abs(resultX - round(resultX)) < xResolution) ? round(resultX) : resultX;
        unsigned int indexX = static_cast<unsigned int>(resultX);
        indexX = (indexX > (getSizeX() - 1)) ? (getSizeX() - 1) : indexX;
        return indexX;
    }

    //!
    //! \brief indexFromYPos
    //! \param y
    //! \return
    //!
    unsigned int indexFromYPos(const double &y) const
    {
        // TODO-Pat: @Ken: I think the comment below is a valid assumption. Also, rotating just the y portion of the point doesn't make sense
        //                  - Every usage of this method is preceded by a rotation

        // No need to rotate point, it is assumed that this y position is already rotated to a standard cartesian frame

        double resultY = (y - yMin) / yResolution;
        resultY = (std::abs(resultY - round(resultY)) < yResolution) ? round(resultY) : resultY;
        unsigned int indexY = static_cast<unsigned int>(resultY);
        indexY = (indexY > (getSizeY() - 1)) ? (getSizeY() - 1) : indexY;
        return indexY;
    }

    //!
    //! \brief indexFromPos
    //! \param x
    //! \param y
    //! \return
    //!
    unsigned int indexFromPos(const double &x, const double &y) const
    {
        // Rotate point about origin before passing back:
        double rotatedX, rotatedY;
        if(originPosition.areAllPositionsValid()) {
            double originX = this->getOriginPosition().getXPosition();
            double originY = this->getOriginPosition().getYPosition();
            geometry::rotatePoint_2D(rotatedX, rotatedY, x, y, -this->getRotationAngleDegrees(), originX, originY);
        }
        else {
            geometry::rotatePoint_2D(rotatedX, rotatedY, x, y, -this->getRotationAngleDegrees());
        }

        return indexFromXPos(rotatedX) + indexFromYPos(rotatedY) * this->xSize;
    }

    //!
    //! \brief getSizeX
    //! \return
    //!
    size_t getSizeX() const
    {
        return this->xSize;
    }

    //!
    //! \brief getSizeY
    //! \return
    //!
    size_t getSizeY() const
    {
        return this->ySize;
    }

    //!
    //! \brief getSize
    //! \return
    //!
    size_t getSize() const
    {
        return (this->xSize * this->ySize);
    }

    //!
    //! \brief getXMin
    //! \return
    //!
    double getXMin() const
    {
        return this->xMin;
    }

    //!
    //! \brief getYMin
    //! \return
    //!
    double getYMin() const
    {
        return this->yMin;
    }

    //!
    //! \brief getXMax
    //! \return
    //!
    double getXMax() const
    {
        return this->xMax;
    }

    //!
    //! \brief getYMax
    //! \return
    //!
    double getYMax() const
    {
        return this->yMax;
    }

    //!
    //! \brief getXResolution
    //! \return
    //!
    double getXResolution() const
    {
        return this->xResolution;
    }

    //!
    //! \brief getYResolution
    //! \return
    //!
    double getYResolution() const
    {
        return this->yResolution;
    }

    unsigned int getNodeCount() const
    {
        return xSize * ySize;
    }

    double getXLength() const
    {
        return xMax - xMin;
    }

    double getYLength() const
    {
        return yMax - yMin;
    }

    pose::CartesianPosition_2D getOriginPosition() const
    {
        return this->originPosition;
    }

    //!
    //! \brief getRotationAngleDegrees
    //! \return Angle in degrees (+ is CCW, - is CW)
    //!
    double getRotationAngleDegrees() const
    {
        return this->rotationAngleDegrees;
    }

public:
    void connectUpdatedGridSizeCallback(CallbackFunctionPtr_UpdatedGridSize cb, void *p)
    {
        m_CBUpdatedGridSize = cb;
        m_FunctionTarget = p;
    }
public:
    void callUpdatedGridSizeCallback()
    {
        mapSize obj;
        obj.x_max = this->xMax;
        obj.x_min = this->xMin;
        obj.x_res = this->xResolution;

        obj.y_max = this->yMax;
        obj.y_min = this->yMin;
        obj.y_res = this->yResolution;

        if((m_CBUpdatedGridSize != nullptr) && (m_FunctionTarget != nullptr))
        {
            m_CBUpdatedGridSize(m_FunctionTarget, obj);
        }
    }

public:
    virtual bool updateGridSize(const double &minX, const double &maxX, const double &minY, const double &maxY, const double &x_res, const double &y_res);

    virtual bool updateGridSizeByLength(const double &x_length = 10.0, const double &y_length = 10.0,
                     const double &x_res = 0.5, const double &y_res = 0.5);

    virtual void setXResolution(const double &x_res);

    virtual void setYResolution(const double &y_res);

    virtual void updateResolution(const double &x_res, const double &y_res);

    virtual void updateGridRotation(const double &angle);

protected:
    virtual void resizeGrid(const double &minX, const double &maxX, const double &minY, const double &maxY, const double &x_res, const double &y_res);

    virtual void resizeGridSizeByLength(const double &x_length = 10.0, const double &y_length = 10.0,
                     const double &x_res = 0.5, const double &y_res = 0.5);

public:

    BaseGridMap& operator = (const BaseGridMap &rhs)
    {
        this->originPosition = rhs.originPosition;
        this->xMin = rhs.xMin;
        this->xMax = rhs.xMax;
        this->yMin = rhs.yMin;
        this->yMax = rhs.yMax;
        this->xResolution = rhs.xResolution;
        this->yResolution = rhs.yResolution;
        this->xSize = rhs.xSize;
        this->ySize = rhs.ySize;
        this->rotationAngleDegrees = rhs.rotationAngleDegrees;
        return *this;
    }

    bool operator == (const BaseGridMap &rhs) const
    {
        if(this->originPosition != rhs.originPosition){
            return false;
        }
        if(fabs(this->xMin - rhs.xMin) > std::numeric_limits<double>::epsilon()){
            return false;
        }
        if(fabs(this->xMax - rhs.xMax) > std::numeric_limits<double>::epsilon()){
            return false;
        }
        if(fabs(this->yMin - rhs.yMin) > std::numeric_limits<double>::epsilon()){
            return false;
        }
        if(fabs(this->yMax - rhs.yMax) > std::numeric_limits<double>::epsilon()){
            return false;
        }
        if(fabs(this->xResolution - rhs.xResolution) > std::numeric_limits<double>::epsilon()){
            return false;
        }
        if(fabs(this->yResolution - rhs.yResolution) > std::numeric_limits<double>::epsilon()){
            return false;
        }
        if(this->xSize != rhs.xSize){
            return false;
        }
        if(this->ySize != rhs.ySize){
            return false;
        }
        if(fabs(this->rotationAngleDegrees - rhs.rotationAngleDegrees) > std::numeric_limits<double>::epsilon()) {
            return false;
        }
        return true;
    }

    bool operator != (const BaseGridMap &rhs) const{
        return !(*this == rhs);
    }

protected:
    CallbackFunctionPtr_UpdatedGridSize m_CBUpdatedGridSize;
    void *m_FunctionTarget;

protected:
    pose::CartesianPosition_2D originPosition; //!< Position of the map relative to the grid frame

    double xMin, yMin; //!< Description of members
    double xMax, yMax; //!< Description of members
    double xResolution, yResolution; //!< Description of members
    size_t xSize, ySize; //!< Description of members

    //!
    //! \brief rotationAngleDegrees + is CCW, - is CW
    //!
    double rotationAngleDegrees;
};

} //end of namespace maps
} //end of namespace mace

#endif // BASE_GRID_MAP_H
