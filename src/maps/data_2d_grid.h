#ifndef DATA_2D_GRID_H
#define DATA_2D_GRID_H

#include <iostream>
#include <vector>

#include "common/common.h"
#include "common/class_forward.h"

#include "iterators/grid_map_iterator.h"
#include "base_grid_map.h"

namespace mace {
namespace maps {


template <class T>
class Data2DGrid : public BaseGridMap
{
public:

    Data2DGrid(const T *fill_value,
               const double &x_min = -10, const double &x_max = 10,
               const double &y_min = -10, const double &y_max = 10,
               const double &x_res = 0.5, const double &y_res = 0.5,
               const pose::CartesianPosition_2D &origin = pose::CartesianPosition_2D(),
               const double &rotationAngle = 0.0):
        BaseGridMap(x_min,x_max,y_min,y_max,x_res,y_res,origin, rotationAngle)
    {
        sizeGrid(fill_value);
    }

    Data2DGrid(const T *fill_value,
               const double &x_length, const double &y_length,
               const double &x_res, const double &y_res,
               const pose::CartesianPosition_2D &origin = pose::CartesianPosition_2D(),
               const double &rotationAngle = 0.0):
        BaseGridMap(x_length,y_length,x_res,y_res,origin, rotationAngle)
    {
        sizeGrid(fill_value);
    }

    Data2DGrid(const Data2DGrid &copy):
        BaseGridMap(copy)
    {
        this->m_defaultFill = copy.getFill();
        this->clear();
        mace::maps::GridMapIterator it(this);
        for(;!it.isPastEnd();++it)
        {
            T* ptr = this->getCellByIndex(*it);
            if(ptr != nullptr)
                *ptr = *copy.getCellByIndex(*it);
        }
    }


    virtual ~Data2DGrid() = default;

    //!
    //! \brief clear
    //!
    void clear()
    {
        m_dataMap.clear();
        m_dataMap.resize(xSize*ySize);
        fill(m_defaultFill);
    }

    bool updateGridResolution(const double &x_res, const double &y_res)
    {
        return this->updateGridSize(this->xMin,this->xMax,this->yMin,this->yMax,x_res,y_res);
    }

    // min/max X/Y values are assumed in standard coordinate (cartesian) frame
    //!
    //! \brief updateGridSize Update the underlying map layers size to the given dimensions. NOTE: The values given are assumed to be in a standard cartesian coordinate frame (i.e. not rotated)
    //! \param minX Minimum x value
    //! \param maxX Maximum x value
    //! \param minY Minimum y value
    //! \param maxY Maximum y value
    //! \param xRes X resolution
    //! \param yRes Y Resolution
    //! \return True if successful, false otherwise
    //!
    bool updateGridSize(const double &minX, const double &maxX, const double &minY, const double &maxY, const double &x_res, const double &y_res) override
    {
        bool resolutionChanged = false;

        if((minX != this->xMin) || (maxX != this->xMax) ||
                (minY != this->yMin) || (maxY != this->yMax) ||
                (x_res != this->xResolution) || (y_res != this->yResolution))
        {
            if((x_res != this->xResolution) || (y_res != this->yResolution))
            {
                resolutionChanged = true;
            }
            //First clone this object
            Data2DGrid* clone = new Data2DGrid(*this);
            //update the underlying size structure
            BaseGridMap::resizeGrid(minX,maxX,minY,maxY,x_res,y_res);

            //clear this contents and update with the default values
            this->clear();
            //copy the contents over
            double xPos, yPos;
            mace::maps::GridMapIterator it(clone);
            for(;!it.isPastEnd();++it)
            {
                const T* ptr = clone->getCellByIndex(*it);
                clone->getPositionFromIndex(*it,xPos,yPos);
                T* currentValue = this->getCellByPos(xPos,yPos);
                if(currentValue != nullptr)
                {
//                    if(*currentValue != this->getFill())
//                    {
//                        clone->getPositionFromIndex(*it,xPos,yPos);
//                        int thisIndex = this->indexFromPos(xPos,yPos);
//                        int otherIndex = *it;
//                        std::cout<<"I was already assigned a value."<<std::endl;

//                    }

                    *currentValue = *ptr;

                }
                else
                {
                    std::cout<<"The value had a nullptr?"<<std::endl;
                }
            }
            delete clone;
            clone = nullptr;
        }
        return resolutionChanged;
    }

    bool updateGridSizeByLength(const double &x_length = 10.0, const double &y_length = 10.0,
                                const double &x_res = 0.5, const double &y_res = 0.5) override
    {
        bool resolutionChanged = false;

        if((abs((this->xMax - this->xMin) - x_length) < 0.001) ||
                (abs((this->yMax - this->yMin) - y_length) < 0.001) ||
                (x_res != this->xResolution) || (y_res != this->yResolution))
        {
            resolutionChanged = true;
            //update the underlying size structure
            BaseGridMap::resizeGridSizeByLength(x_length,y_length,x_res,y_res);
        }

        return resolutionChanged;
    }

    //!
    //! \brief fill
    //! \param value
    //!
    void fill(const T &value)
    {
        for (typename std::vector<T>::iterator it = m_dataMap.begin(); it != m_dataMap.end(); ++it)
            *it = value;
    }

    bool findIndex(const T* find, unsigned int &index) const
    {
        for (size_t i = 0; i < getSize(); i++)
            if(find == &m_dataMap.at(i))
            {
                index = i;
                return true;
            }
        return false;
    }

    std::vector<int> getCellNeighbors(const unsigned int &index, bool ignoreDiagonals = false)
    {
        std::vector<int> rtnCells;

        unsigned int indexX, indexY;
        getIndexDecomposed(index,indexX,indexY);

        int startY = (((int)indexY + 1) >= ((int)ySize - 1)) ? (ySize - 1) : (int)indexY + 1;
        int endY = (((int)indexY - 1) >= 0) ? (indexY - 1) : 0;

        for(int i = startY; i >= endY; i--)
        {
            int startX = (((int)indexX - 1) >= 0) ? (indexX - 1) : 0;
            int endX = (((int)indexX + 1) >= (int)xSize-1) ? ((int)xSize - 1) : (int)indexX+1 ;

            if(ignoreDiagonals && (i != (int)indexY))
            {
                    startX = (int)indexX;
                    endX = (int)indexX;
            }

            for(int j = startX; j <= endX; j++)
            {
                if((j > (int)xSize - 1) || ((i == (int)indexY) && (j == (int)indexX)))
                    continue;
                rtnCells.push_back(this->indexFromPos(j,i));
            }
        }
        return rtnCells;
    }

    T getFill() const
    {
        return this->m_defaultFill;
    }

    T* getCellByIndex(const unsigned int &index)
    {
        if (index > (this->getNodeCount() - 1))
            return nullptr;
        else
            return &m_dataMap[index];
    }

    const T* getCellByIndex(const unsigned int &index) const
    {
        if (index > (this->getNodeCount() - 1))
            return nullptr;
        else
            return &m_dataMap[index];
    }

    //!
    //! \brief getCellByPos
    //! \param x
    //! \param y
    //! \return
    //!
    T* getCellByPos(const double &x, const double &y)
    {
        // Rotate point about origin before grabbing data back:
        double rotatedX, rotatedY;
        if(this->originPosition.areAllPositionsValid()) {
            double originX = this->getOriginPosition().getXPosition();
            double originY = this->getOriginPosition().getYPosition();
            geometry::rotatePoint_2D(rotatedX, rotatedY, x, y, -this->getRotationAngleDegrees(), originX, originY);
        }
        else {
            geometry::rotatePoint_2D(rotatedX, rotatedY, x, y, -this->getRotationAngleDegrees());
        }


        int cx = indexFromXPos(rotatedX);
        int cy = indexFromYPos(rotatedY);
        if (cx < 0 || cx >= static_cast<int>(xSize)) return nullptr; //implies something greater than the X range was asked for
        if (cy < 0 || cy >= static_cast<int>(ySize)) return nullptr; //implies something greater than the Y range was asked for
        return &m_dataMap[cx + cy * xSize];
    }

    //!
    //! \brief getCellByIndex
    //! \param xIndex
    //! \param yIndex
    //! \return
    //!
    T* getCellByPosIndex(const unsigned int &xIndex, const unsigned int &yIndex) const
    {
        if (xIndex >= xSize || yIndex >= ySize)
            return nullptr;
        else
            return &m_dataMap[xIndex + yIndex * xSize];
    }

    //!
    //! \brief getCellByPosIndex
    //! \param xIndex
    //! \param yIndex
    //! \return
    //!
    T* getCellByPosIndex(const unsigned int &xIndex, const unsigned int &yIndex)
    {
        if (xIndex >= xSize || yIndex >= ySize)
            return nullptr;
        else
            return &m_dataMap[xIndex + yIndex * xSize];
    }


    std::vector<T> getDataMap() const
    {
        return this->m_dataMap;
    }

    bool cellValueEqualTo(const unsigned int &index, const T *value) const
    {
        if(m_dataMap[index] == *value)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

protected:
    void sizeGrid(const T *fill_value)
    {
        // Cells memory:
        if (fill_value)
        {
            m_defaultFill = *fill_value;
            m_dataMap.assign(xSize * ySize, *fill_value);
        }
        else
            m_dataMap.resize(xSize * ySize);
    }

public:

    const T* operator [](const unsigned int &index)const
    {
        if (index > (this->getNodeCount() - 1))
            return nullptr;
        else
            return &m_dataMap[index];
    }

    Data2DGrid& operator = (const Data2DGrid &rhs)
    {
        BaseGridMap::operator ==(rhs);
        this->m_defaultFill = rhs.getFill();
        this->m_dataMap = rhs.getDataMap();
        return *this;
    }

    bool operator == (const Data2DGrid &rhs) const
    {
        if(!BaseGridMap::operator ==(rhs))
            return false;
        if(this->m_defaultFill != rhs.m_defaultFill){
            return false;
        }
        mace::maps::GridMapIterator it(this);
        for(;!it.isPastEnd();++it)
        {
            const T* ptr = this->getCellByIndex(*it);
            if(*ptr != *rhs.getCellByIndex(*it))
                return false;
        }
        return true;
    }

    bool operator != (const BaseGridMap &rhs) const{
        return !(*this == rhs);
    }


protected:
    //!
    //! \brief m_dataMap
    //!
    std::vector<T> m_dataMap;

    //!
    //! \brief m_defaultFill
    //!
    T m_defaultFill;

};



} //end of namespace maps
} //end of namespace mace

#endif // DATA_2D_GRID_H
