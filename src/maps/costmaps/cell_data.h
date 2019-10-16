#ifndef CELLDATA_H
#define CELLDATA_H

#include <cmath>
#include <limits>

namespace mace{
namespace costmap{

class CellData
{
public:
    CellData(double distance, unsigned int index,
             unsigned int xIndex, unsigned int yIndex,
             unsigned int obstacleXIndex, unsigned int obstacleYIndex):
        m_Distance(distance), m_VectorIndex(index),
        m_XIndex(xIndex), m_YIndex(yIndex),
        m_ObstacleXIndex(obstacleXIndex), m_ObstacleYIndex(obstacleYIndex)
    {

    }


    ~CellData() = default;

    /** Relational Operators */
public:

    //!
    //! \brief operator <
    //! \param rhs
    //! \return
    //!
    bool operator < (const CellData &rhs) const
    {
        if(this->m_Distance >= rhs.m_Distance)
            return false;
        return true;
    }

    //!
    //! \brief operator >=
    //! \param rhs
    //! \return
    //!
    bool operator >= (const CellData &rhs) const
    {
        return !(*this < rhs);
    }

    //!
    //! \brief operator >
    //! \param rhs
    //! \return
    //!
    bool operator > (const CellData &rhs) const
    {
        if(this->m_Distance <= rhs.m_Distance)
            return false;
        return true;
    }

    //!
    //! \brief operator <=
    //! \param rhs
    //! \return
    //!
    bool operator <= (const CellData &rhs) const
    {
        return !(this->m_Distance > rhs.m_Distance);
    }

    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const CellData &rhs) const
    {
        if(fabs(this->m_Distance - rhs.m_Distance) > std::numeric_limits<double>::epsilon())
            return false;
        if(this->m_VectorIndex != rhs.m_VectorIndex)
            return false;
        if(this->m_XIndex != rhs.m_XIndex)
            return false;
        if(this->m_YIndex != rhs.m_YIndex)
            return false;
        if(this->m_ObstacleXIndex != rhs.m_ObstacleXIndex)
            return false;
        if(this->m_ObstacleYIndex != rhs.m_ObstacleYIndex)
            return false;
        return true;
    }

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return
    //!
    bool operator != (const CellData &rhs) {
        return !(*this == rhs);
    }

    /** Assignment Operators */
public:

    //!
    //! \brief operator =
    //! \param rhs
    //! \return
    //!
    CellData& operator = (const CellData &rhs)
    {
        this->m_Distance = rhs.m_Distance;
        this->m_VectorIndex = rhs.m_VectorIndex;
        this->m_XIndex = rhs.m_XIndex;
        this->m_YIndex = rhs.m_YIndex;
        this->m_ObstacleXIndex = rhs.m_ObstacleXIndex;
        this->m_ObstacleYIndex = rhs.m_ObstacleYIndex;
        return *this;
    }


double m_Distance = 0.0;
unsigned int m_VectorIndex = 0;
unsigned int m_XIndex = 0 , m_YIndex = 0;
unsigned int m_ObstacleXIndex = 0 , m_ObstacleYIndex = 0;
};

} //end of namespace costmap
} //end of namespace mace
#endif // CELLDATA_H
