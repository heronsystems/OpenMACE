#ifndef DYNAMIC_2D_GRID_H
#define DYNAMIC_2D_GRID_H

#include <vector>
#include <cmath>

namespace mace {
namespace maps {

template <class T>
//!
//! \brief The Dynamic2DGrid class
//!
class Dynamic2DGrid
{
public:

    //!
    //! \brief Dynamic2DGrid
    //! \param x_min
    //! \param x_max
    //! \param y_min
    //! \param y_max
    //! \param x_res
    //! \param y_res
    //! \param fill_value
    //!
    Dynamic2DGrid(const double &x_min = -10.0, const double &x_max = 10.0,
                  const double &y_min = -10.0, const double &y_max = 10.0,
                  const double &x_res = 0.5, const double &y_res = 0.5,
                  const T *fill_value = nullptr);

    //!
    //! \brief ~Dynamic2DGrid
    //!
    virtual ~Dynamic2DGrid()
    {

    }

    //!
    //! \brief setGridSize
    //! \param x_min
    //! \param x_max
    //! \param y_min
    //! \param y_max
    //! \param x_res
    //! \param y_res
    //! \param fill_value
    //!
    void setGridSize(const double &x_min, const double &x_max,
                     const double &y_min, const double &y_max,
                     const double &x_res, const double &y_res,
                     const T *fill_value = nullptr);

    //!
    //! \brief clear
    //!
    void clear();

    //!
    //! \brief fill
    //! \param value
    //!
    void fill(const T& value);

    //!
    //! \brief getCellByPos
    //! \param x
    //! \param y
    //! \return
    //!
    T* getCellByPos(const double &x, const double &y) const;

    //!
    //! \brief getCellByIndex
    //! \param xIndex
    //! \param yIndex
    //! \return
    //!
    T* getCellByIndex(const unsigned int &xIndex, const unsigned int &yIndex) const;

    //!
    //! \brief indexFromXPos
    //! \param x
    //! \return
    //!
    int indexFromXPos(const double &x) const
    {
        return static_cast<int>(round((x - m_xMin) / m_xResolution));
    }

    void getPositionFromIndex(const unsigned int &index, double &x, double &y)
    {
        int yIndex = floor(index/m_xSize);
        y = m_yMin + yIndex * m_yResolution;

        int xIndex = (index % m_xSize);
        x = m_xMin + xIndex * m_xResolution;

    }

    //!
    //! \brief indexFromYPos
    //! \param y
    //! \return
    //!
    int indexFromYPos(const double &y) const
    {
        return static_cast<int>(round((y - m_yMin) / m_yResolution));
    }

    //!
    //! \brief indexFromPos
    //! \param x
    //! \param y
    //! \return
    //!
    int indexFromPos(const double &x, const double &y) const
    {
        return indexFromXPos(x) + indexFromYPos(y) * this->m_xSize;
    }

    //!
    //! \brief getSizeX
    //! \return
    //!
    size_t getSizeX() const
    {
        return this->m_xSize;
    }

    //!
    //! \brief getSizeY
    //! \return
    //!
    size_t getSizeY() const
    {
        return this->m_ySize;
    }

    //!
    //! \brief getXMin
    //! \return
    //!
    double getXMin() const
    {
        return this->m_xMin;
    }

    //!
    //! \brief getYMin
    //! \return
    //!
    double getYMin() const
    {
        return this->m_yMin;
    }

    //!
    //! \brief getXMax
    //! \return
    //!
    double getXMax() const
    {
        return this->m_xMax;
    }

    //!
    //! \brief getYMax
    //! \return
    //!
    double getYMax() const
    {
        return this->m_yMax;
    }

    //!
    //! \brief getXResolution
    //! \return
    //!
    double getXResolution() const
    {
        return this->m_xResolution;
    }

    //!
    //! \brief getYResolution
    //! \return
    //!
    double getYResolution() const
    {
        return this->m_yResolution;
    }


    std::vector<T> getDataMap() const
    {
        return this->m_dataMap;
    }

    unsigned int getNodeCount() const
    {
        return m_dataMap.size();
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

    double m_xMin, m_yMin; //!< Description of members
    double m_xMax, m_yMax; //!< Description of members
    double m_xResolution, m_yResolution; //!< Description of members
    size_t m_xSize, m_ySize; //!< Description of members
};

} //end of namespace maps
} //end of namespace mace

#include "dynamic_2D_grid.tpp"

#endif // DYNAMIC_2D_GRID_H
