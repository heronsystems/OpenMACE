#include "dynamic_2D_grid.h"

namespace mace {
namespace maps {

template <class T>
Dynamic2DGrid<T>::Dynamic2DGrid(const double &x_min, const double &x_max,
                             const double &y_min, const double &y_max,
                             const double &x_res, const double &y_res,
                             const T *fill_value):
    m_dataMap(), m_xMin(0.0), m_yMin(0.0), m_xMax(0.0), m_yMax(0.0), m_xResolution(0.0), m_yResolution(0.0),
    m_xSize(0), m_ySize(0)

{
    setGridSize(x_min,x_max,y_min,y_max,x_res,y_res, fill_value);
}

template <class T>
void Dynamic2DGrid<T>::setGridSize(const double &x_min, const double &x_max, const double &y_min, const double &y_max, const double &x_res, const double &y_res, const T *fill_value)
{
    // Update the internal resolution memebers
    m_xResolution = x_res;
    m_yResolution = y_res;

    // Adjust sizes to adapt them to full sized cells acording to the desired resolution
    m_xMin = m_xResolution * lrint(x_min / m_xResolution);
    m_xMax = m_xResolution * lrint(x_max / m_xResolution);
    m_yMin = m_yResolution * lrint(y_min / m_yResolution);
    m_yMax = m_yResolution * lrint(y_max / m_yResolution);

    // Now the number of cells should be integers:
    m_xSize = round((m_xMax - m_xMin) / m_xResolution) + 1;
    m_ySize = round((m_yMax - m_yMin) / m_yResolution) + 1;

    // Cells memory:
    if (fill_value)
    {
        m_defaultFill = *fill_value;
        m_dataMap.assign(m_xSize * m_ySize, *fill_value);
    }
    else
        m_dataMap.resize(m_xSize * m_ySize);
}

template <class T>
void Dynamic2DGrid<T>::clear()
{
    m_dataMap.clear();
    m_dataMap.resize(m_xSize*m_ySize);
    fill(m_defaultFill);
}

template <class T>
void Dynamic2DGrid<T>::fill(const T &value)
{
    for (typename std::vector<T>::iterator it = m_dataMap.begin(); it != m_dataMap.end(); ++it)
        *it = value;
}

template <class T>
T* Dynamic2DGrid<T>::getCellByIndex(const unsigned int &xIndex, const unsigned int &yIndex) const
{
    if (xIndex >= m_xSize || yIndex >= m_ySize)
        return nullptr;
    else
        return &m_dataMap[xIndex + yIndex * m_xSize];
}

template <class T>
T* Dynamic2DGrid<T>::getCellByPos(const double &x, const double &y) const
{
    int cx = indexFromXPos(x);
    int cy = indexFromYPos(y);
    if (cx < 0 || cx >= static_cast<int>(m_xSize)) return nullptr;
    if (cy < 0 || cy >= static_cast<int>(m_ySize)) return nullptr;
    return &m_dataMap[cx + cy * m_xSize];
}

} //end of namespace maps
} //end of namespace mace
