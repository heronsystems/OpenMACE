#include "bounded_2D_grid.h"

namespace mace {
namespace maps {

Bounded2DGrid::Bounded2DGrid(const double &x_min, const double &x_max,
                             const double &y_min, const double &y_max,
                             const double &x_res, const double &y_res,
                             const CartesianPosition_2D *fill_value):
    Dynamic2DGrid(x_min, x_max, y_min, y_max, x_res, y_res, fill_value)
{

}

Bounded2DGrid::Bounded2DGrid(const geometry::Polygon_Cartesian &boundingPolygon,
                             const double &x_res, const double &y_res,
                             const CartesianPosition_2D* fill_value):
    Dynamic2DGrid(boundingPolygon.getXMin(),boundingPolygon.getXMax(),
                  boundingPolygon.getYMin(),boundingPolygon.getYMax(),
                  x_res,y_res,fill_value)
{
    setBoundingPolygon(boundingPolygon);
}

std::vector<CartesianPosition_2D*> Bounded2DGrid::setBoundingPolygon(const geometry::Polygon_Cartesian &polygon)
{
    this->clearData();

    this->m_boundary = polygon;
    unsigned int size = getNodeCount();

    for(unsigned int i = 0; i < size; i++)
    {
        double x = 0, y = 0;
        getPositionFromIndex(i,x,y);
        m_dataMap[i].updatePosition(x,y);
        if(m_boundary.contains(x,y,true))
        {
            m_constrainedData.push_back(&m_dataMap[i]);
        }
    }
    return m_constrainedData;
}

std::vector<CartesianPosition_2D*> Bounded2DGrid::getBoundedDataVector() const
{
    return this->m_constrainedData;
}

std::list<CartesianPosition_2D*> Bounded2DGrid::getBoundedDataList() const
{
    std::list<CartesianPosition_2D*> returnList;
    size_t size = m_constrainedData.size();
    for(size_t i = 0; i < size; i++)
    {
        returnList.push_back(m_constrainedData[i]);
    }

    return returnList;
}

void Bounded2DGrid::print() const
{
    size_t size = m_constrainedData.size();
    for(size_t i = 0; i < size; i++)
    {
        std::cout<<"Value at "<<i<<" X:"<<m_constrainedData[i]->getXPosition()<<" Y:"<<m_constrainedData[i]->getYPosition()<<std::endl;
    }
}

void Bounded2DGrid::clearData()
{
    m_constrainedData.clear();
    m_constrainedData.shrink_to_fit();
}

} //end of namespace maps
} //end of namespace mace

