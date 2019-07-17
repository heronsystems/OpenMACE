#include "data_2d.h"

namespace mace {
namespace misc {

Data2D::Data2D():
    Eigen::Vector2d()
{

}

Data2D::~Data2D()
{

}

Data2D::Data2D(const Data2D &copy)
{
    this->x = copy.x;
    this->dataXFlag = copy.dataXFlag;

    this->y = copy.y;
    this->dataYFlag = copy.dataYFlag;
}

Data2D::Data2D(const double &x, const double &y)
{
    this->setData_2D(x,y);
}

void Data2D::setData_2D(const Data2D &data2D)
{
    this->x = data2D.x;
    this->dataXFlag = data2D.dataXFlag;

    this->y = data2D.y;
    this->dataYFlag = data2D.dataYFlag;
}

void Data2D::setData_2D(const double &x, const double &y)
{
    this->setX(x);
    this->setY(y);
}

} //end of namespace misc
} //end of namespace mace
