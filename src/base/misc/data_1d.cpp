#include "data_1d.h"

namespace mace {
namespace misc {

//!
//! \brief Data1D::Data1D
//!
Data1D::Data1D()
{

}

//!
//! \brief Data1D::~Data1D
//!
Data1D::~Data1D()
{

}

Data1D::Data1D(const Data1D &copy)
{
    this->x = copy.x;
    this->dataXFlag = copy.dataXFlag;
}

Data1D::Data1D(const double &z)
{
    this->setData(z);
}

void Data1D::setData(const Data1D &data1D)
{
    this->x = data1D.x;
    this->dataXFlag = data1D.dataXFlag;
}

void Data1D::setData(const double &x)
{
    this->setX(x);
}

} //end of namespace misc
} //end of namespace mace
