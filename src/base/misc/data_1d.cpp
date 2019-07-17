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
    this->z = copy.z;
    this->dataZFlag = copy.dataZFlag;
}

Data1D::Data1D(const double &z)
{
    this->setData_1D(z);
}

void Data1D::setData_1D(const Data1D &data1D)
{
    this->z = data1D.z;
    this->dataZFlag = data1D.dataZFlag;
}

void Data1D::setData_1D(const double &z)
{
    this->setZ(z);
}

} //end of namespace misc
} //end of namespace mace
