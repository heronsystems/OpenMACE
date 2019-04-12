#include "data_1d.h"

namespace mace {
namespace misc {

Data1D::Data1D(const Data1D &copy)
{
    this->z = copy.z;
    this->dataZFlag = copy.dataZFlag;
}

Data1D::Data1D(const double &z)
{
    this->setData(z);
}

void Data1D::setData(const Data1D &data1D)
{
    this->z = data1D.z;
    this->dataZFlag = data1D.dataZFlag;
}

void Data1D::setData(const double &z)
{
    this->setZ(z);
}

} //end of namespace misc
} //end of namespace mace
