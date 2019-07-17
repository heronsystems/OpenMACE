#ifndef DATA_COMPONENTS_H
#define DATA_COMPONENTS_H

#include "data_1d.h"
#include "data_2d.h"
#include "data_3d.h"

namespace mace {
namespace misc {

namespace details {

template<class DIMBASE>
struct OrientationTypeHelper;

template<>
struct OrientationTypeHelper<Data3D>
{
public:
    static const int static_size = 3;
};

} //end of namespace details

} //end of namespace misc
} //end of namespace mace

#endif // DATA_COMPONENTS_H
