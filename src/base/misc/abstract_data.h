#ifndef ABSTRACT_DATA_H
#define ABSTRACT_DATA_H

#include "data_forward_definition.h"

namespace mace {
namespace misc {

namespace details {

template<class POSITIONBASE, class DIMBASE>
struct PositionTypeHelper;

template<class POSITIONBASE>
struct PositionTypeHelper<POSITIONBASE, Data1D>
{
public:
    static const int static_size = 1;
};

template<class POSITIONBASE>
struct PositionTypeHelper<POSITIONBASE, Data2D>
{
public:
    static const int static_size = 2;
};

template<class POSITIONBASE>
struct PositionTypeHelper<POSITIONBASE, Data3D>
{
public:
    static const int static_size = 3;
};


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

#endif // ABSTRACT_DATA_H
