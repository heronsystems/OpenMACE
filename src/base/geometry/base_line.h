#ifndef BASE_LINE_H
#define BASE_LINE_H

#include <vector>
#include <stdlib.h>

#include "geometry_helper.h"

#include "base/pose/cartesian_position_2D.h"
#include "base/pose/cartesian_position_3D.h"

namespace mace{
namespace geometry{

template <class T>
class LineBase
{
public:
    LineBase(const std::string &descriptor = "Line"):
        name(descriptor)
    {

    }

public:
    virtual void beginLine(const T &obj)
    {
        begin = obj;
    }

    virtual void endLine(const T &obj)
    {
        end = obj;
    }

    virtual T getBeginLine() const
    {
        return begin;
    }

    virtual T getEndLine() const
    {
        return end;
    }

protected:
    std::string name;
    T begin;
    T end;
};

typedef LineBase<mace::pose::CartesianPosition_2D> Line_Cartesian2D;

} //end of namepsace geometry
} //end of namespace mace

#endif // BASE_LINE_H
