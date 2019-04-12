#ifndef BASE_POSITION_H
#define BASE_POSITION_H

#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include "abstract_position.h"
#include "base/misc/data_2d.h"
#include "base/misc/data_3d.h"

#include "base/math/helper_pi.h"

namespace mace{
namespace pose{


class CartesianPosition
{
public:
    CartesianPosition() = default;

    virtual ~CartesianPosition() = default;
};

class GeodeticPosition
{
public:
    GeodeticPosition() = default;

    virtual ~GeodeticPosition() = default;
};

template<typename T>
class Position : public T
{
public:

    Position() = default;

    ~Position() = default;

    template <typename NEWT>
    Position(const Position<NEWT> &ref):
        T(ref)
    {
        this->name = ref.name;
    }

    template<typename ... Arg>
    Position(const Arg ... arg):
        T(arg ...),
        name("Position Object")
    {
    }

    template<typename ... Arg>
    Position(const char *str, const Arg ... arg):
        T(arg ...),
        name(str)
    {
    }

public:
    std::string getName() const
    {
        return this->name;
    }

public:
    Position& operator = (const Position &copy)
    {
        T::operator=(copy);
        this->name = copy.name;
        return *this;
    }

    bool operator == (const Position &rhs) const
    {
        if(this->name != rhs.name){
            return false;
        }
        return true;
    }

    bool operator !=(const Position &rhs) const
    {
        return !(*this == rhs);
    }


private:
    std::string name;
};



} // end of namespace pose
} // end of namespace mace

#endif // BASE_POSITION_H
